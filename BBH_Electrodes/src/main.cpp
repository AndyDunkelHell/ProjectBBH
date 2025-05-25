#ifdef CORE_CM7  

#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>
#include "RPC.h"
#include "rtos.h"
#include "SerialRPC.h"
#include <Arduino.h>
#include "mbed.h"
#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>

#include "CommandHandler.h"
#include <atomic>
#include <Adafruit_PWMServoDriver.h>

static Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire2);
CommandHandler<10, 90, 15> SerialCommandHandler;

bool startSerial = false;
bool initInterp = false; // true if interpreter is initialized
// Include the TensorFlow Lite model file.
#include "modelnocls.h"
#include "test_samples.h"
// #define Serial SerialRPC 

extern TwoWire Wire1;
Adafruit_LSM6DS3TRC imu;
bool IMU_board = true; // true if IMU board is present
using namespace std::chrono_literals;

// Statically allocate error‐reporter, resolver, arena, interpreter:
static tflite::MicroErrorReporter     error_reporter;
constexpr int kOpResolverMaxOps = 20;  
static tflite::MicroMutableOpResolver<kOpResolverMaxOps> resolver;
constexpr size_t kTensorArenaSize = 150 * 1024;
uint8_t tensor_arena[kTensorArenaSize]
    __attribute__((section(".bss.$RAM_D2"), aligned(16)));
static const tflite::Model* model = tflite::GetModel(model2D_noclsflat_tflite);
static tflite::MicroInterpreter* interp;
static TfLiteTensor* input_tensor;
static TfLiteTensor* output_tensor;

static float window_buf[512][18];
static float  imu_buf[512][6];
static int imu_idx = 0; // index for IMU buffer

const int SERVOMIN = 125;
const int SERVOMAX = 575;
const int SERVONUM = 16;
bool servo_board = false; // true if servo board is present

static int N_CLASSES = 4; // number of classes in the model

struct EmgPacket { 
  int16_t values[12]; 
};

struct PacketHeader {
  uint8_t  sync;     // fixed magic, e.g. 0xAA
  uint8_t  type;     // 0 = EMG, 1 = IMU, 2 = CTRL, …
  uint16_t seq;      // monotonically increasing
  uint16_t len;      // payload length in bytes (so you can vary it)
};

// type-0 payload:
struct EmgPayload {
  int16_t values[12];
};

// Thread handle
static rtos::Thread rpcThread(osPriorityNormal, 16 * 1024); 
                                       // 16 KB stack for safety

// Forward declarations
void initInterpreter();
void rpcReceiveTask();

extern "C" void DebugLog(const char* s) {
  if (Serial) { // Check if Serial has been initialized
    // Serial.print("TFLM_LOG: "); // Add a prefix to distinguish TFLM logs
    Serial.print(s);
    // TF_LITE_REPORT_ERROR usually includes a newline in its format string.
    // If not, add Serial.println() or Serial.print("\n") here.
  }
}
constexpr int   MA_WINDOW = 15;
constexpr float norm_mean[18] = {
 1.7510592e-05f, -6.5103150e-06f,  8.5237180e-06f,  3.6101002e-05f,
  2.5973031e-05f,  4.0530118e-05f,  1.5493080e-05f, 2.5338548e-05f,
  3.8996986e-05f,  2.0694490e-05f,  1.0160831e-05f,  1.0710345e-05f,
  9.9831186e-06f,  6.5045087e-06f, -4.3529295e-07f, -4.4205347e-08f,
  3.8402288e-07f,  2.3899187e-07f};
constexpr float norm_std[18] = {0.9995055f,  1.0012661f,  1.0012894f,  0.9988487f,  0.99982405f, 0.99965686f,
 0.9998318f,  0.99919933f, 1.0000762f,  0.9996321f,  0.9992363f,  1.000127f,
 0.99999887f, 1.0000004f,  0.99999005f, 1.0000004f,  1.0000037f,  1.0000271f};
//––– per‐channel TKE‐MA state:
float win3[12][3] = {0};              // rolling 3‐point buffer
float tke_sum[12] = {0};              // running sum over MA_WINDOW
float tke_hist[12][MA_WINDOW] = {0};  // circular history
int   tke_idx[12] = {0};              // insert ptr per channel

bool inferenceRun = false; // true if inference is running
void processSample(const int16_t raw_emg[12], const float imu[6],
                   float out_feat[18])
{
  // 1) TKE & MA
  for(int ch=0; ch<12; ++ch){
    // shift 3‐point window
    win3[ch][0] = win3[ch][1];
    win3[ch][1] = win3[ch][2];
    win3[ch][2] = raw_emg[ch];
    // compute TKE
    float tke = win3[ch][1]*win3[ch][1]
              - win3[ch][0]*win3[ch][2];
    // subtract oldest, add new
    tke_sum[ch] -= tke_hist[ch][tke_idx[ch]];
    tke_hist[ch][tke_idx[ch]] = tke;
    tke_sum[ch] += tke;
    // advance circular index
    if(++tke_idx[ch] >= MA_WINDOW) tke_idx[ch] = 0;
    // moving average
    float ma = tke_sum[ch] / float(MA_WINDOW);

    // 2) normalize EMG feature
    out_feat[ch] = (ma - norm_mean[ch]) / norm_std[ch];
  }

  // 3) normalize IMU features
  for(int k=0; k<6; ++k){
    out_feat[12 + k] = (imu[k] - norm_mean[12 + k]) / norm_std[12 + k];
  }
}

void LogMyAppMessage(const char* format, ...) {
  if (!Serial) { // Don't try to log if Serial isn't ready
    return;
  }

  char buffer[256]; // Or a larger buffer if you expect very long messages
  va_list args;
  va_start(args, format);
  // Format the string into the buffer
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  // Pass the already formatted string to the TFLM error reporter
  // This will then call your DebugLog("TFLM_LOG: " + formatted_string)
  TF_LITE_REPORT_ERROR(&error_reporter, buffer); 
}

// Call once in setup():
void initInterpreter() {
  Serial.println("M7: initInterpreter - start");
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize);
  interp = &static_interpreter;
  Serial.println("M7: initInterpreter - interpreter created.");
  // interp = new tflite::MicroInterpreter(
  //   model, resolver, tensor_arena, kTensorArenaSize, &error_reporter
  // );
  Serial.println(uintptr_t(tensor_arena) & 0xF);
  size_t arena_ptr_user  = reinterpret_cast<size_t>(tensor_arena);
  // size_t arena_ptr_interp= reinterpret_cast<size_t>(interp->arena());

  Serial.print("Your arena   @ 0x"); Serial.println(arena_ptr_user, HEX);
  // Serial.print("Interp arena @ 0x"); Serial.println(arena_ptr_interp, HEX);
    // interp->SetAllocationInfo(true); 

  TfLiteStatus alloc_status = interp->AllocateTensors();
  if (alloc_status != kTfLiteOk) {
    LogMyAppMessage("AllocateTensors() call failed directly with status code: %d. Arena used bytes: %u\n", 
                    static_cast<int>(alloc_status), 
                    static_cast<unsigned int>(interp->arena_used_bytes()));
    // Serial.println(interp->arena_used_bytes());
    
    // Serial.print("AllocateTensors() failed: ");
    
    while(1);
  }
  Serial.println(interp->arena_used_bytes());
    // 6. Get the input tensor pointer
  input_tensor = interp->input(0); // Get the first input tensor

  if (input_tensor == nullptr) {
    Serial.println("M7: initInterpreter - FATAL ERROR: input_tensor is NULL even after AllocateTensors() succeeded!");
    while(1); // Halt
  }
  Serial.println("M7: initInterpreter - input_tensor pointer obtained successfully.");

  output_tensor = interp->output(0);
  
  if (output_tensor == nullptr) {
    Serial.println("M7: initInterpreter - FATAL ERROR: output_tensor is NULL even after AllocateTensors() succeeded!");
    while(1); // Halt
  }
  initInterp = true; // Interpreter is initialized
}

// New function to run inference on a single test sample
void run_test_inference(const float sample_data[][TEST_SAMPLE_N_CHANNELS], const char* sample_name, int expected_label) {
  if (interp == nullptr || input_tensor == nullptr || output_tensor == nullptr) {
    // global_error_reporter.Report("Interpreter not initialized for test inference!");
    Serial.println("ERROR: Interpreter not ready for test inference.");
    return;
  }

  Serial.print("Running test inference for: ");
  Serial.print(sample_name);
  Serial.print(" (Expected Label: ");
  Serial.print(expected_label);
  Serial.println(")");
    // Inside run_test_inference, before copying to input_tensor
  Serial.print("Sample data check [0][0]: "); Serial.println(sample_data[0][0], 6);
  Serial.print("Sample data check [10][5]: "); Serial.println(sample_data[10][5], 6);
  Serial.print("Sample data check [MAX-1][MAX-1]: "); Serial.println(sample_data[TEST_SAMPLE_WINDOW_SIZE-1][TEST_SAMPLE_N_CHANNELS-1], 6);

  // 1. Copy test sample data to the input tensor
  // Assuming float32 input. If your model is int8 input, this needs to change.
  if (input_tensor->type == kTfLiteFloat32) {
    // Check dimensions
    if (input_tensor->dims->size != 3 || // Should be [1, WINDOW_SIZE, N_CHANNELS]
        input_tensor->dims->data[0] != 1 ||
        input_tensor->dims->data[1] != TEST_SAMPLE_WINDOW_SIZE ||
        input_tensor->dims->data[2] != TEST_SAMPLE_N_CHANNELS) {
      // global_error_reporter.Report("Test sample dimensions mismatch with input tensor!");
      Serial.print("ERROR: Test sample dimensions: [1][");
      Serial.print(TEST_SAMPLE_WINDOW_SIZE);
      Serial.print("][");
      Serial.print(TEST_SAMPLE_N_CHANNELS);
      Serial.print("] do not match input tensor: [");
      Serial.print(input_tensor->dims->data[0]);
      Serial.print("][");
      Serial.print(input_tensor->dims->data[1]);
      Serial.print("][");
      Serial.print(input_tensor->dims->data[2]);
      Serial.println("]");
      return;
    }
    
    // Flatten the 2D sample_data array for memcpy or element-wise copy
      for (int t = 0; t < TEST_SAMPLE_WINDOW_SIZE; ++t) {
        for (int c = 0; c < TEST_SAMPLE_N_CHANNELS; ++c) {
            input_tensor->data.f[t * TEST_SAMPLE_N_CHANNELS + c] = sample_data[t][c];
        }
    }  
    // Or using memcpy if you are sure about layout and sizes:
    // memcpy(input_tensor->data.f, sample_data, TEST_SAMPLE_WINDOW_SIZE * TEST_SAMPLE_N_CHANNELS * sizeof(float));

  } else if (input_tensor->type == kTfLiteInt8) {
    // global_error_reporter.Report("Input tensor is int8. Test sample data is float. Quantization needed for test samples.");
    Serial.println("ERROR: Input tensor is int8, but test samples are float. Implement quantization for test samples.");
    // TODO: If your model input is int8, you need to quantize sample_data here
    // using input_tensor->params.scale and input_tensor->params.zero_point
    // and ensure test_samples.h provides int8_t data.
    return;
  } else {
    // global_error_reporter.Report("Unsupported input tensor type for test inference.");
    Serial.println("ERROR: Unsupported input tensor type.");
    return;
  }

  Serial.print("tensor[0]  ");  Serial.println(input_tensor->data.f[0], 6);
  Serial.print("tensor[17] ");  Serial.println(input_tensor->data.f[17], 6);
  Serial.print("dims->size = "); Serial.println(input_tensor->dims->size);
  Serial.print("dims        = [");
  for (int i = 0; i < input_tensor->dims->size; ++i) {
    Serial.print(input_tensor->dims->data[i]); Serial.print(i+1 == input_tensor->dims->size ? "]\n" : "][");
}
  // 2. Perform inference
  unsigned long startTime = micros();
  TfLiteStatus invoke_status = interp->Invoke();
  unsigned long duration = micros() - startTime;

  if (invoke_status != kTfLiteOk) {
    // ("Invoke failed on %s with status %d", sample_name, static_cast<int>(invoke_status));
    Serial.print("ERROR: Invoke failed for ");
    Serial.print(sample_name);
    Serial.print(" Status: ");
    Serial.println(static_cast<int>(invoke_status));
    return;
  }

  Serial.print("Inference for ");
  Serial.print(sample_name);
  Serial.print(" took ");
  Serial.print(duration);
  Serial.println(" microseconds.");

  // 3. Get output tensor and process results
  // Assuming float32 output. If int8, dequantization is needed.
  if (output_tensor->type == kTfLiteFloat32) {
    Serial.print("Output logits for ");
    Serial.print(sample_name);
    Serial.print(": [");
    // Assuming output_tensor->dims->data[0] is batch (should be 1)
    // and output_tensor->dims->data[1] is N_CLASSES
    int num_classes_output = output_tensor->dims->data[output_tensor->dims->size -1]; // Last dimension is num_classes
    if (num_classes_output != N_CLASSES) {
        Serial.print(" WARN: Output tensor classes (");
        Serial.print(num_classes_output);
        Serial.print(") != N_CLASSES (");
        Serial.print(N_CLASSES);
        Serial.print("). Check model. ");
    }

    for (int i = 0; i < num_classes_output; ++i) {
      Serial.print(output_tensor->data.f[i], 6); // Print float output
      if (i < num_classes_output - 1) {
        Serial.print(", ");
      }
    }
    Serial.println("]");

    // Find predicted class
    int predicted_class = -1;
    float max_val = -1000000.0f; // Initialize with a very small number
    for (int i = 0; i < num_classes_output; ++i) {
      if (output_tensor->data.f[i] > max_val) {
        max_val = output_tensor->data.f[i];
        predicted_class = i;
      }
    }
    Serial.print("Predicted class for ");
    Serial.print(sample_name);
    Serial.print(": ");
    Serial.print(predicted_class);
    if (predicted_class == expected_label) {
      Serial.println(" (Correct!)");
    } else {
      Serial.print(" (Incorrect, expected: ");
      Serial.print(expected_label);
      Serial.println(")");
    }

  } else if (output_tensor->type == kTfLiteInt8) {
    // global_error_reporter.Report("Output tensor is int8. Test sample processing needs dequantization.");
    Serial.println("INFO: Output tensor is int8. Implement dequantization to see float values.");
    // TODO: If your model output is int8, you need to dequantize output_tensor->data.int8 here
    // using output_tensor->params.scale and output_tensor->params.zero_point.
    // Then find the predicted class from the dequantized float values.
  } else {
    // global_error_reporter.Report("Unsupported output tensor type for test inference.");
    Serial.println("ERROR: Unsupported output tensor type.");
  }
  Serial.println("------------------------------------");
}
void runInference() {
  Serial.print("I");
  // --- Start: Print Input Tensor Details ---
  if (input_tensor != nullptr) {
    Serial.print("Input Tensor Details:\n"); // Use \n for newline if Serial handles it, otherwise separate println calls

    // Print Tensor Type
    Serial.print("  Type (as int): ");
    Serial.println(static_cast<int>(input_tensor->type)); // kTfLiteFloat32 is 1, kTfLiteInt8 is 3, etc.

    // Print Tensor Bytes (total size)
    Serial.print("  Bytes: ");
    Serial.println(input_tensor->bytes);

    // Print Number of Dimensions
    if (input_tensor->dims != nullptr) {
      Serial.print("  Num Dimensions: ");
      Serial.println(input_tensor->dims->size);

      // Print Each Dimension's Size
      Serial.print("  Dimensions: [");
      for (int i = 0; i < input_tensor->dims->size; ++i) {
        Serial.print(input_tensor->dims->data[i]);
        if (i < input_tensor->dims->size - 1) {
          Serial.print(", ");
        }
      }
      Serial.println("]");
    } else {
      Serial.println("  Dims structure is null.");
    }
  } else {
    Serial.println("Input_tensor is null.");
  }
  Serial.println("--- End: Input Tensor Details ---");
  // --- End: Print Input Tensor Details ---
  memcpy(input_tensor->data.f,
         window_buf,
         sizeof(window_buf));

  // 2) invoke
  TfLiteStatus status = interp->Invoke();
  if (status != kTfLiteOk) {
    LogMyAppMessage("Invoke failed with status: %d\n", static_cast<int>(status));
    return;
  }

  // 3) send back your predicted class
  float* out = output_tensor->data.f;
  int   best = 0;
  for (int i = 1; i < N_CLASSES; ++i) {
    if (out[i] > out[best]) best = i;
  }
  char msg[16];
  int  n = snprintf(msg, sizeof(msg), "C:%d\n", best);
  Serial.write(msg, n);
}
void rpcReceiveTask() {
  PacketHeader hdr;
  EmgPayload  payload;

  Serial.println("M7: rpcReceiveTask - start");

  while (true) {
    // ——— Wait for valid header ———
    do {
      if (SerialRPC.readBytes((char*)&hdr.sync, 1) != 1)
        continue;
    } while (hdr.sync != 0xAA);
    SerialRPC.readBytes(((char*)&hdr) + 1, sizeof(hdr) - 1);

    if (hdr.type == 0 && hdr.len == sizeof(EmgPayload)) {
      SerialRPC.readBytes((char*)&payload, sizeof(payload));
    }

    // compute index of most recent imu sample
    uint16_t lastImu = (imu_idx + 512 - 1) & 0x01FF;

    if (startSerial) {
      // ——— Streaming mode: print each EMG sample + its IMU, separated by ‘|’ ———
      for (int c = 0; c < 12; ++c) {
        Serial.print(payload.values[c]);
        if (c < 11) Serial.print(",");
      }
      Serial.print("|");
      for (int c = 0; c < 6; ++c) {
        Serial.print(imu_buf[lastImu][c], 6);
        if (c < 5) Serial.print(",");
      }
      Serial.println();

    } else {
      static int win_ptr = 0;
      float feat[18];

      // 1) process raw EMG + latest IMU into a normalized 18-dim feature
      processSample(payload.values, imu_buf[lastImu], feat);

      // 2) store the feature row
      for (int i = 0; i < 18; ++i) {
        window_buf[win_ptr][i] = feat[i];
      }

      // 3) advance pointer & check for a full window
      if (++win_ptr >= 512) {
        if(inferenceRun){runInference();};// one 512×18 window ready
             
        win_ptr = 0;      // wrap around
      }
    }
  }
}


void Disconn(CommandParameter &parameters){

  Serial.println(F("OK"));
  uint8_t code = 0x00;
  SerialRPC.write(&code, 1);
  SerialRPC.flush();
  startSerial = false;

  // sampleTicker.detach();

  
}
  
  
void BBHIdentity(CommandParameter &parameters){
  Serial.println(F("BBH_Portenta \r")); 
  }

  int angleToPulseinv(int ang){
    int pulse = map(ang, 190, 80, SERVOMIN, SERVOMAX);
    return pulse;
  }
  int angleToPulseCMC(int ang){
    int pulse = map(ang, 70, 50, 250, 500);
    return pulse;
  }
  
  int angleToPulse(int ang){
    int pulse = map(ang, 80, 190, SERVOMIN, SERVOMAX);
    return pulse;
  }
 
void I2CInit(){
  byte error, address;
    // ————— Scan secondary I2C bus (Wire1) —————
  int count1 = 0;
  Serial.println("Scanning secondary I2C bus (Wire1) for devices...");
  for (address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
    if (error == 0) {
      Serial.print("Wire1 device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      count1++;
    } else if (error == 4) {
      Serial.print("Wire1 unknown error at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (count1 == 0) Serial.println("No devices found on Wire1.");
  // ————— Scan tertiary I2C bus (Wire2) for PWM driver —————
  Serial.println("Scanning tertiary I2C bus (Wire2) for PWM driver...");
  Wire2.beginTransmission(0x40); // Address of Adafruit PWM Servo Driver
  error = Wire2.endTransmission();
  if (error == 0) {
    Serial.println("Adafruit PWM Servo Driver found at 0x40");
    servo_board = true;
  } else if (error == 4) {
    Serial.println("Wire2 unknown error at 0x40");
  } else {
    servo_board = false;
    Serial.println("No Adafruit PWM Servo Driver found on Wire2.");

  }
}  

void UpdateDeg(CommandParameter &parameters){
  if(!servo_board){
    return;
  }

  int ang0 = parameters.NextParameterAsInteger();
  Serial.print(ang0);
  pwm.setPWM(0,0,angleToPulse(ang0));
  int ang1 = parameters.NextParameterAsInteger();
  pwm.setPWM(1,0,angleToPulse(ang1));
  int ang2 = parameters.NextParameterAsInteger();
  pwm.setPWM(2,0,angleToPulseinv(ang2));
  int ang3 = parameters.NextParameterAsInteger();
  pwm.setPWM(3,0,angleToPulse(ang3));
  int ang4 = parameters.NextParameterAsInteger();
  pwm.setPWM(4,0,angleToPulseinv(ang4));
  int ang5 = parameters.NextParameterAsInteger();
  pwm.setPWM(5,0,angleToPulse(ang5));
  int ang6 = parameters.NextParameterAsInteger();
  pwm.setPWM(6,0,angleToPulse(ang6));
  int ang7 = parameters.NextParameterAsInteger();
  pwm.setPWM(7,0,angleToPulse(ang7));
  int ang8 = parameters.NextParameterAsInteger();
  pwm.setPWM(8,0,angleToPulse(ang8));
  int ang9 = parameters.NextParameterAsInteger();
  pwm.setPWM(9,0,angleToPulse(ang9));
  int ang10 = parameters.NextParameterAsInteger();
  pwm.setPWM(10,0,angleToPulseinv(ang10));
  int ang11 = parameters.NextParameterAsInteger();
  pwm.setPWM(11,0,angleToPulse(ang11));
  int ang12 = parameters.NextParameterAsInteger();
  pwm.setPWM(12,0,angleToPulse(ang12));
  int ang13 = parameters.NextParameterAsInteger();
  pwm.setPWM(13,0,angleToPulseinv(ang13));
  int ang14 = parameters.NextParameterAsInteger();
  pwm.setPWM(14,0,angleToPulse(ang14));
  int ang15 = parameters.NextParameterAsInteger();
  pwm.setPWM(15,0,angleToPulseinv(ang15));
  //Serial.println("g"+String(ang0));
  
}

void connConfirm(CommandParameter &parameters)
{
  Serial.println(F("OK"));
  // Set the sampling ticker to trigger at about 83 microseconds (approx. 12kHz sample rate)
  Serial.println("Ready to receive data");
}


void conn(CommandParameter &parameters)
{
  Serial.println(F("OK"));
  // Set the sampling ticker to trigger at about 83 microseconds (approx. 12kHz sample rate)
  startSerial = true;
  uint8_t code = 0x01;
  SerialRPC.write(&code, 1);
  Serial.println("Ready to receive data");
}

void inferenceSwitch(CommandParameter &parameters)
{
  if (inferenceRun) {
    inferenceRun = false;
    Serial.println(F("Inference stopped"));
  } else {
    inferenceRun = true;
    uint8_t code = 0x01;
    SerialRPC.write(&code, 1);
    Serial.println(F("Inference started"));
  }
}

void setup() {
   // Wait for Serial to be ready
  Serial.begin(250000);
  while (!Serial){}        
  bootM4();


  if (!SerialRPC.begin(250000)) {
    Serial.println("Failed to initialize SerialRPC!");
    // handle error…
  }else {
    Serial.println("SerialRPC initialized successfully!");
  }

  while (true) {
    uint8_t byte = 0;
    // Wait until the byte 0xAC is received

      if (SerialRPC.available() > 0) {
        byte = SerialRPC.read();
        if (byte == 0xAC) {
          Serial.println("Received byte 0xAC, starting M7 I2C init...");
          break; // Exit the loop when the byte is received
        }
        char line = (char)byte;

        // char line = (char)SerialRPC.read();
        Serial.print(line);
      }
      // rtos::ThisThread::sleep_for(1ms);
    
  }
  Serial.println("M7 I2C init started");
  Wire1.begin();
  Wire2.begin(); // SDA2/SCL2 for PWM driver
  delay(100);

    // Explicit address 0x6A for LSM6DS3TRC
    // secondary I2C for IMU on SDA1/SCL1
  if (! imu.begin_I2C(0x6A, &Wire1)) {
      Serial.println("Failed to find LSM6DS3TR-C on Wire1!");
      IMU_board = false;
  }else {
      Serial.println("Found LSM6DS3TR-C on Wire1!");
      IMU_board = true;
  }
  I2CInit();

    // // Core math
    resolver.AddAdd();
    resolver.AddMul();
    resolver.AddSub();
    resolver.AddMean();          // or .AddReduceMean() if int8
    resolver.AddRsqrt();
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
    resolver.AddReshape();
    resolver.AddTranspose();
    resolver.AddSplit();
    resolver.AddPack();
    resolver.AddStridedSlice();
    resolver.AddConcatenation(); 
    resolver.AddTanh();

    resolver.AddSum();

    
    initInterpreter();

      Serial.println("\n--- Running Inferences on Test Samples ---");
  if (initInterp) { // Check if interpreter is ready
    for (int i = 0; i < NUM_TEST_SAMPLES; ++i) {
      char sample_name_buffer[30]; // Increased buffer size
      sprintf(sample_name_buffer, "Sample %d", i); 
      run_test_inference((const float (*)[TEST_SAMPLE_N_CHANNELS])all_test_samples[i], sample_name_buffer, test_sample_labels[i]);
    }
  } else {
    Serial.println("ERROR: Interpreter not initialized, cannot run test samples.");
  }
  Serial.println("--- Finished Test Sample Inferences ---\n");

  SerialCommandHandler.AddCommand(F("connect"), conn);
  SerialCommandHandler.AddCommand(F("connected"), connConfirm);
  SerialCommandHandler.AddCommand(F("DC"), Disconn);
  SerialCommandHandler.AddCommand(F("UD"), UpdateDeg);
  SerialCommandHandler.AddCommand(F("identity"), BBHIdentity);
  SerialCommandHandler.AddCommand(F("inference"), inferenceSwitch);

  // 1) start the RPC task so its stack is carved out first
  rpcThread.start(mbed::callback(rpcReceiveTask));

    // if (IMU_board) {
    //   imuThread.start(mbed::callback(imuReceiveTask));
    // }
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWM(0, 0, SERVOMIN);

}



void loop() {

  SerialCommandHandler.Process();

  
  if(IMU_board){
      sensors_event_t accel, gyro, temp;
      imu.getEvent(&accel, &gyro, &temp);
      imu_buf[imu_idx][0] = accel.acceleration.x;
      imu_buf[imu_idx][1] = accel.acceleration.y;
      imu_buf[imu_idx][2] = accel.acceleration.z;
      imu_buf[imu_idx][3] = gyro.gyro.x;
      imu_buf[imu_idx][4] = gyro.gyro.y;
      imu_buf[imu_idx][5] = gyro.gyro.z;
      imu_idx = (imu_idx + 1) % 512;
      rtos::ThisThread::sleep_for(2ms);
}
}
#endif // CORE_CM7