#ifdef CORE_CM4  // M4 core code handles IMU sampling and RPC transmission

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
// #include <tensorflow/lite/version.h>

// Include the TensorFlow Lite model file.
#include "model_tiny.h"
#include "test_samples.h"
// #define Serial SerialRPC 

extern TwoWire Wire1;
Adafruit_LSM6DS3TRC imu;
bool IMU_board = true; // true if IMU board is present
using namespace std::chrono_literals;

// Statically allocate error‐reporter, resolver, arena, interpreter:
static tflite::MicroErrorReporter     error_reporter;
constexpr int kOpResolverMaxOps = 16;  
static tflite::MicroMutableOpResolver<kOpResolverMaxOps> resolver;
constexpr size_t kTensorArenaSize = 120 * 1024;
uint8_t tensor_arena[kTensorArenaSize]
    __attribute__((section(".bss.$RAM_D2"), aligned(16)));
static const tflite::Model* model = tflite::GetModel(model2Dtinyv5_tflite);
static tflite::MicroInterpreter* interp;
static TfLiteTensor* input_tensor;
static TfLiteTensor* output_tensor;
bool initInterp = false; // Flag to check if interpreter is initialized

static float window_buf[512][18];

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
// static rtos::Thread rpcThread(osPriorityHigh, 16 * 1024); 
                                       // 16 KB stack for safety

// Forward declarations
void initInterpreter();
void rpcReceiveTask();

volatile bool M4boardMode = false; // true if M4 is in RPC mode

extern "C" void DebugLog(const char* s) {
  if (Serial) { // Check if Serial has been initialized
    // Serial.print("TFLM_LOG: ");
    SerialRPC.print(s);
  }
}

void LogMessage(const char* format, ...) {
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
  SerialRPC.println("M7: initInterpreter - start");
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize);
  interp = &static_interpreter;
  SerialRPC.println("M7: initInterpreter - interpreter created.");

  SerialRPC.println(uintptr_t(tensor_arena) & 0xF);
  size_t arena_ptr_user  = reinterpret_cast<size_t>(tensor_arena);

  SerialRPC.println("Your arena   @ 0x"); SerialRPC.println(arena_ptr_user, HEX);

  TfLiteStatus alloc_status = interp->AllocateTensors();
  if (alloc_status != kTfLiteOk) {
    LogMessage("AllocateTensors() call failed directly with status code: %d. Arena used bytes: %u\n", 
                    static_cast<int>(alloc_status), 
                    static_cast<unsigned int>(interp->arena_used_bytes()));
    while(1);
  }
  SerialRPC.println(interp->arena_used_bytes());

  input_tensor = interp->input(0); // Get the first input tensor

  if (input_tensor == nullptr) {
    SerialRPC.println("M7: initInterpreter - FATAL ERROR: input_tensor is NULL even after AllocateTensors() succeeded!");
    while(1); // Halt
  }
  SerialRPC.println("M7: initInterpreter - input_tensor pointer obtained successfully.");

  output_tensor = interp->output(0);
  
  if (output_tensor == nullptr) {
    SerialRPC.println("M7: initInterpreter - FATAL ERROR: output_tensor is NULL even after AllocateTensors() succeeded!");
    while(1); // Halt
  }
  initInterp = true; // Interpreter is initialized
}

// Test run inference on a single test sample
void run_test_inference(const float sample_data[][TEST_SAMPLE_N_CHANNELS], const char* sample_name, int expected_label) {
  if (interp == nullptr || input_tensor == nullptr || output_tensor == nullptr) {
    SerialRPC.println("ERROR: Interpreter not ready for test inference.");
    return;
  }

  SerialRPC.print("Running test inference for: ");
  SerialRPC.print(sample_name);
  SerialRPC.print(" (Expected Label: ");
  SerialRPC.print(expected_label);
  SerialRPC.println(")");
    // Inside run_test_inference, before copying to input_tensor
  SerialRPC.print("Sample data check [0][0]: "); SerialRPC.println(sample_data[0][0], 6);
  SerialRPC.print("Sample data check [10][5]: "); SerialRPC.println(sample_data[10][5], 6);
  SerialRPC.print("Sample data check [MAX-1][MAX-1]: "); SerialRPC.println(sample_data[TEST_SAMPLE_WINDOW_SIZE-1][TEST_SAMPLE_N_CHANNELS-1], 6);

  // 1. Copy test sample data to the input tensor
  // Assuming float32 input. If your model is int8 input, this needs to change.
  if (input_tensor->type == kTfLiteFloat32) {
    // Check dimensions
    if (input_tensor->dims->size != 3 || // Should be [1, WINDOW_SIZE, N_CHANNELS]
        input_tensor->dims->data[0] != 1 ||
        input_tensor->dims->data[1] != TEST_SAMPLE_WINDOW_SIZE ||
        input_tensor->dims->data[2] != TEST_SAMPLE_N_CHANNELS) {
      
      SerialRPC.print("ERROR: Test sample dimensions: [1][");
      SerialRPC.print(TEST_SAMPLE_WINDOW_SIZE);
      SerialRPC.print("][");
      SerialRPC.print(TEST_SAMPLE_N_CHANNELS);
      SerialRPC.print("] do not match input tensor: [");
      SerialRPC.print(input_tensor->dims->data[0]);
      SerialRPC.print("][");
      SerialRPC.print(input_tensor->dims->data[1]);
      SerialRPC.print("][");
      SerialRPC.print(input_tensor->dims->data[2]);
      SerialRPC.println("]");
      return;
    }
    // Copy data to input tensor
    memcpy(input_tensor->data.f, sample_data, TEST_SAMPLE_WINDOW_SIZE * TEST_SAMPLE_N_CHANNELS * sizeof(float));

  } else if (input_tensor->type == kTfLiteInt8) {

    SerialRPC.println("ERROR: Input tensor is int8, but test samples are float. Implement quantization for test samples.");
    // TODO: If your model input is int8, you need to quantize sample_data here
    // using input_tensor->params.scale and input_tensor->params.zero_point
    // and ensure test_samples.h provides int8_t data.
    return;
  } else {

    SerialRPC.println("ERROR: Unsupported input tensor type.");
    return;
  }

  SerialRPC.print("tensor[0]  ");  SerialRPC.println(input_tensor->data.f[0], 6);
  SerialRPC.print("tensor[17] ");  SerialRPC.println(input_tensor->data.f[17], 6);
  SerialRPC.print("dims->size = "); SerialRPC.println(input_tensor->dims->size);
  SerialRPC.print("dims        = [");
  for (int i = 0; i < input_tensor->dims->size; ++i) {
    SerialRPC.print(input_tensor->dims->data[i]); SerialRPC.print(i+1 == input_tensor->dims->size ? "]\n" : "][");
}
  // 2. Perform inference
  unsigned long startTime = micros();
  TfLiteStatus invoke_status = interp->Invoke();
  unsigned long duration = micros() - startTime;

  if (invoke_status != kTfLiteOk) {
    SerialRPC.print("ERROR: Invoke failed for ");
    SerialRPC.print(sample_name);
    SerialRPC.print(" Status: ");
    SerialRPC.println(static_cast<int>(invoke_status));
    return;
  }

  SerialRPC.print("Inference for ");
  SerialRPC.print(sample_name);
  SerialRPC.print(" took ");
  SerialRPC.print(duration);
  SerialRPC.println(" microseconds.");

  // Get output tensor and process results
  // Assuming float32 output. If int8, dequantization is needed.
  if (output_tensor->type == kTfLiteFloat32) {
    SerialRPC.print("Output logits for ");
    SerialRPC.print(sample_name);
    SerialRPC.print(": [");
    // Assuming output_tensor->dims->data[0] is batch (should be 1)
    // and output_tensor->dims->data[1] is N_CLASSES
    int num_classes_output = output_tensor->dims->data[output_tensor->dims->size -1]; // Last dimension is num_classes
    if (num_classes_output != N_CLASSES) {
        SerialRPC.print(" WARN: Output tensor classes (");
        SerialRPC.print(num_classes_output);
        SerialRPC.print(") != N_CLASSES (");
        SerialRPC.print(N_CLASSES);
        SerialRPC.print("). Check model. ");
    }

    for (int i = 0; i < num_classes_output; ++i) {
      SerialRPC.print(output_tensor->data.f[i], 6); // Print float output
      if (i < num_classes_output - 1) {
        SerialRPC.print(", ");
      }
    }
    SerialRPC.println("]");

    // Find predicted class
    int predicted_class = -1;
    float max_val = -1000000.0f; // Initialize with a very small number
    for (int i = 0; i < num_classes_output; ++i) {
      if (output_tensor->data.f[i] > max_val) {
        max_val = output_tensor->data.f[i];
        predicted_class = i;
      }
    }
    SerialRPC.print("Predicted class for ");
    SerialRPC.print(sample_name);
    SerialRPC.print(": ");
    SerialRPC.print(predicted_class);
    if (predicted_class == expected_label) {
      SerialRPC.println(" (Correct!)");
    } else {
      SerialRPC.print(" (Incorrect, expected: ");
      SerialRPC.print(expected_label);
      SerialRPC.println(")");
    }

  } else if (output_tensor->type == kTfLiteInt8) {
    // global_error_reporter.Report("Output tensor is int8. Test sample processing needs dequantization.");
    SerialRPC.println("INFO: Output tensor is int8. Implement dequantization to see float values.");
    // TODO: If your model output is int8, you need to dequantize output_tensor->data.int8 here
    // using output_tensor->params.scale and output_tensor->params.zero_point.
    // Then find the predicted class from the dequantized float values.
  } else {
    // global_error_reporter.Report("Unsupported output tensor type for test inference.");
    SerialRPC.println("ERROR: Unsupported output tensor type.");
  }
  SerialRPC.println("------------------------------------");
}
void runInference() {
  SerialRPC.print("I");
  // --- Start: Print Input Tensor Details ---
  if (input_tensor != nullptr) {
    SerialRPC.print("Input Tensor Details:\n"); // Use \n for newline if SerialRPC handles it, otherwise separate println calls

    // Print Tensor Type
    SerialRPC.print("  Type (as int): ");
    SerialRPC.println(static_cast<int>(input_tensor->type)); // kTfLiteFloat32 is 1, kTfLiteInt8 is 3, etc.

    // Print Tensor Bytes (total size)
    SerialRPC.print("  Bytes: ");
    SerialRPC.println(input_tensor->bytes);

    // Print Number of Dimensions
    if (input_tensor->dims != nullptr) {
      SerialRPC.print("  Num Dimensions: ");
      SerialRPC.println(input_tensor->dims->size);

      // Print Each Dimension's Size
      SerialRPC.print("  Dimensions: [");
      for (int i = 0; i < input_tensor->dims->size; ++i) {
        SerialRPC.print(input_tensor->dims->data[i]);
        if (i < input_tensor->dims->size - 1) {
          SerialRPC.print(", ");
        }
      }
      SerialRPC.println("]");
    } else {
      SerialRPC.println("  Dims structure is null.");
    }
  } else {
    SerialRPC.println("Input_tensor is null.");
  }
  SerialRPC.println("--- End: Input Tensor Details ---");
  // --- End: Print Input Tensor Details ---
  memcpy(input_tensor->data.f,
         window_buf,
         sizeof(window_buf));

  // 2) invoke
  TfLiteStatus status = interp->Invoke();
  if (status != kTfLiteOk) {
  TF_LITE_REPORT_ERROR(&error_reporter,
                       "Invoke failed with status: %d\n",
                       static_cast<int>(status));

    char msg[32];
    int n = snprintf(msg, sizeof(msg),
                     "INVOKE_ERR:%d\n", static_cast<int>(status));
    SerialRPC.write(msg, n);
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
  SerialRPC.write(msg, n);
}



void setup() {
  // RPC.begin();           
  // Explicit address 0x6A for LSM6DS3TRC
  if (!imu.begin_I2C(0x6A, &Wire1)) while(1);
  // imu.setAccelRange(LSM6DS3TRC_ACCEL_RANGE_4_G);
  // imu.setGyroRange(LSM6DS3TRC_GYRO_RANGE_500_DPS);
  // RPC.println("M4 and IMU initialized on Wire1 (SDA1/SCL1)");

  Serial.begin(460800);
  while (!Serial) {}
  if (!SerialRPC.begin()) {
    RPC.println("Failed to initialize SerialRPC!");
    // handle error…
  }

  // // 2) now malloc the arena from the heap
  // tensor_arena = (uint8_t*)malloc(kTensorArenaSize);
  // if (!tensor_arena) {
  //   SerialRPC.println("ERROR: arena malloc failed");
  //   while (1) { }  // halt so you see the error
  // }

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

      SerialRPC.println("\n--- Running Inferences on Test Samples ---");
  if (initInterp) { // Check if interpreter is ready
    for (int i = 0; i < NUM_TEST_SAMPLES; ++i) {
      char sample_name_buffer[30]; // Increased buffer size
      sprintf(sample_name_buffer, "Sample %d", i); 
      run_test_inference((const float (*)[TEST_SAMPLE_N_CHANNELS])all_test_samples[i], sample_name_buffer, test_sample_labels[i]);
    }
  } else {
    SerialRPC.println("ERROR: Interpreter not initialized, cannot run test samples.");
  }
  SerialRPC.println("--- Finished Test Sample Inferences ---\n");

  uint8_t ok = 0xAC;
  SerialRPC.write(&ok, 1);



}
int emg_idx = 0;
void loop() {
  sensors_event_t accel, gyro, temp;
  if(IMU_board){
      imu.getEvent(&accel, &gyro, &temp);
      rtos::ThisThread::sleep_for(2ms);
  }
  
  
  PacketHeader hdr;
  EmgPayload  payload;
  if (!M4boardMode)
  {
    if (SerialRPC.available() >= 1) {
      uint8_t code = SerialRPC.read();
      M4boardMode = (code == 0x01);
      // conn();
    }


      int32_t ax_i = (int32_t)(accel.acceleration.x * 1000.0f);
      int32_t ay_i = (int32_t)(accel.acceleration.y * 1000.0f);
      int32_t az_i = (int32_t)(accel.acceleration.z * 1000.0f);
      int32_t gx_i = (int32_t)(gyro.gyro.x * 1000.0f);
      int32_t gy_i = (int32_t)(gyro.gyro.y * 1000.0f);
      int32_t gz_i = (int32_t)(gyro.gyro.z * 1000.0f);

      // Build one line with leading '|' delimiter:
      char buf[80];
      snprintf(buf, sizeof(buf),
              "|%ld,%ld,%ld,%ld,%ld,%ld",
              ax_i, ay_i, az_i, gx_i, gy_i, gz_i);

      // Send in one RPC transaction:
      SerialRPC.println(buf);

      
  }else{
      do {
        if (SerialRPC.readBytes((char*)&hdr.sync, 1) != 1)
          continue;
      } while (hdr.sync != 0xAA);
      SerialRPC.readBytes(((char*)&hdr) + 1, sizeof(hdr) - 1);

      if (hdr.type == 0 && hdr.len == sizeof(EmgPayload)) {
        SerialRPC.readBytes((char*)&payload, sizeof(payload));
      }
      
      // for (int c = 0; c < 12; ++c) {
      //   SerialRPC.print(payload.values[c]);
      //   if (c < 11) SerialRPC.print(",");
      // }
      // SerialRPC.print("|");
      // for (int c = 0; c < 6; ++c) {
      //   SerialRPC.print(accel.acceleration.x * 1000.0f);
      //   if (c < 5) SerialRPC.print(",");
      // }
      // SerialRPC.println();

    //       // 4) Interleave EMG and the fresh IMU data into the window_buf
    for (int c = 0; c < 12; ++c) {
      window_buf[emg_idx][c] = float(payload.values[c]);
    }
    // // Now add the IMU data to the remaining 6 channels
    window_buf[emg_idx][12] = accel.acceleration.x * 1000.0f;
    window_buf[emg_idx][13] = accel.acceleration.y * 1000.0f;
    window_buf[emg_idx][14] = accel.acceleration.z * 1000.0f;
    window_buf[emg_idx][15] = gyro.gyro.x * 1000.0f;
    window_buf[emg_idx][16] = gyro.gyro.y * 1000.0f;
    window_buf[emg_idx][17] = gyro.gyro.z * 1000.0f;

    // 5) Advance window index and run inference when the window is full
    if (++emg_idx >= 512) {
      emg_idx = 0;
      SerialRPC.print("E");
      runInference(); // Run inference on the complete window
    }
  }
  
}
#endif // CORE_CM4