#ifdef CORE_CM4  // M4 core code handles IMU sampling and RPC transmission

#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>
#include "RPC.h"
#include "rtos.h"
#include "SerialRPC.h"
#include <Arduino.h>
#include "mbed.h"
#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow\lite\micro\tflite_bridge\micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
// #include <tensorflow/lite/version.h>

// Include the TensorFlow Lite model file.
#include "model2Dv3flat.h"
// #define Serial SerialRPC 

extern TwoWire Wire1;
Adafruit_LSM6DS3TRC imu;
bool IMU_board = true; // true if IMU board is present
using namespace std::chrono_literals;

// Statically allocate error‐reporter, resolver, arena, interpreter:
static tflite::MicroErrorReporter     error_reporter;
// constexpr int kOpResolverMaxOps = 18;  
// static tflite::MicroMutableOpResolver<kOpResolverMaxOps> resolver;
static tflite::AllOpsResolver        resolver;
constexpr int kTensorArenaSize = 120 * 1024;
// static uint8_t* tensor_arena = nullptr;
uint8_t tensor_arena[kTensorArenaSize];
static const tflite::Model* model     = tflite::GetModel(model2Dv3flat_tflite);
static tflite::MicroInterpreter* interp;
static TfLiteTensor* input_tensor;
static TfLiteTensor* output_tensor;

static float window_buf[512][18];
static float  imu_buf[512][6];
static int imu_idx = 0; // index for IMU buffer

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
static rtos::Thread rpcThread(osPriorityHigh, 16 * 1024); 
                                       // 16 KB stack for safety

// Forward declarations
void initInterpreter();
void rpcReceiveTask();

volatile bool M4boardMode = true; // true if M4 is in RPC mode


// Call once in setup():
void initInterpreter() {
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize);
  interp = &static_interpreter;
  // interp = new tflite::MicroInterpreter(
  //   model, resolver, tensor_arena, kTensorArenaSize, &error_reporter
  // );
  TfLiteStatus alloc_status = interp->AllocateTensors();
  if (alloc_status != kTfLiteOk) {
    // TF_LITE_REPORT_ERROR(&error_reporter,
    //                     "AllocateTensors() failed: %d\n",
    //                     static_cast<int>(alloc_status));
    SerialRPC.print("AllocateTensors() failed: ");
    while(1);
  }
    // 6. Get the input tensor pointer
  input_tensor = interp->input(0); // Get the first input tensor

  if (input_tensor == nullptr) {
    SerialRPC.println("M4: initInterpreter - FATAL ERROR: input_tensor is NULL even after AllocateTensors() succeeded!");
    while(1); // Halt
  }
  SerialRPC.println("M4: initInterpreter - input_tensor pointer obtained successfully.");

  output_tensor = interp->output(0);
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

void rpcReceiveTask() {

  int emg_idx = 0;
  PacketHeader hdr;
  EmgPayload  payload;

  while (true) {
    // auto free = rpcThread.free_stack();
    // SerialRPC.print("Free stack: ");
    // SerialRPC.print(free);
    // ——— 1) find sync byte ———
    do {
      if (SerialRPC.readBytes((char*)&hdr.sync, 1) != 1) 
        continue; 
    } while (hdr.sync != 0xAA);

    // ——— 2) read rest of header ———
    SerialRPC.readBytes(((char*)&hdr) + 1, sizeof(hdr)-1);

    // ——— 3) validate & read payload ———
    if (hdr.type == 0 && hdr.len == sizeof(EmgPayload)) {
      SerialRPC.readBytes((char*)&payload, sizeof(payload));
    }
    // else{
    //   // unknown packet → skip and resync
    //   SerialRPC.readBytes(nullptr, hdr.len);
    //   continue;
    // }

    // ——— 4) interleave: EMG[0..11] + last IMU[12..17] ———
    for (int c = 0; c < 12; ++c) {
      window_buf[emg_idx][c] = float(payload.values[c]);
    }
    uint16_t lastImu = (imu_idx + 511) & 0x01FF;  // (imu_idx-1) mod 512
    for (int c = 0; c < 6; ++c) {
      window_buf[emg_idx][12 + c] = imu_buf[lastImu][c];
    }

    // ——— 5) advance & inference ———
    if (++emg_idx >= 512) {
      emg_idx = 0;
      SerialRPC.print("E");
      runInference();    // your existing copy→tensor→Invoke→RPC-return
    }
  }

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

  // 1) start the RPC task so its stack is carved out first
  rpcThread.start(mbed::callback(rpcReceiveTask));

  // // 2) now malloc the arena from the heap
  // tensor_arena = (uint8_t*)malloc(kTensorArenaSize);
  // if (!tensor_arena) {
  //   SerialRPC.println("ERROR: arena malloc failed");
  //   while (1) { }  // halt so you see the error
  // }

    // // Core math
    // resolver.AddAdd();
    // resolver.AddSub();
    // resolver.AddMul();
    // resolver.AddMean();
    // resolver.AddMinimum();
    // resolver.AddRsqrt();
    // resolver.AddTanh();

    // // Matrix & gather
    // resolver.AddAdd();
    // resolver.AddSub();
    // resolver.AddMul();
    // resolver.AddRsqrt();
    // resolver.AddTanh();
    // resolver.AddMean();

    // // Tensor manipulation
    // resolver.AddConcatenation();
    // resolver.AddPack();           // for the PACK op
    // resolver.AddReshape();
    // resolver.AddSplit();
    // resolver.AddStridedSlice();
    // resolver.AddTranspose();

    // // Type-conversion
    // resolver.AddCast();


  initInterpreter();


}

void loop() {
  
  if(IMU_board){
    // SerialRPC.println("|1,1,1,1,1,1"); // IMU board is present
    if (!M4boardMode) {
      sensors_event_t accel, gyro, temp;
      imu.getEvent(&accel, &gyro, &temp);
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

      rtos::ThisThread::sleep_for(10ms);
    }else{
      sensors_event_t accel, gyro, temp;
      imu.getEvent(&accel, &gyro, &temp);
      imu_buf[imu_idx][0] = accel.acceleration.x * 0.001f;
      imu_buf[imu_idx][1] = accel.acceleration.y * 0.001f;
      imu_buf[imu_idx][2] = accel.acceleration.z * 0.001f;
      imu_buf[imu_idx][3] = gyro.gyro.x         * 0.001f;
      imu_buf[imu_idx][4] = gyro.gyro.y         * 0.001f;
      imu_buf[imu_idx][5] = gyro.gyro.z         * 0.001f;
      imu_idx = (imu_idx + 1) % 512;
      rtos::ThisThread::sleep_for(10ms);
    }
}else{
    SerialRPC.println("|0,1,0,0,0,0"); // IMU board is not present
  }

}
#endif // CORE_CM4