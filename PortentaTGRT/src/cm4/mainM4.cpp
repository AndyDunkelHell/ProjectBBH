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
#include "model_final.h"
#include "test_samples.h" 

extern TwoWire Wire1;
Adafruit_LSM6DS3TRC imu;
bool IMU_board = true; // true if IMU board is present
using namespace std::chrono_literals;

// Statically allocate error‐reporter, resolver, arena, interpreter:
static tflite::MicroErrorReporter     error_reporter;
constexpr int kOpResolverMaxOps = 16;  
static tflite::MicroMutableOpResolver<kOpResolverMaxOps> resolver;
constexpr size_t kTensorArenaSize = 140 * 1024;
uint8_t tensor_arena[kTensorArenaSize]
    __attribute__((section(".bss.$RAM_D2"), aligned(16)));
static const tflite::Model* model = tflite::GetModel(model_StudentGold_Final_tflite);
static tflite::MicroInterpreter* interp;
static TfLiteTensor* input_tensor;
static TfLiteTensor* output_tensor;
bool initInterp = false; // Flag to check if interpreter is initialized

static float window_buf[512][18] __attribute__((section(".axi_window_buf"), aligned(32), used));

static int N_CLASSES = 4; // number of classes in the model

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
//================================================================
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

// Helper to log formatted messages via the TFLM error reporter and Serial with the M7 core
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

// Test run inference on a single test sample (mirrors runInference pipeline)
void run_test_inference(const float sample_data[][TEST_SAMPLE_N_CHANNELS],
                        const char* sample_name,
                        int expected_label) {
  if (!interp || !input_tensor || !output_tensor) {
    SerialRPC.println("ERROR: Interpreter not ready for test inference.");
    return;
  }

  // Expect [1, 512, 18] float32
  const int T = TEST_SAMPLE_WINDOW_SIZE;   // 512
  const int C = TEST_SAMPLE_N_CHANNELS;    // 18
  const int EMG_C = 12;                    // first 12 = EMG

  if (!input_tensor->dims || input_tensor->dims->size != 3 ||
      input_tensor->dims->data[0] != 1 ||
      input_tensor->dims->data[1] != T ||
      input_tensor->dims->data[2] != C ||
      input_tensor->type != kTfLiteFloat32) {
    SerialRPC.println("ERROR: input must be [1,512,18] float32.");
    return;
  }

  // --- scratch buffers (stack/static; no heap) ---
  static float tmp_in [TEST_SAMPLE_WINDOW_SIZE];
  static float tmp_bp [TEST_SAMPLE_WINDOW_SIZE];
  static float tmp_tke[TEST_SAMPLE_WINDOW_SIZE];
  static float tmp_ma [TEST_SAMPLE_WINDOW_SIZE];

  // Same helpers as in runInference()
  auto fir_same = [](const float* x, float* y, int n, const float* h, int m) {
    const int half = (m - 1) / 2;
    for (int t = 0; t < n; ++t) {
      float acc = 0.0f;
      for (int k = 0; k < m; ++k) {
        int xi = t - k + half;
        float xv = (xi >= 0 && xi < n) ? x[xi] : 0.0f;
        acc += h[k] * xv;
      }
      y[t] = acc;
    }
  };
  auto teager = [](const float* x, float* e, int n) {
    if (n == 0) return;
    if (n == 1) { e[0] = 0.0f; return; }
    e[0] = 0.0f;
    for (int t = 1; t < n - 1; ++t) {
      float xt = x[t];
      e[t] = xt * xt - x[t - 1] * x[t + 1];
    }
    e[n - 1] = 0.0f;
  };
  auto movavg = [](const float* x, float* y, int n, int win) {
    if (win <= 1) { for (int i = 0; i < n; ++i) y[i] = x[i]; return; }
    int half = win / 2;
    for (int i = 0; i < n; ++i) {
      int i0 = i - half, i1 = i + (win - half - 1);
      float s = 0.0f;
      for (int j = i0; j <= i1; ++j) if (j >= 0 && j < n) s += x[j];
      y[i] = s / (float)win;
    }
  };

  // 1) Build input features channel-by-channel, then z-score into input tensor
  float* dst = input_tensor->data.f;

  for (int c = 0; c < C; ++c) {
    // Gather channel c from the provided test sample
    for (int t = 0; t < T; ++t) tmp_in[t] = sample_data[t][c];

    const float* feat = nullptr;

    if (c < EMG_C) {
      // EMG: band-pass -> TKE -> moving average (same as runInference)
      #if defined(BP_NUM_TAPS)
        fir_same(tmp_in, tmp_bp, T, BP_TAPS, BP_NUM_TAPS);
      #else
        for (int t = 0; t < T; ++t) tmp_bp[t] = tmp_in[t];
      #endif

      teager(tmp_bp, tmp_tke, T);

      #if defined(MA_WIN)
        movavg(tmp_tke, tmp_ma, T, MA_WIN);
        feat = tmp_ma;
      #else
        feat = tmp_tke;
      #endif
    } else {
      // IMU: no BP/TKE/MA — use raw channel (same as runInference)
      feat = tmp_in;
    }

    // z-score with train-time MU/SIGMA from test_samples.h
    const float mu_c = MU[c];
    const float sg_c = SIGMA[c];
    const float inv_sg = (sg_c != 0.0f) ? (1.0f / sg_c) : 0.0f;

    for (int t = 0; t < T; ++t) {
      dst[t * C + c] = (sg_c != 0.0f) ? (feat[t] - mu_c) * inv_sg : 0.0f;
    }
  }

  // 2) Inference
  unsigned long start_us = micros();
  TfLiteStatus status = interp->Invoke();
  unsigned long dur_us = micros() - start_us;

  if (status != kTfLiteOk) {
    SerialRPC.print("ERROR: Invoke failed for "); SerialRPC.println(sample_name);
    return;
  }

  // 3) Read logits, argmax, print summary
  int num_classes_output = output_tensor->dims->data[output_tensor->dims->size - 1];
  int predicted_class = 0;
  float* out = output_tensor->data.f;
  for (int i = 1; i < num_classes_output; ++i) if (out[i] > out[predicted_class]) predicted_class = i;

  SerialRPC.print("Inference for "); SerialRPC.print(sample_name);
  SerialRPC.print(" took "); SerialRPC.print(dur_us); SerialRPC.println(" us.");
  SerialRPC.print("Logits: [");
  for (int i = 0; i < num_classes_output; ++i) {
    SerialRPC.print(out[i], 6);
    if (i < num_classes_output - 1) SerialRPC.print(", ");
  }
  SerialRPC.println("]");

  SerialRPC.print("Predicted class: "); SerialRPC.print(predicted_class);
  SerialRPC.print(predicted_class == expected_label ? " (Correct!)" : " (Incorrect, expected: ");
  if (predicted_class != expected_label) { SerialRPC.print(expected_label); SerialRPC.print(")"); }
  SerialRPC.println();
  SerialRPC.println("------------------------------------");
}


// Test run inference on a single test sample
void runInference() {
  SerialRPC.print("I");
  if (!interp || !input_tensor || !output_tensor) {
    SerialRPC.println("ERROR: Interpreter not ready.");
    return;
  }

  // Expect [1, 512, 18] float32
  const int T = TEST_SAMPLE_WINDOW_SIZE;   // 512
  const int C = TEST_SAMPLE_N_CHANNELS;    // 18
  const int EMG_C = 12;                    // first 12 = EMG

  if (!input_tensor->dims || input_tensor->dims->size != 3 ||
      input_tensor->dims->data[0] != 1 ||
      input_tensor->dims->data[1] != T ||
      input_tensor->dims->data[2] != C ||
      input_tensor->type != kTfLiteFloat32) {
    SerialRPC.println("ERROR: input must be [1,512,18] float32.");
    return;
  }

  // --- small scratch buffers (stack/static to avoid heap) ---
  static float tmp_in [TEST_SAMPLE_WINDOW_SIZE];
  static float tmp_bp [TEST_SAMPLE_WINDOW_SIZE];
  static float tmp_tke[TEST_SAMPLE_WINDOW_SIZE];
  static float tmp_ma [TEST_SAMPLE_WINDOW_SIZE];

  // Simple helpers
  auto fir_same = [](const float* x, float* y, int n, const float* h, int m) {
    const int half = (m - 1) / 2;
    for (int t = 0; t < n; ++t) {
      float acc = 0.0f;
      for (int k = 0; k < m; ++k) {
        int xi = t - k + half;
        float xv = (xi >= 0 && xi < n) ? x[xi] : 0.0f;
        acc += h[k] * xv;
      }
      y[t] = acc;
    }
  };
  auto teager = [](const float* x, float* e, int n) {
    if (n == 0) return;
    if (n == 1) { e[0] = 0.0f; return; }
    e[0] = 0.0f;
    for (int t = 1; t < n - 1; ++t) {
      float xt = x[t];
      e[t] = xt * xt - x[t - 1] * x[t + 1];
    }
    e[n - 1] = 0.0f;
  };
  auto movavg = [](const float* x, float* y, int n, int win) {
    if (win <= 1) { for (int i = 0; i < n; ++i) y[i] = x[i]; return; }
    int half = win / 2;
    for (int i = 0; i < n; ++i) {
      int i0 = i - half, i1 = i + (win - half - 1);
      float s = 0.0f;
      for (int j = i0; j <= i1; ++j) if (j >= 0 && j < n) s += x[j];
      y[i] = s / (float)win;
    }
  };

  // 1) Build input features channel-by-channel, then z-score into input tensor
  float* dst = input_tensor->data.f;

  for (int c = 0; c < C; ++c) {
    // Gather channel c from live ring/window buffer
    for (int t = 0; t < T; ++t) tmp_in[t] = window_buf[t][c];

    const float* feat = nullptr;

    if (c < EMG_C) {
      // EMG: band-pass -> TKE -> moving average
      #if defined(BP_NUM_TAPS)
        fir_same(tmp_in, tmp_bp, T, BP_TAPS, BP_NUM_TAPS);
      #else
        for (int t = 0; t < T; ++t) tmp_bp[t] = tmp_in[t]; // pass-through if no taps compiled
      #endif

      teager(tmp_bp, tmp_tke, T);

      #if defined(MA_WIN)
        movavg(tmp_tke, tmp_ma, T, MA_WIN);
        feat = tmp_ma;
      #else
        feat = tmp_tke;
      #endif
    } else {
      // IMU: no BP/TKE/MA — use raw channel
      feat = tmp_in;
    }

    // z-score with train-time MU/SIGMA from test_samples.h
    const float mu_c = MU[c];
    const float sg_c = SIGMA[c];
    const float inv_sg = (sg_c != 0.0f) ? (1.0f / sg_c) : 0.0f;

    for (int t = 0; t < T; ++t) {
      dst[t * C + c] = (sg_c != 0.0f) ? (feat[t] - mu_c) * inv_sg : 0.0f;
    }
  }

  // 2) Invoke
  TfLiteStatus status = interp->Invoke();
  if (status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(&error_reporter, "Invoke failed: %d\n", (int)status);
    char msg[32]; int n = snprintf(msg, sizeof(msg), "INVOKE_ERR:%d\n", (int)status);
    SerialRPC.write(msg, n);
    return;
  }

  // 3) Argmax over float32 logits and send class
  int best = 0;
  float* out = output_tensor->data.f;
  for (int i = 1; i < N_CLASSES; ++i) if (out[i] > out[best]) best = i;

  char msg[16]; int n = snprintf(msg, sizeof(msg), "C:%d\n", best);
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
      // rtos::ThisThread::sleep_for(2ms);
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
    window_buf[emg_idx][12] = accel.acceleration.x;
    window_buf[emg_idx][13] = accel.acceleration.y;
    window_buf[emg_idx][14] = accel.acceleration.z;
    window_buf[emg_idx][15] = gyro.gyro.x;
    window_buf[emg_idx][16] = gyro.gyro.y;
    window_buf[emg_idx][17] = gyro.gyro.z;

    // 5) Advance window index and run inference when the window is full
    if (++emg_idx >= 512) {
      emg_idx = 0;
      SerialRPC.print("E");
      runInference(); // Run inference on the complete window
    }
  }
  
}
#endif // CORE_CM4