#ifdef CORE_CM7  // M7 core code handles Intan SPI and RPC reception
#include <SPI.h>
#include "mbed.h"
#include "rtos.h"
#include "events/EventQueue.h" // For Ticker and EventQueue (from the Portenta Arduino core)
#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>
#include "RPC.h" 
#include <Adafruit_Sensor.h>
#include "stm32h7xx.h" // STM32 registers
#include "CommandHandler.h"
#include <atomic>
#include <SerialRPC.h>
#include <Adafruit_PWMServoDriver.h>

#define NUM_CHANNELS 2


int CHANNELS[12] = {1,2};

// Alternatively, you could change the order with:
// int CHANNELS[12] = {11, 12, 13, 14, 8, 7, 6, 5, 4, 3, 2, 1};

const int chipSelectPin = PIN_SPI_SS;
int serialData = 0;

// Global arrays for raw and filtered data for each channel
volatile int16_t channel_data[NUM_CHANNELS] = {0};
volatile int16_t final_channel_data[NUM_CHANNELS] = {0};
// Buffers for filtering (using float for precision)
float inBuffer[NUM_CHANNELS][3] = {0};
float outBuffer[NUM_CHANNELS][3] = {0};

// Create an EventQueue and a Ticker (from Mbed OS)
events::EventQueue queue(256 * EVENTS_EVENT_SIZE);
mbed::Ticker sampleTicker;

static rtos::Thread eventThread(osPriorityHigh, 16 * 1024);
static rtos::Thread imuThread(osPriorityNormal, 4 * 1024);

// IMU handling on CM7: lock‑free ring buffer for incoming samples
struct IMUSample { int32_t ax, ay, az, gx, gy, gz; };
constexpr size_t IMU_BUFFER_SIZE = 32;
IMUSample imuBuffer[IMU_BUFFER_SIZE];
std::atomic<size_t> imuHead(0), imuTail(0);

// IMU instance on secondary I2C bus (Wire1)
extern TwoWire Wire1;
Adafruit_LSM6DS3TRC imu;
bool IMU_board = false;
  

// BoardMode: true for EMG+IMU data collection, false for prediction Mode
volatile bool boardMode = false; // Flag for EMG mode

extern TwoWire Wire2; // I2C bus for the PWM driver (Adafruit_PWMServoDriver)
bool servo_board = false;
// Forward declarations of SPI commands and helper functions.
uint16_t SendConvertCommand(uint8_t channelnum);
uint16_t SendReadCommand(uint8_t regnum);
uint16_t SendConvertCommandH(uint8_t channelnum);
uint16_t SendWriteCommand(uint8_t regnum, uint8_t data);
void Calibrate();
void NotchFilter50(uint8_t ch);
void printAllSamples();

CommandHandler<10, 90, 15> SerialCommandHandler;

bool startSerial = false;

static Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire2);


const int SERVOMIN = 125;
const int SERVOMAX = 575;
const int SERVONUM = 16;

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


static uint16_t seq_counter = 0;

using namespace std::chrono_literals;


//================================================================
// Notch filter (unchanged)
//================================================================
void NotchFilter50(uint8_t ch)
{
  // Shift previous inputs
  inBuffer[ch][0] = inBuffer[ch][1];
  inBuffer[ch][1] = inBuffer[ch][2];
  inBuffer[ch][2] = channel_data[ch];

  // Apply the IIR notch filter equation
  outBuffer[ch][2] = 0.9696f * inBuffer[ch][0] - 1.8443f * inBuffer[ch][1] + 0.9696f * inBuffer[ch][2] - 0.9391f * outBuffer[ch][0] + 1.8442f * outBuffer[ch][1];

  // Shift previous outputs
  outBuffer[ch][0] = outBuffer[ch][1];
  outBuffer[ch][1] = outBuffer[ch][2];

  // Store the filtered result
  final_channel_data[ch] = outBuffer[ch][2];
}

void noNotchFilter(uint8_t ch)
{
  final_channel_data[ch] = channel_data[ch];
}

  // Static variables to control flush vs. normal operation.
  // static bool flushing = true; // Start in flush mode.
  // static int flushCounter = 0; // Count dummy commands issued.
//================================================================
// SPI sampling task with pipeline delay handling and queue‐based printing
//================================================================
// Index for the command being sent in THIS call.
static uint8_t commandChannelIndex = 0;
// Counter for how many valid results we have received in this cycle.
static uint8_t resultsReceivedCount = 0;

volatile int i = 0; 
void spiSampleTask(){

  	//Read the channel data from whichever sample in the pipeline corresponds to this 1/2000th of a second
  if (i == 0) {
		channel_data[0] = SendConvertCommand(CHANNELS[i]);
	}

	if (i == 1) {
		channel_data[1] = SendConvertCommand(CHANNELS[i]);

	}

  final_channel_data[i] = channel_data[i];

  if (i == 1)
{		//If we just read the data from the SECONDCHANNEL, read FIRSTCHANNEL on the next iteration
		i = 0;
    queue.call(printAllSamples);}
	else
{		//If we just read the data from the FIRSTCHANNEL, read SECONDCHANNEL on the next iteration
		i++;}

}


//-----------------------------------------------------------------------------
// Thread to receive ASCII IMU lines from M4 over RPC and push into ring buffer
//-----------------------------------------------------------------------------  
void imuReceiveTask() {
  static char buf[80];
  size_t idx = 0;
  // int32_t       predicted = -1;

  while (true) {
    if(boardMode){
      
      if (SerialRPC.available()) {
        
        char line = (char)SerialRPC.read();
        // Debug echo of raw characters:
        Serial.print(line);
        // SerialRPC.readBytes((char*)&predicted, sizeof(predicted));

        // Serial.print(F("Predicted class: "));
        // Serial.println(predicted);
        
      }
      continue;
    }
      if (SerialRPC.available()) {
          char line = (char)SerialRPC.read();
          // Debug echo of raw characters:
          // Serial.print(line);
          // On newline, process a complete record
          if (line == '\n') {
              // Null-terminate and only accept lines that start with '|'
              buf[idx] = '\0';
              if (idx > 0 && buf[0] == '|') {
                  IMUSample sample;
                  // Skip the '|' at buf[0]
                  if (sscanf(buf + 1,
                             "%ld,%ld,%ld,%ld,%ld,%ld",
                             &sample.ax, &sample.ay, &sample.az,
                             &sample.gx, &sample.gy, &sample.gz) == 6) {
                      // Enqueue into lock-free FIFO
                      size_t head = imuHead.load();
                      size_t next = (head + 1) % IMU_BUFFER_SIZE;
                      imuBuffer[head] = sample;
                      imuHead.store(next);
                      // Debug:
                      // // Serial.println(sample.ax);
                      // Serial.println(buf + 1); // Print the whole line (excluding '|')
                      // If buffer full, advance tail (drop oldest)
                      if (next == imuTail.load()) {
                          imuTail.store((imuTail.load() + 1) % IMU_BUFFER_SIZE);
                      }
                  }
              }
              // Reset buffer for next line
              idx = 0;
          } else {
              // Accumulate character (if it fits)
              if (idx < sizeof(buf) - 1) {
                  buf[idx++] = line;
              }
          }
      } else {
          continue;
      }
  }
}

//================================================================
// Print function: prints all channel samples at once.
//================================================================
void printAllSamples()
{
  // EMG+IMU sampling mode
  if(!boardMode){
    // Serial.print("ELEC,");
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
      serialData = (final_channel_data[i] * 0.195);
      SerialUSB.print(serialData);
      if (i < NUM_CHANNELS - 1)
        SerialUSB.print(",");
    }
      // Append IMU data from ring buffer or previous sample
      Serial.print("|");
      static IMUSample prevSample = {0,0,0,0,0,0};
      size_t tail = imuTail.load();
      size_t head = imuHead.load();

      IMUSample s;
      if (tail != head) {
          // New sample available
          s = imuBuffer[tail];
          imuTail.store((tail + 1) % IMU_BUFFER_SIZE);
          prevSample = s;  // Update fallback sample
      } else {
          // Use last-seen sample when buffer empty
          s = prevSample;
      }

      // Serialize and print s (six scaled ints)
      char imuBuf[120];
      snprintf(imuBuf, sizeof(imuBuf),
              "%ld,%ld,%ld,%ld,%ld,%ld",
              s.ax, s.ay, s.az,
              s.gx, s.gy, s.gz);
      SerialUSB.println(imuBuf);  // All in one atomic call

      return;

    }
    // Prediction mode: buffer 512 EMG samples

      PacketHeader hdr;
      hdr.sync = 0xAA;
      hdr.type = 0;                     // EMG
      hdr.seq  = seq_counter++;
      hdr.len  = sizeof(EmgPayload);

      EmgPayload payload;
      for (int ch = 0; ch < NUM_CHANNELS; ch++)
        payload.values[ch] = final_channel_data[ch];

      // write header + payload in one go:
      SerialRPC.write((uint8_t*)&hdr,     sizeof(hdr));
      SerialRPC.write((uint8_t*)&payload, sizeof(payload));

}


//================================================================
// SPI command functions (unchanged)
//================================================================
uint16_t SendReadCommand(uint8_t regnum)
{
  uint16_t mask = regnum << 8;
  mask = 0b1100000000000000 | mask;
  digitalWrite(chipSelectPin, LOW);
  SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
  uint16_t out = SPI.transfer16(mask);
  SPI.endTransaction();
  digitalWrite(chipSelectPin, HIGH);
  return out;
}

uint16_t SendConvertCommandH(uint8_t channelnum)
{
  uint16_t mask = channelnum << 8;
  mask = 0b0000000000000001 | mask;
  digitalWrite(chipSelectPin, LOW);
  SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
  uint16_t out = SPI.transfer16(mask);
  SPI.endTransaction();
  digitalWrite(chipSelectPin, HIGH);
  return out;
}

uint16_t SendWriteCommand(uint8_t regnum, uint8_t data)
{
  uint16_t mask = regnum << 8;
  mask = 0b1000000000000000 | mask | data;
  digitalWrite(chipSelectPin, LOW);
  SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
  uint16_t out = SPI.transfer16(mask);
  SPI.endTransaction();
  digitalWrite(chipSelectPin, HIGH);
  return out;
}

void Calibrate()
{
  digitalWrite(chipSelectPin, LOW);
  SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
  SPI.transfer16(0b0101010100000000);
  SPI.endTransaction();
  digitalWrite(chipSelectPin, HIGH);
  for (int i = 0; i < 9; i++)
  {
    SendReadCommand(40);
  }
  
}

//================================================================
// Timer callback: posts the sampling task to the event queue.
//================================================================
void timerCallback()
{
  // queue.call(spiSampleTask);
  bool ok = queue.call(spiSampleTask);
  if (!ok) digitalWrite(LED_BUILTIN, HIGH);
}

//================================================================
// Powering channels: using registers 14 and 15 (unchanged)
//================================================================

void SetAllAmpPwr()
{
  // — Read & flush register 14 twice, then grab its current value
  SendReadCommand(14);
  SendReadCommand(14);
  uint8_t mask14 = SendReadCommand(14);

  // — Read & flush register 15 twice, then grab its current value
  SendReadCommand(15);
  SendReadCommand(15);
  uint8_t mask15 = SendReadCommand(15);

  // — Always power reference electrodes 0 and 15
  mask14 |= (1 << 0);         // channel 0
  mask15 |= (1 << (15 - 8));  // channel 15

  // — Now power your active channels (those in CHANNELS[], which already excludes 0 & 15)
  for (uint8_t i = 0; i < NUM_CHANNELS; i++)
  {
    uint8_t ch = CHANNELS[i];
    if (ch < 8)
      mask14 |= (1 << ch);
    else
      mask15 |= (1 << (ch - 8));
  }

  // — Write back exactly once per register
  SendWriteCommand(14, mask14);
  SendWriteCommand(15, mask15);
}


//================================================================
// CHIP Timer setup and register initialization (mostly unchanged)
//================================================================
void setupCHIP_Timer()
{
  SendWriteCommand(0, 0b11011110);
  SendWriteCommand(1, 0b00100000);
  SendWriteCommand(2, 0b00101000);
  SendWriteCommand(3, 0b00000000);
  SendWriteCommand(4, 0b11011000);
  SendWriteCommand(5, 0b00000000);
  SendWriteCommand(6, 0b00000000);
  SendWriteCommand(7, 0b00000000);
  SendWriteCommand(8, 30);
  SendWriteCommand(9, 5);
  SendWriteCommand(10, 43);
  SendWriteCommand(11, 6);


  // RL = 0 → internal bias-drive off (we’re using an external reference electrode)
  // RLDAC1 = 0 → no DAC output on Jack 1
  uint8_t RL       = 0;
  uint8_t RLDAC1   = 0;

  // ADCaux3en = 0 → don’t enable the aux ADC onboard
  // RLDAC3   = 0 → no DAC output on Jack 3
  // RLDAC2   = 0 → no DAC output on Jack 2
  uint8_t ADCaux3en = 0,
          RLDAC3    = 0,
          RLDAC2    = 0;

  // build the two bytes exactly as the datasheet wants:
  uint8_t R12 = (RL << 7) | (RLDAC1 & 0x7F);
  uint8_t R13 = (ADCaux3en << 7) | (RLDAC3 << 6) | (RLDAC2 << 5);

  // write them out to the Intan
  SendWriteCommand(12, R12);
  SendWriteCommand(13, R13);
  SendWriteCommand(14, 0b00000000);
  SendWriteCommand(15, 0b00000000);

  SetAllAmpPwr();

  Calibrate();

  // for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
  // {
  //   SendConvertCommandH(CHANNELS[ch]);
  // }
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
  {
    SendConvertCommand(CHANNELS[ch]);
  }

  Wire.begin();
  Wire.beginTransmission(56);
  Wire.write(0b11110000);
  Wire.write(0b00001100);
  Wire.endTransmission();
}

//================================================================
// SPI Test (unchanged)
//================================================================
void testSPIConnection()
{
  Serial.println("Starting SPI connection test...");
  SPI.begin();
  SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
  delay(250);
  digitalWrite(chipSelectPin, LOW);
  uint8_t testByte = 0xAA;
  uint8_t response = SPI.transfer(testByte);
  digitalWrite(chipSelectPin, HIGH);
  Serial.print("SPI Test: Sent 0x");
  Serial.print(testByte, HEX);
  Serial.print(", Received 0x");
  Serial.println(response, HEX);
}

//================================================================
// I2C Scanner: scan both Wire and Wire1 buses
//================================================================
void scanI2C() {
  byte error, address;

  // ————— Scan primary I2C bus (Wire) —————
  int count0 = 0;
  Serial.println("Scanning primary I2C bus (Wire) for devices...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Wire device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      count0++;
    } else if (error == 4) {
      Serial.print("Wire unknown error at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (count0 == 0) Serial.println("No devices found on Wire.");

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

  Serial.println("I2C scan complete.");

}

void conn(CommandParameter &Parameters)
{
  // Serial.println("Connected");
  startSerial = true;
  SendConvertCommand(CHANNELS[0]);
  SendConvertCommand(CHANNELS[1]);
  // Set the sampling ticker to trigger at about 83 microseconds (approx. 12kHz sample rate)
  sampleTicker.attach(timerCallback, std::chrono::microseconds(1000));
  // Serial.println("Ticker attached, sampling started.");

}

void Disconn(CommandParameter &parameters){

  Serial.println(F("OK"));

  sampleTicker.detach();

  
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

void modeSwitch(CommandParameter &parameters)
{
  if (boardMode)
  {
    boardMode = false;
    Serial.println(F("EMG+IMU sampling mode"));
    uint8_t code = 0x00;
    SerialRPC.write(&code, 1);
    
  }
  else
  {
    boardMode = true;
    uint8_t code = 0x01;
    Serial.println(F("Prediction mode"));
    SerialRPC.write(&code, 1);
    sampleTicker.attach(timerCallback, std::chrono::microseconds(125));
  }
}

//================================================================
// Setup: initialize SPI, I2C, timers, etc.
//================================================================
void setup()
{
  // Serial.begin(1000000);
  pinMode(LED_BUILTIN, OUTPUT);
  SerialUSB.begin(1000000);
  while (!SerialUSB)
  {} // Wait for Serial to initialize
  Serial.println("Starting simplified connection test...");
  testSPIConnection();
  Wire.begin();
  Wire1.begin();
  Wire2.begin(); // SDA2/SCL2 for PWM driver
  delay(100);
    // secondary I2C for IMU on SDA1/SCL1
    if (! imu.begin_I2C(0x6A, &Wire1)) {
      Serial.println("Failed to find LSM6DS3TR-C on Wire1!");
      IMU_board = false;
    }else {
      Serial.println("Found LSM6DS3TR-C on Wire1!");
      IMU_board = true;
    }
    
  scanI2C();
  setupCHIP_Timer();
  pinMode(D5, OUTPUT);
  digitalWrite(D5, HIGH);
  bootM4();  

  if (!SerialRPC.begin(460800)) {
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
          Serial.flush();
          break; // Exit the loop when the byte is received
        }
        char line = (char)byte;
        Serial.print(line);
      }
    
  }

  // Create a thread for the event queue
  // static rtos::Thread eventThread(osPriorityHigh, 16000); // 16KB stack
  eventThread.start(callback(&queue, &events::EventQueue::dispatch_forever));
  // usbWriteThread.start(usbWriteTask);
  // Start background thread to fetch IMU data from M4
  // static rtos::Thread imuThread(osPriorityNormal, 4*1024);
  if (IMU_board) {
    imuThread.start(mbed::callback(imuReceiveTask));
  }
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWM(0, 0, SERVOMIN);
          
  // RPC.begin();
  SerialCommandHandler.AddCommand(F("connect"), conn);
  SerialCommandHandler.AddCommand(F("connected"), connConfirm);
  SerialCommandHandler.AddCommand(F("DC"), Disconn);
  SerialCommandHandler.AddCommand(F("UD"), UpdateDeg);
  SerialCommandHandler.AddCommand(F("identity"), BBHIdentity);
  SerialCommandHandler.AddCommand(F("mode"), modeSwitch);
}


//================================================================
// Main loop: now empty – printing is handled by the event queue.
//================================================================
void loop()
{
  // if (!startSerial)
  // {
  SerialCommandHandler.Process();
  // }

}

//================================================================
// SendConvertCommand: unchanged (basic conversion command)
//================================================================
uint16_t SendConvertCommand(uint8_t channelnum)
{
  uint16_t mask = channelnum << 8;
  digitalWrite(chipSelectPin, LOW);
  SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
  uint16_t out = SPI.transfer16(mask);
  SPI.endTransaction();
  digitalWrite(chipSelectPin, HIGH);
  return out;
}


#endif // CORE_CM7

