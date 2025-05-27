#ifdef CORE_CM4  
#include <SPI.h>
#include "mbed.h"
#include "rtos.h"
#include "events/EventQueue.h" // For Ticker and EventQueue (from the Portenta Arduino core)
#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>
#include "RPC.h" 
#include <Adafruit_Sensor.h>
#include "stm32h7xx.h" // STM32 registers
#include <atomic>
#include "SerialRPC.h"

#define NUM_CHANNELS 12
int CHANNELS[12] = {1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 13, 14};

const int chipSelectPin = PIN_SPI_SS;
int serialData = 0;

// Global arrays for raw and filtered data for each channel
volatile int16_t channel_data[NUM_CHANNELS] = {0};
volatile int16_t final_channel_data[NUM_CHANNELS] = {0};
// Buffers for filtering (using float for precision)
float inBuffer[NUM_CHANNELS][3] = {0};
float outBuffer[NUM_CHANNELS][3] = {0};

// Create an EventQueue and a Ticker (from Mbed OS)
events::EventQueue queue(32 * EVENTS_EVENT_SIZE);
mbed::Ticker sampleTicker;

static rtos::Thread eventThread(osPriorityHigh, 16 * 1024);

// Forward declarations of SPI commands and helper functions.
uint16_t SendConvertCommand(uint8_t channelnum);
uint16_t SendReadCommand(uint8_t regnum);
uint16_t SendConvertCommandH(uint8_t channelnum);
uint16_t SendWriteCommand(uint8_t regnum, uint8_t data);

void Calibrate();
void NotchFilter50(uint8_t ch);
void printAllSamples();

bool startSerial = false;

struct PacketHeader {
  uint8_t  sync;     // fixed magic, e.g. 0xAA
  uint8_t  type;     // 0 = EMG, 1 = IMU, 2 = CTRL, …
  uint16_t seq;      // monotonically increasing
  uint16_t len;      // payload length in bytes
};

// type-0 payload:
struct EmgPayload {
  int16_t values[12];
};

static uint16_t seq_counter = 0;
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
//================================================================
// SPI sampling task with pipeline delay handling and queue‐based printing
//================================================================
void spiSampleTask()
{
  // For a 3-command delay:
  const int pipelineDelay = 2;
  // Total number of dummy (flush) commands required:
  const int flushCommands = NUM_CHANNELS + pipelineDelay;

  // Static variables to control flush vs. normal operation.
  static bool flushing = true; // Start in flush mode.
  static int flushCounter = 0; // Count dummy commands issued.

  // Variables for normal operation (after flush is complete):
  static bool pipelineInitialized = false;
  // pipelineQueue will hold channel indices (0 to NUM_CHANNELS-1)
  static uint8_t pipelineQueue[pipelineDelay];
  static uint8_t currentChannelIndex = 0; // Next channel index (0...NUM_CHANNELS-1) to issue a command for.
  static uint8_t sampleCounter = 0;       // Count how many valid samples have been processed.
  // SerialRPC.print("spiSampleTask: flushing=");
  //------------------------------------------------------------------
  // FLUSH PHASE: Issue dummy conversion commands to fill the pipeline.
  //------------------------------------------------------------------
  if (flushing)
  {
    // Issue a dummy conversion command for the channel at currentChannelIndex.
    SendConvertCommand(CHANNELS[currentChannelIndex]);
    // Move to the next channel index (wrap around).
    currentChannelIndex = (currentChannelIndex + 1) % NUM_CHANNELS;
    flushCounter++;
    // When we've issued flushCommands dummy commands, initialize the pipeline.
    if (flushCounter >= flushCommands)
    {
      flushing = false;
      // Fill the pipelineQueue with the channel indices that correspond to the last 'pipelineDelay' commands.
      for (int i = flushCommands - pipelineDelay; i < flushCommands; i++)
      {
        // Instead of storing CHANNELS[i % NUM_CHANNELS],
        // store the channel index (i % NUM_CHANNELS).
        pipelineQueue[i - (flushCommands - pipelineDelay)] = i % NUM_CHANNELS;
      }
      pipelineInitialized = true;
    }
    return; // Don't process any result during flushing.
  }

  // Safety check (should never happen)
  if (!pipelineInitialized)
  {
    return;
  }

  //------------------------------------------------------------------
  // NORMAL OPERATION: Process conversion results in a round-robin manner.
  //------------------------------------------------------------------
  // Issue a conversion command for the current channel.
  uint16_t newResult = SendConvertCommandH(CHANNELS[currentChannelIndex]);

  // The returned result corresponds to the channel at the head of the pipeline.
  uint8_t channelIndexToProcess = pipelineQueue[0];

  // Shift the pipelineQueue one position to the left.
  for (int i = 0; i < pipelineDelay - 1; i++)
  {
    pipelineQueue[i] = pipelineQueue[i + 1];
  }
  // Append the current channel index at the end of the pipeline.
  pipelineQueue[pipelineDelay - 1] = currentChannelIndex;

  // Update currentChannelIndex for next call (wrap around).
  currentChannelIndex = (currentChannelIndex + 1) % NUM_CHANNELS;

  // Store the new conversion result into the proper slot.
  channel_data[channelIndexToProcess] = newResult;
  // Process the raw data – if you're not filtering, use the noNotchFilter.
  noNotchFilter(channelIndexToProcess);
  // NotchFilter50(channelIndexToProcess);

  // Increment the sample counter. When we've processed a full cycle of NUM_CHANNELS samples,
  // schedule printing of the complete set.
  sampleCounter++;
  if (sampleCounter >= NUM_CHANNELS)
  {
    queue.call(printAllSamples);
    sampleCounter = 0;
  }

}


//================================================================
// Print function: prints all channel samples at once.
//================================================================
void printAllSamples()
{
      PacketHeader hdr;
      hdr.sync = 0xAA;
      hdr.type = 0;                     // EMG
      hdr.seq  = seq_counter++;
      hdr.len  = sizeof(EmgPayload);

      EmgPayload payload;
      for (int ch = 0; ch < 12; ch++)
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
  queue.call(spiSampleTask);
}

//================================================================
// Powering channels: using registers 14 and 15
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


  // RL = 0 → internal bias-drive off
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

  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
  {
    SendConvertCommandH(CHANNELS[ch]);
  }
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
// SPI Test
//================================================================
void testSPIConnection()
{
  SerialRPC.println("Starting SPI connection test...");
  SPI.begin();
  SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
  delay(250);
  digitalWrite(chipSelectPin, LOW);
  uint8_t testByte = 0xAA;
  uint8_t response = SPI.transfer(testByte);
  digitalWrite(chipSelectPin, HIGH);
  SerialRPC.print("SPI Test: Sent 0x");
  SerialRPC.print(testByte, HEX);
  SerialRPC.print(", Received 0x");
  SerialRPC.println(response, HEX);
}

//================================================================
// I2C Scanner: scan Wire for devices (Intan Shield)
//================================================================
void scanI2C() {
  byte error, address;

  // ————— Scan primary I2C bus (Wire) —————
  int count0 = 0;
  SerialRPC.println("Scanning primary I2C bus (Wire) for devices...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      SerialRPC.print("Wire device found at 0x");
      if (address < 16) Serial.print("0");
      SerialRPC.println(address, HEX);
      count0++;
    } else if (error == 4) {
      SerialRPC.print("Wire unknown error at 0x");
      if (address < 16) Serial.print("0");
      SerialRPC.println(address, HEX);
    }
  }
  if (count0 == 0) SerialRPC.println("No devices found on Wire.");

  SerialRPC.println("I2C scan complete.");
  
  uint8_t ok = 0xAC;
  SerialRPC.write(&ok, 1);
  

}

void conn()
{
  SerialRPC.println("Connected");
  if(!startSerial){
      SerialRPC.println(F("OK"));
      sampleTicker.detach();
  }else{
    // Set the sampling ticker to trigger at about 83 microseconds (approx. 12kHz sample rate)
    sampleTicker.attach(timerCallback, std::chrono::microseconds(83));
    SerialRPC.print(startSerial);
    SerialRPC.println("Ticker attached, sampling started.");
  }

}

//================================================================
// Setup: initialize SPI, I2C, timers, etc.
//================================================================
void setup()
{

  Serial.begin(250000);
  while (!Serial) {}
  if (!SerialRPC.begin()) {
    RPC.println("Failed to initialize SerialRPC!");
  }

  SerialRPC.println("Starting M4 connection test...");
  delay(100);
  testSPIConnection();
  Wire.begin();
  delay(100);
    
  scanI2C();
  setupCHIP_Timer();
  pinMode(D5, OUTPUT);
  digitalWrite(D5, HIGH);
  

  // Create a thread for the event queue
  static rtos::Thread eventThread(osPriorityHigh, 16000); // 16KB stack
  eventThread.start(callback(&queue, &events::EventQueue::dispatch_forever));
}

//================================================================
// Main loop: Expecting a single byte to start the serial connection and sending EMG data
//================================================================
void loop()
{
  if (!startSerial)
  {
    if (SerialRPC.available() >= 1) {
      uint8_t code = SerialRPC.read();
      startSerial = (code == 0x01);
      conn();
    }
  }
}

//================================================================
// SendConvertCommand: unchanged
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


#endif // CORE_CM4

