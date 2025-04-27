#include <SPI.h>
#include "mbed.h"
#include "rtos.h"
#include "events/EventQueue.h" // For Ticker and EventQueue (from the Portenta Arduino core)
#include <Wire.h>
#include "stm32h7xx.h" // STM32 registers
#include "CommandHandler.h"

#define NUM_CHANNELS 12
int CHANNELS[12] = {1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 13, 14};
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
events::EventQueue queue(32 * EVENTS_EVENT_SIZE);
mbed::Ticker sampleTicker;

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

  //------------------------------------------------------------------
  // FLUSH PHASE: Issue dummy conversion commands to fill the pipeline.
  //------------------------------------------------------------------
  if (flushing)
  {
    // Issue a dummy conversion command for the channel at currentChannelIndex.
    SendConvertCommandH(CHANNELS[currentChannelIndex]);
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
  // noNotchFilter(channelIndexToProcess);
  NotchFilter50(channelIndexToProcess);
  // (If you want to use the notch filter instead, call NotchFilter50(channelIndexToProcess);)

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
  for (uint8_t i = 0; i < NUM_CHANNELS; i++)
  {
    serialData = (int)(final_channel_data[i] * 0.195);
    Serial.print(serialData);
    if (i < NUM_CHANNELS - 1)
      Serial.print(", ");
  }
  Serial.println(",550,-550");
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
// Powering channels: using registers 14 and 15 (unchanged)
//================================================================
void SetAllAmpPwr()
{
  uint8_t previousreg14, previousreg15;

  SendReadCommand(14);
  SendReadCommand(14);
  previousreg14 = SendReadCommand(14);
  SendReadCommand(15);
  SendReadCommand(15);
  previousreg15 = SendReadCommand(15);

  int final_channel = CHANNELS[NUM_CHANNELS - 1];

  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
  {
    if (CHANNELS[ch] == final_channel)
    {
      if (CHANNELS[ch] < 8)
      {
        SendWriteCommand(14, (1 << CHANNELS[ch]) | previousreg14);
      }
      else if (CHANNELS[ch] >= 8)
      {
        SendWriteCommand(15, (1 << abs(CHANNELS[ch] - 8)) | previousreg15);
      }
    }
    else
    {
      if (CHANNELS[ch] < 8)
      {
        SendWriteCommand(14, (1 << CHANNELS[ch]) | previousreg14);
        previousreg14 = (1 << CHANNELS[ch]) | previousreg14;
      }
      else if (CHANNELS[ch] >= 8)
      {
        SendWriteCommand(15, (1 << abs(CHANNELS[ch] - 8)) | previousreg15);
        previousreg15 = (1 << abs(CHANNELS[ch] - 8)) | previousreg15;
      }
    }
  }
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

  uint8_t RL = 0, RLDAC1 = 5;
  uint8_t ADCaux3en = 0, RLDAC3 = 0, RLDAC2 = 1;
  uint8_t R12 = ((RL << 7) | RLDAC1);
  uint8_t R13 = (ADCaux3en << 7) | (RLDAC3 << 6) | RLDAC2;
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
// I2C Scanner (unchanged)
//================================================================
void scanI2C()
{
  byte error, address;
  int count = 0;
  Serial.println("Scanning for I2C devices...");
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      count++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (count == 0)
  {
    Serial.println("No I2C devices found.");
  }
  Serial.println("I2C scan complete.");
}

void conn(CommandParameter &Parameters)
{
  Serial.println("Connected");
  startSerial = true;
  // Create a thread for the event queue
  static rtos::Thread eventThread(osPriorityHigh, 16000); // 16KB stack
  eventThread.start(callback(&queue, &events::EventQueue::dispatch_forever));
  Serial.println("Event thread started");

  // Set the sampling ticker to trigger at about 83 microseconds (approx. 12kHz sample rate)
  sampleTicker.attach(timerCallback, std::chrono::microseconds(83));
}

//================================================================
// Setup: initialize SPI, I2C, timers, etc.
//================================================================
void setup()
{
  Serial.begin(250000);
  while (!Serial)
  {
  } // Wait for Serial to initialize
  Serial.println("Starting simplified connection test...");
  testSPIConnection();
  Wire.begin();
  delay(100);
  scanI2C();
  setupCHIP_Timer();

  pinMode(D5, OUTPUT);
  digitalWrite(D5, HIGH);

  SerialCommandHandler.AddCommand(F("connect"), conn);
}

//================================================================
// Main loop: now empty – printing is handled by the event queue.
//================================================================
void loop()
{
  if (!startSerial)
  {
    SerialCommandHandler.Process();
  }
  // Nothing to do here as printing occurs once a full set of samples is ready.
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
