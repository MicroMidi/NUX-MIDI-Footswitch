/**
   --------------------------------------------------------
   This client code is based on the example file of author @RobertoHE incorporated in the BLE-MIDI library.
   I adjusted it for the "Nux Mighty Plug / Air" device and generic ESP32 devices with an attached SSD1306 OLED display unit with 128x64 resolution connected via I2C interface.
   
   This version supports four buttons:
   - two main push-buttons (2, 3) to decrement/increment the effect number
   - two addititional push-buttons (1, 4) to set the effect to the lowest/highest value.

   Optionally an analog volume pedal can be attached to the solution in order to control the master volume of the NUX device

   The connection status, the current effect and the master volume level is shown on the external OLED display.

   (authors: @RobertoHE/@MicroMidi)
   --------------------------------------------------------
*/

#include <Arduino.h>

// Bluetooth MIDI libraries
#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_Client_ESP32.h>
//#include <hardware/BLEMIDI_ESP32_NimBLE.h>
//#include <hardware/BLEMIDI_ESP32.h>
//#include <hardware/BLEMIDI_nRF52.h>
//#include <hardware/BLEMIDI_ArduinoBLE.h>

// AVD switch library
#include <avdweb_Switch.h>

// ThingPulse SSD1306 library
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`

// USER DEFINITIONS START
// ======================

// Uncomment matching device name for NUX Mighty Plug / Nux Mighty Air:
// NUX Mighty Plug
#define MIDI_DEVICE_NAME "NUX MIGHTY PLUG MIDI"
// NUX Mighty Air
// #define MIDI_DEVICE_NAME "NUX MIGHTY AIR MIDI"

// Define PINs for all push buttons
#define PIN_BUTTON_1 32
#define PIN_BUTTON_2 33
#define PIN_BUTTON_3 12
#define PIN_BUTTON_4 13

// Comment out PIN_VOLUME definition if NO volume pedal is attached to the solution
#define PIN_VOLUME 36     // PIN for Volume Pedal

// OLED PIN definitions
// For SDA-/SCL-PINs are Arduino default values are used
// Adapt the following two values and uncomment the defintions if you need customized values
// #define SDA 21
// #define SCL 22
// Uncomment this defintion if your SSD1306 device supports a reset PIN
// #define OLED_RESET 16

// ======================
// USER DEFINITIONS END


#ifndef LED_BUILTIN
#define LED_BUILTIN 25 //modify for match with yout board
#endif

#define MAX_EFFECT_COUNT 7

#ifdef PIN_VOLUME
// Constants for volume pedal
const int POT_THRESHOLD = 3;            // Threshold amount to guard against false values
const int HISTORY_BUFFER_LENGTH = 10;    // History buffer length
// (to guard against noise being sent)

// Globals for volume pedal
static int s_history[HISTORY_BUFFER_LENGTH];
int MIDIVolume = -1;
#endif

Switch pushButton1 = Switch(PIN_BUTTON_1); // set effect number to lowest value
Switch pushButton2 = Switch(PIN_BUTTON_2); // decrement effect
Switch pushButton3 = Switch(PIN_BUTTON_3); // increment effect
Switch pushButton4 = Switch(PIN_BUTTON_4); // set effect number to highest value

void ReadCB(void *parameter);       //Continuos Read function (See FreeRTOS multitasks)

unsigned long t0 = millis();
unsigned long tBatt = millis();
unsigned long tSearchDevice = millis();
int ScanCount = 0;

bool isConnected = false;

byte CurrentEffect = 0;
int LEDState = LOW;
bool FirstRun = true;
bool FirstRunDisconnect = false;

// BT-symbol
const unsigned char BT_bits[] PROGMEM = {
  0x18, 0x28, 0x4A, 0x2C, 0x18, 0x2C, 0x4A, 0x28, 0x18, 0x00,
};
#define BT_width 8
#define BT_height 10

// Initalize display
// Initialize the OLED display using Wire library
SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL

// Connect to the NUX BLE-MIDI service
BLEMIDI_CREATE_INSTANCE(MIDI_DEVICE_NAME, MIDI)       //Connect to a specific name server
//BLEMIDI_CREATE_DEFAULT_INSTANCE(); //Connect to first server found
//BLEMIDI_CREATE_INSTANCE("",MIDI)                  //Connect to the first server found
//BLEMIDI_CREATE_INSTANCE("f2:c1:d9:36:e7:6b",MIDI) //Connect to a specific BLE address server

/**
   -----------------------------------------------------------------------------
  When BLE is connected, the internal LED will turn on (indicating that connection was successful)
   -----------------------------------------------------------------------------
*/

void setup()
{
  Serial.begin(115200);
  MIDI.begin(MIDI_CHANNEL_OMNI);

#ifdef PIN_VOLUME
  // Initialize ADC-input for volume pedal
  adcAttachPin(PIN_VOLUME);

  // Initialize he buffer
  for (int i = 0; i < HISTORY_BUFFER_LENGTH; i++)
  {
    s_history[i] = -1;
  }
#endif

// Reset SSD1306 display unit if necessary
#ifdef OLED_RESET
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW); // set reset PIN to LOW to reset SSD1306 display unit
  delay(50);
  digitalWrite(OLED_RESET, HIGH); // while SSD1306 display unit is running, the reset PIN must be set to HIGH
#endif

  // Setup display parameters
  display.init();
  display.flipScreenVertically();
  display.setContrast(255);
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.clear();
  display.drawString(10, 20, "Initialize device ...");
  display.display();

  BLEMIDI.setHandleConnected([]()
  {
    Serial.println("---------CONNECTED---------");
    display.drawXbm(0, 0, BT_width, BT_height, BT_bits);
    display.display();
    isConnected = true;
    FirstRunDisconnect = true;
    digitalWrite(LED_BUILTIN, HIGH);
    // Serial.println(advDevice->toString().c_str());
    t0 = millis();
  });

  BLEMIDI.setHandleDisconnected([]()
  {
    Serial.println("---------NOT CONNECTED---------");
    isConnected = false;
    FirstRun = true;
    CurrentEffect = 0;
    tSearchDevice = millis();
    ScanCount = 0;
    digitalWrite(LED_BUILTIN, LOW);
  });

  MIDI.setHandleControlChange([](byte channel, byte ControlNumber, byte ControlValue)
  {
    Serial.print("ControlChange: CH: ");
    Serial.print(channel);
    Serial.print(" | ");
    Serial.print(ControlNumber);
    Serial.print(", ");
    Serial.println(ControlValue);

    CurrentEffect = ControlValue;
    DisplayEffect(CurrentEffect + 1);

#ifdef PIN_VOLUME
    // Adjust MIDI volume to pedal level
    if (MIDIVolume >= 0)
      MIDI.sendControlChange(81, MIDIVolume, 1);
#endif
  });

  xTaskCreatePinnedToCore(ReadCB,           //See FreeRTOS for more multitask info
                          "MIDI-READ",
                          3000,
                          NULL,
                          1,
                          NULL,
                          1); //Core0 or Core1

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void loop()
{
  //MIDI.read();  // This function is called in the other task

  if (isConnected) {

    if (FirstRun == true && (millis() - t0) > 200) {
      FirstRun = false;

      Serial.print("millis(): ");
      Serial.println(millis());
      Serial.print("t0: ");
      Serial.println(t0);
      Serial.println("First run");

      vTaskDelay(250 / portTICK_PERIOD_MS);
      MIDI.sendControlChange(49, 0, 1);

      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 20, "NUX connected");
      display.display();

      delay(1000);

      display.clear();
      DisplayEffect(CurrentEffect + 1);
      display.display();

#ifdef PIN_VOLUME
      MIDIVolume = -1;
#endif
    }

    pushButton1.poll();
    if (pushButton1.pushed()) {

      CurrentEffect = 0;

      MIDI.sendControlChange(49, CurrentEffect, 1);
      DisplayEffect(CurrentEffect + 1);

#ifdef PIN_VOLUME
      // Adjust MIDI volume to pedal level
      if (MIDIVolume >= 0)
        MIDI.sendControlChange(81, MIDIVolume, 1);
#endif
    }

    pushButton2.poll();
    if (pushButton2.pushed()) {

      if (CurrentEffect > 0)
        CurrentEffect--;
      else
        CurrentEffect = (MAX_EFFECT_COUNT - 1);

      MIDI.sendControlChange(49, CurrentEffect, 1);
      DisplayEffect(CurrentEffect + 1);

#ifdef PIN_VOLUME
      // Adjust MIDI volume to pedal level
      if (MIDIVolume >= 0)
        MIDI.sendControlChange(81, MIDIVolume, 1);
#endif
    }

    pushButton3.poll();
    if (pushButton3.pushed()) {

      if (CurrentEffect < (MAX_EFFECT_COUNT - 1))
        CurrentEffect++;
      else
        CurrentEffect = 0;

      MIDI.sendControlChange(49, CurrentEffect, 1);
      DisplayEffect(CurrentEffect + 1);

#ifdef PIN_VOLUME
      // Adjust MIDI volume to pedal level

      if (MIDIVolume >= 0)
        MIDI.sendControlChange(81, MIDIVolume, 1);
#endif
    }

    pushButton4.poll();
    if (pushButton4.pushed()) {

      CurrentEffect = MAX_EFFECT_COUNT - 1;

      MIDI.sendControlChange(49, CurrentEffect, 1);
      DisplayEffect(CurrentEffect + 1);

#ifdef PIN_VOLUME
      // Adjust MIDI volume to pedal level
      if (MIDIVolume >= 0)
        MIDI.sendControlChange(81, MIDIVolume, 1);
#endif
    }

#ifdef PIN_VOLUME
    // Loop for reading volume pedal
    ReadVolumePedal();
#endif

  }
  // Loop for disconnected device
  else {
    if ((millis() - tSearchDevice) > 1000) {
      tSearchDevice = millis();
      if (FirstRunDisconnect == true) {
        FirstRunDisconnect = false;
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setFont(ArialMT_Plain_16);
        display.drawString(40, 10, "NUX");
        display.drawString(10, 30, "disconnected");
        display.display();

        delay (1000);
      }

      if (ScanCount == 0) {
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 20, "Scan for device");
      }

      display.drawProgressBar(0, 40, 120, 10, ScanCount * 10);
      display.display();

      if (ScanCount < 10)
        ScanCount++;
      else
        ScanCount = 0;
    }
  }
}

void DisplayEffect(int Effect)
{
  display.clear();
  display.drawXbm(0, 0, BT_width, BT_height, BT_bits);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24);
  display.drawString(10, 20, "BANK: " + (String)(Effect));

#ifdef PIN_VOLUME
  if (MIDIVolume >= 0)
    display.drawProgressBar(0, 50, 120, 10, map(MIDIVolume, 0, 127, 0, 100));
#endif

  display.display();
}

#ifdef PIN_VOLUME
void ReadVolumePedal()
{
  // Values of volume pedal
  static int s_nLastPotValue = 0;
  static int s_nLastMappedValue = 0;

  int nCurrentPotValue = analogRead(PIN_VOLUME);

  if (abs(nCurrentPotValue - s_nLastPotValue) < POT_THRESHOLD)
    return;
  s_nLastPotValue = nCurrentPotValue;

  int nMappedValue = map(nCurrentPotValue, 0, 4095, 0, 127);    // Map the value to 0-127

  if (nMappedValue == s_nLastMappedValue)
    return;

  for (int i = 0; i < HISTORY_BUFFER_LENGTH; i++)
  {
    if (s_history[i] == nMappedValue)
      return;
  }

  memcpy(&s_history[0], &s_history[1], sizeof(int) * (HISTORY_BUFFER_LENGTH - 1));
  s_history[HISTORY_BUFFER_LENGTH - 1] = nMappedValue;
  s_nLastMappedValue = nMappedValue;
  Serial.print("MIDI VolumeChange: ");
  Serial.println(nMappedValue);
  MIDI.sendControlChange(81, nMappedValue, 1);
  MIDIVolume = nMappedValue;
  DisplayEffect(CurrentEffect + 1);
}
#endif

/**
   This function is called by xTaskCreatePinnedToCore() to perform a multitask execution.
   In this task, read() is called every millisecond (approx.).
   read() function performs connection, reconnection and scan-BLE functions.
   Call read() method repeatedly to perform a successfull connection with the server
   in case connection is lost.
*/
void ReadCB(void *parameter)
{
  //  Serial.print("READ Task is started on core: ");
  //  Serial.println(xPortGetCoreID());
  for (;;)
  {
    MIDI.read();
    vTaskDelay(1 / portTICK_PERIOD_MS); //Feed the watchdog of FreeRTOS.
    //Serial.println(uxTaskGetStackHighWaterMark(NULL)); //Only for debug. You can see the watermark of the free resources assigned by the xTaskCreatePinnedToCore() function.
  }
  vTaskDelay(1);
}
