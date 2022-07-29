/**
   --------------------------------------------------------
   This client code is based on the example file of author @RobertoHE incorporated in the BLE-MIDI library.
   I adjusted it for the "Nux Mighty Plug / Air" device and the Heltec ESP 32 WiFi Kit. This version supports four buttons:
   - two main push-buttons (2, 3) to decrement/increment the effect number
   - two addititional push-buttons (1, 4) to set the effect to the lowest/highest value.
   
   Optionally an analog volume pedal can be attached to the solution in order to controö the master volume of the NUX device

   The connection status, battery level, the current effect and the master volume level is shown on the integrated OLED display.

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

// Heltec WiFi Kit ESP32
#include "heltec.h"

// ESP32 Power-Management
#include <esp_adc_cal.h>
#include <driver/adc.h>

// USER DEFINITIONS START
// ======================

// Uncommenr matching device name for NUX Mighty Plug / Nux Mighty Air:
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

// Power Management
// #define MAXBATT                 4200    // The default Lipo is 4200mv when the battery is fully charged.
#define MAXBATT                 4000    // The default Lipo is 4200mv when the battery is fully charged.
// #define LIGHT_SLEEP_VOLTAGE     3750    // Point where start light sleep
#define LIGHT_SLEEP_VOLTAGE     3300    // Point where start light sleep
#define MINBATT                 3200    // The default Lipo is 3200mv when the battery is empty...this WILL be low on the 3.3v rail specs!!!

#define VOLTAGE_DIVIDER         3.20    // Lora has 220k/100k voltage divider so need to reverse that reduction via (220k+100k)/100k on vbat GPIO37 or ADC1_1 (early revs were GPIO13 or ADC2_4 but do NOT use with WiFi.begin())
#define DEFAULT_VREF            1100    // Default VREF use if no e-fuse calibration
#define VBATT_SAMPLE            500     // Battery sample rate in ms
#define VBATT_SMOOTH            50      // Number of averages in sample
#define ADC_READ_STABILIZE      5       // in ms (delay from GPIO control and ADC connections times)
#define LO_BATT_SLEEP_TIME      10*60*1000*1000     // How long when low batt to stay in sleep (us)
#define HELTEC_V2_1             1       // Set this to switch between GPIO13(V2.0) and GPIO37(V2.1) for VBatt ADC.
#define VBATT_GPIO              21      // Heltec GPIO to toggle VBatt read connection ... WARNING!!! This also connects VEXT to VCC=3.3v so be careful what is on header.  Also, take care NOT to have ADC read connection in OPEN DRAIN when GPIO goes HIGH
#define __DEBUG                 0       // DEBUG Serial output

uint16_t voltage;
uint16_t Sample();
void drawBattery(uint16_t, bool = false);
esp_adc_cal_characteristics_t *adc_chars;

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

  // Initalize display
  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);
  // Flip screen display is no longer needed with current Heltec board software and library version 1.1.1
  // Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->clear();
  Heltec.display->drawString(10, 20, "Initialize device ...");
  Heltec.display->display();

  // Initialize power management
  tBatt = millis();
  InitPowerManagement();

  BLEMIDI.setHandleConnected([]()
  {
    Serial.println("---------CONNECTED---------");
    Heltec.display->drawXbm(0, 0, BT_width, BT_height, BT_bits);
    Heltec.display->display();
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
    if (MIDIVolume >=0)
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

      Heltec.display->clear();
      Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
      Heltec.display->setFont(ArialMT_Plain_16);
      Heltec.display->drawString(0, 20, "NUX connected");
      Heltec.display->display();

      delay(1000);

      Heltec.display->clear();
      DisplayEffect(CurrentEffect + 1);
      Heltec.display->display();

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
        Heltec.display->clear();
        Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
        Heltec.display->setFont(ArialMT_Plain_16);
        Heltec.display->drawString(40, 10, "NUX");
        Heltec.display->drawString(10, 30, "disconnected");
        Heltec.display->display();

        delay (1000);
      }

      if (ScanCount == 0) {
        Heltec.display->clear();
        Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
        Heltec.display->setFont(ArialMT_Plain_16);
        Heltec.display->drawString(0, 20, "Scan for device");
      }

      drawBattery(voltage, voltage < LIGHT_SLEEP_VOLTAGE);
      Heltec.display->drawProgressBar(0, 40, 120, 10, ScanCount * 10);
      Heltec.display->display();

      if (ScanCount < 10)
        ScanCount++;
      else
        ScanCount = 0;
    }
  }

  //Loop for battery management
  if ((millis() - tBatt) > 2000) {
    tBatt = millis();
    voltage = Sample();
    if (voltage < MINBATT) {                  // Low Voltage cut off shut down to protect battery as long as possible
      Heltec.display->setColor(WHITE);
      Heltec.display->setFont(ArialMT_Plain_10);
      Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
      Heltec.display->drawString(64, 24, "Shutdown!!");
      Heltec.display->display();
      delay(2000);
#if defined(__DEBUG) && __DEBUG > 0
      Serial.printf(" !! Shutting down...low battery volotage: %dmV.\n", voltage);
      delay(10);
#endif
      esp_sleep_enable_timer_wakeup(LO_BATT_SLEEP_TIME);
      esp_deep_sleep_start();
    } else if (voltage < LIGHT_SLEEP_VOLTAGE) {     // Use light sleep once on battery
      uint64_t s = VBATT_SAMPLE;
#if defined(__DEBUG) && __DEBUG > 0
      Serial.printf(" - Light Sleep (%dms)...battery volotage: %dmV.\n", (int)s, voltage);
      delay(20);
#endif
      esp_sleep_enable_timer_wakeup(s * 1000);   // Light Sleep does not flush buffer
      esp_light_sleep_start();
    }
    delay(ADC_READ_STABILIZE);

    drawBattery(voltage, voltage < LIGHT_SLEEP_VOLTAGE);
    Heltec.display->display();
  }
}

void DisplayEffect(int Effect)
{
  Heltec.display->clear();
  Heltec.display->drawXbm(0, 0, BT_width, BT_height, BT_bits);
  drawBattery(voltage, voltage < LIGHT_SLEEP_VOLTAGE);
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_24);
  Heltec.display->drawString(10, 20, "BANK: " + (String)(Effect));

#ifdef PIN_VOLUME
  if (MIDIVolume >= 0)
    Heltec.display->drawProgressBar(0, 50, 120, 10, map(MIDIVolume, 0, 127, 0, 100));
#endif

  Heltec.display->display();
}

#ifdef PIN_VOLUME
void ReadVolumePedal()
{
  // Values of volume pedal
  static int s_nLastPotValue = 0;
  static int s_nLastMappedValue = 0;

  adcStart(PIN_VOLUME);
  while (adcBusy(PIN_VOLUME));
  int nCurrentPotValue = analogRead(PIN_VOLUME);
  adcEnd(PIN_VOLUME);

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

void InitPowerManagement()
{
  while (! Serial);
  delay(20);

  // Characterize ADC at particular atten
#if (defined(HELTEC_V2_1))
  adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_6);
#else
  // Use this for older V2.0 with VBatt reading wired to GPIO13
  adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  adc2_config_channel_atten(ADC2_CHANNEL_4, ADC_ATTEN_DB_6);
#endif

#if defined(__DEBUG) && __DEBUG > 0
  Serial.printf("ADC Calibration: ");
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.printf("eFuse Vref\n");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    Serial.printf("Two Point\n");
  } else {
    Serial.printf("Default[%dmV]\n", DEFAULT_VREF);
  }
#else
  if (val_type);    // Suppress warning
#endif

#if defined(__DEBUG) && __DEBUG >= 1
  Serial.printf("ADC Calibration: ");
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.printf("eFuse Vref\n");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    Serial.printf("Two Point\n");
  } else {
    Serial.printf("Default[%dmV]\n", DEFAULT_VREF);
  }
#else
  if (val_type);    // Suppress warning
#endif

  // Prime the Sample register
  for (uint8_t i = 0; i < VBATT_SMOOTH; i++) {
    Sample();
  }

  pinMode(VBATT_GPIO, OUTPUT);
  digitalWrite(VBATT_GPIO, LOW);              // ESP32 Lora v2.1 reads on GPIO37 when GPIO21 is low
  // delay(ADC_READ_STABILIZE);                  // let GPIO stabilize

}

// Poll the proper ADC for VBatt on Heltec Lora 32 with GPIO21 toggled
uint16_t ReadVBatt() {
  uint16_t reading = 666;

  digitalWrite(VBATT_GPIO, LOW);              // ESP32 Lora v2.1 reads on GPIO37 when GPIO21 is low
  delay(ADC_READ_STABILIZE);                  // let GPIO stabilize
#if (defined(HELTEC_V2_1))
  pinMode(ADC1_CHANNEL_1, OPEN_DRAIN);        // ADC GPIO37
  reading = adc1_get_raw(ADC1_CHANNEL_1);
  pinMode(ADC1_CHANNEL_1, INPUT);             // Disconnect ADC before GPIO goes back high so we protect ADC from direct connect to VBATT (i.e. no divider)
#else
  pinMode(ADC2_CHANNEL_4, OPEN_DRAIN);        // ADC GPIO13
  adc2_get_raw(ADC2_CHANNEL_4, ADC_WIDTH_BIT_12, &reading);
  pinMode(ADC2_CHANNEL_4, INPUT);             // Disconnect ADC before GPIO goes back high so we protect ADC from direct connect to VBATT (i.e. no divider
#endif

  uint16_t voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
  voltage *= VOLTAGE_DIVIDER;

  return voltage;
}

//  Use a buffer to average/sample ADC
uint16_t Sample() {
  static uint8_t i = 0;
  static uint16_t samp[VBATT_SMOOTH];
  static int32_t t = 0;
  static bool f = true;
  if (f) {
    for (uint8_t c = 0; c < VBATT_SMOOTH; c++) {
      samp[c] = 0;  // Initialize the sample array first time
    } f = false;
  }
  t -= samp[i];   // doing a rolling recording, so remove the old rolled around value out of total and get ready to put new one in.
  if (t < 0) {
    t = 0;
  }

  // ADC read
  uint16_t voltage = ReadVBatt();

  samp[i] = voltage;
#if defined(__DEBUG) && __DEBUG > 0
  Serial.printf("ADC Raw Reading[%d]: %d", i, voltage);
#endif
  t += samp[i];

  if (++i >= VBATT_SMOOTH) {
    i = 0;
  }
  uint16_t s = round(((float)t / (float)VBATT_SMOOTH));
#if defined(__DEBUG) && __DEBUG > 0
  Serial.printf("   Smoothed of %d/%d = %d\n", t, VBATT_SMOOTH, s);
#endif

  return s;
}

void drawBattery(uint16_t voltage, bool sleep) {
  Heltec.display->setColor(BLACK);
  Heltec.display->fillRect(99, 0, 29, 24);

  Heltec.display->setColor(WHITE);
  Heltec.display->drawRect(104, 0, 12, 6);
  Heltec.display->fillRect(116, 2, 1, 2);

  uint16_t v = voltage;
  if (v < MINBATT) {
    v = MINBATT;
  }
  if (v > MAXBATT) {
    v = MAXBATT;
  }
  double pct = map(v, MINBATT, MAXBATT, 0, 100);
  uint8_t bars = round(pct / 10.0);
  Heltec.display->fillRect(105, 1, bars, 4);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
  // Draw small "z" when using sleep
  if (sleep > 0) {
    Heltec.display->drawHorizontalLine(121, 0, 4);
    Heltec.display->drawHorizontalLine(121, 5, 4);
    Heltec.display->setPixel(124, 1);
    Heltec.display->setPixel(123, 2);
    Heltec.display->setPixel(122, 3);
    Heltec.display->setPixel(121, 4);
  }
  Heltec.display->drawString(127, 5, String((int)round(pct)) + "%");
  // Heltec.display->drawString(127, 14, String(round(voltage / 10.0) / 100.0) + "V");
#if defined(__DEBUG) && __DEBUG > 0
  static uint8_t c = 0;
  if ((c++ % 10) == 0) {
    c = 1;
    Serial.printf("VBAT: %dmV [%4.1f%%] %d bars\n", voltage, pct, bars);
  }
#endif
}

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
