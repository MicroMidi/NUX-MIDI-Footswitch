/**
   --------------------------------------------------------
   This example shows how to use client MidiBLE
   Client BLEMIDI works im a similar way Server (Common) BLEMIDI, but with some exception.

   The most importart exception is read() method. This function works as usual, but
   now it manages machine-states BLE connection too. The
   read() function must be called several times continuously in order to scan BLE device
   and connect with the server. In this example, read() is called in a "multitask function of
   FreeRTOS", but it can be called in loop() function as usual.

   Some BLEMIDI_CREATE_INSTANCE() are added in MidiBLE-Client to be able to choose a specific server to connect
   or to connect to the first server which has the MIDI characteristic. You can choose the server by typing in the name field
   the name of the server or the BLE address of the server. If you want to connect
   to the first MIDI server BLE found by the device, you just have to set the name field empty ("").

   FOR ADVANCED USERS: Other advanced BLE configurations can be changed in hardware/BLEMIDI_Client_ESP32.h
   #defines in the head of the file (IMPORTANT: Only the first user defines must be modified). These configurations
   are related to security (password, pairing and securityCallback()), communication params, the device name
   and other stuffs. Modify defines at your own risk.



   @auth RobertoHE
   --------------------------------------------------------
*/

#include <Arduino.h>
#include <avdweb_Switch.h>
#include <BLEMIDI_Transport.h>

#include <hardware/BLEMIDI_Client_ESP32.h>

#include <AdvancedSevenSegment.h>

//#include <hardware/BLEMIDI_ESP32_NimBLE.h>
//#include <hardware/BLEMIDI_ESP32.h>
//#include <hardware/BLEMIDI_nRF52.h>
//#include <hardware/BLEMIDI_ArduinoBLE.h>

BLEMIDI_CREATE_DEFAULT_INSTANCE(); //Connect to first server found

//BLEMIDI_CREATE_INSTANCE("",MIDI)                  //Connect to the first server found
//BLEMIDI_CREATE_INSTANCE("f2:c1:d9:36:e7:6b",MIDI) //Connect to a specific BLE address server
//BLEMIDI_CREATE_INSTANCE("MyBLEserver",MIDI)       //Connect to a specific name server

#ifndef LED_BUILTIN
#define LED_BUILTIN 2 //modify for match with yout board
#endif

// PIN fot toogle on/off LED
#define PIN_LED    4
// PINs for both switches
#define PIN_BUTTON_UP 13
#define PIN_BUTTON_DOWN 12
// Max. 7 effects for the NUX Mighy Plug / Air
#define MAX_EFFECT_COUNT 7

// PINS for the 7-segment LED disply
#define SEG_G 21
#define SEG_F 19
#define SEG_A 18
#define SEG_B 5
#define SEG_E 32
#define SEG_D 33
#define SEG_C 26
#define SEG_DP 27

AdvanceSevenSegment sevenSegment(SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G, SEG_DP);

Switch pushButtonUp = Switch(PIN_BUTTON_UP);
Switch pushButtonDown = Switch(PIN_BUTTON_DOWN);

void ReadCB(void *parameter);       //Continuos Read function (See FreeRTOS multitasks)

unsigned long t0 = millis();
bool isConnected = false;

byte CurrentEffect = 0;
int LEDState = LOW;
bool FirstRun = true;

static NimBLEAdvertisedDevice* advDevice;

/**
   -----------------------------------------------------------------------------
   When BLE is connected, the internal LED will turn on (indicating that connection was successful)
   -----------------------------------------------------------------------------
*/
void setup()
{
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  MIDI.begin(MIDI_CHANNEL_OMNI);

  sevenSegment.clean();

  BLEMIDI.setHandleConnected([]()
  {
    Serial.println("---------CONNECTED---------");
    isConnected = true;
    digitalWrite(LED_BUILTIN, HIGH);
    // Serial.println(advDevice->toString().c_str());
  });

  BLEMIDI.setHandleDisconnected([]()
  {
    Serial.println("---------NOT CONNECTED---------");
    isConnected = false;
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
    sevenSegment.setNumber(CurrentEffect + 1);

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

  //BL: Wait for connection to be established
  //delay(2000);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void loop()
{
  //MIDI.read();  // This function is called in the other task

  if (isConnected && (millis() - t0) > 1000)
  {
    t0 = millis();
    
    if (FirstRun == true) {
      Serial.println("First run");
      FirstRun = false;

      MIDI.sendControlChange(49, 0, 1);
      sevenSegment.setNumber(1);
    }

    pushButtonUp.poll();

    if (pushButtonUp.longPress()) {
      if (LEDState == LOW)
        LEDState = HIGH;
      else
        LEDState = LOW;

      digitalWrite(PIN_LED, LEDState);
      CurrentEffect = 0;

      MIDI.sendControlChange(49, CurrentEffect, 1);
      sevenSegment.setNumber(CurrentEffect + 1);
    }

    if (pushButtonUp.pushed()) {
      if (LEDState == LOW)
        LEDState = HIGH;
      else
        LEDState = LOW;


      digitalWrite(PIN_LED, LEDState);

      if (CurrentEffect < (MAX_EFFECT_COUNT - 1))
        CurrentEffect++;
      else
        CurrentEffect = 0;

      MIDI.sendControlChange(49, CurrentEffect, 1);
      sevenSegment.setNumber(CurrentEffect + 1);
    }

    pushButtonDown.poll();

    if (pushButtonDown.pushed()) {
      if (LEDState == LOW)
        LEDState = HIGH;
      else
        LEDState = LOW;

      digitalWrite(PIN_LED, LEDState);

      if (CurrentEffect > 0)
        CurrentEffect--;
      else
        CurrentEffect = (MAX_EFFECT_COUNT - 1);

      MIDI.sendControlChange(49, CurrentEffect, 1);
      sevenSegment.setNumber(CurrentEffect + 1);
    }
  }
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
