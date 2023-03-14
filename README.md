# NUX-MIDI-Footswitch
# BLE MIDI Footswitch Project for the NUX Mighty Plug / NUX Mighty Plug Pro (beta) / Mighty Air
[![Github All Releases download count](https://img.shields.io/github/downloads/MicroMidi/NUX-MIDI-Footswitch/total?style=flat-square)](https://github.com/MicroMidi/NUX-MIDI-Footswitch/releases)

This project is intended to build a simple wireless footswitch for the NUX Mighty Plug / Mighty Plug Pro (beta) / Mighty Air based on the ESP32 microcontroller with integrated Bluetooth functionality. The footswitch operates wireless and interacts via Bluetooth Low Energy (BLE) and MIDI commands directly with the NUX device.

There are currently three variants of the code:
- Generic ESP32 module with 7-segment LED-display: This variant is based on a generic ESP32 microcontroller and implements a wireless footswitch with two buttons and a seven segment display for showing the selected effect on the NUX device. In the current version the code supports two buttons: The first button is implemented to increment the effect-number, the second button to decrement the effect-number. A long press on the increment-button sets the effect back to the first effect of the NUX device. A seven segment LED-display shows the current effect that is selected, another optional LED toggles on/off with every button-press event.

- Heltec ESP32 WiFi-Kit with integrated OLED display and integrated management for an attached LiPo battery: This variant is based on the versatile Heltec ESP32 WiFi-Kit  (https://heltec.org/project/wifi-kit-32/) which has already an OLED display attached and supports voltage- and charge-management for an external LiPo-battery that can be easily attached to the Heltec module. This version uses two switches to increment and decrement the effects, the other two buttons set the effect to the lowest and highest effect bank. The OLED display shows the selected effect, the level of the volume pedal (optional), the battery charge level and the connection status. If you want to adjust the master volume of the NUX device with an external analog effect pedal the code now supports reading the analog input of effect pedal and translates it to MIDI commands for the master volume level of the NUX device. This is a (very useful an comfortable) option - but it can be easily deactivated in the code if pure effect switching is needed.

- Generic ESP32 module with external OLED display: This variant is based on a generic ESP32 microcontroller and  implements a wirelesss footswitch with four buttons, an optional analog volume pedal and an external SSD1306 OLED display. This version also uses two switches to increment and decrement the effects, the other two buttons set the effect to the lowest and highest effect bank. The OLED display shows the selected effect, the level of the volume pedal (optional) and the connection status. This variant also supports as an option the connection of an external analog pedal to control the master volume level of the NUX device

The code in all variants is able to handle MIDI effect-switching-events sent from the NUX Mighty device to synchronize the selected effect between footswitch and NUX device. This ensures that the indicated effect of the footswitch and the NUX Mighty device are always in sync - even if you switch the selected effects at the NUX device.

# Included libraries
The client code for both versions uses the powerful BLE-MIDI library from @lathoub (https://github.com/lathoub/Arduino-BLE-MIDI) which leverages the NimBLE-Bluetooth library (https://github.com/h2zero/NimBLE-Arduino) with an extraordinary low memory footprint and a very reliable operation. MIDI communication is implemented by the MIDI library of Francois Best (https://github.com/FortySevenEffects/arduino_midi_library). All these libraries can be found in the Arduino IDE library manager. In addition to this you need the following library for the switch buttons, which can be downloaded here: https://github.com/avandalen/avdweb_Switch

If you want to display the selected effect at the footswitch on a seven-segment display you need to include this library: https://www.arduino.cc/reference/en/libraries/advance-seven-segment/.

For the Heltec OLED version you need to install the Heltec board software and the Heltec ESP32 Dev-Board library (https://github.com/HelTecAutomation/Heltec_ESP32). Everything you need to know about this task can be found here: https://heltec-automation-docs.readthedocs.io/en/latest/esp32/quick_start.html
Please make sure that you use the most recent version of the board software and the corresponding heltec library!

The generic ESP32 variant with an external OLED display leverages the brilliant ThingPulse library for OLED displays (https://github.com/ThingPulse/esp8266-oled-ssd1306) that needs to be incorporated with the Arduino library manager. Please make sure that you have installed the most recent board software for your ESP32 module!

# Hardware requirements
To build the seven segment footswitch hardware on your own you really do not need many components: Mandatory is an ESP32 microcontroller with two push buttons for effect-switching. Optional is a LED 7-segement display with 8 resistors for each LED-segment. Another optional LED connected with a resistor to the ESP32 indicates a button-press-event by toggling on/off. Power supply can be implemented by an USB-cable or an external battery module.

Even less components are needed for implementing the Heltec OLED version: The Heltec ESP32 WiFi-Kit module, four push buttons, a micro USB cable and a case - that's all. This solution can be enhanced by rechargeable LiPo-battery which will be charged via a connected USB cable. A JST-1.25 battery connector and the hardware for charge management are included in the Heltec board. For an optional volume pedal you need an 6,3mm stereo jack connector that matches to an analog effect pedal.

For the variant with a generic ESP32 module in combination with an external OLED display you need any kind of an ESP32 microcontroller, an SSD1306 OLED display with I2C-interface, four push buttons, a micro USB cable and a case. This variant needs external power supply via USB-connection as no battery support is incorporated in the solution. For an optional volume pedal you also need the 6,3mm stereo jack connector that matches to an analog effect pedal.

This is what the front view of the Heltec ESP32 variant looks like (credits go to Mark Duffill for providing me with such a beautiful and functional enclosure):
<img src="https://github.com/MicroMidi/NUX-MIDI-Footswitch/blob/main/images/Nux-Footswitch-Volume-Front.jpg">

The back view of the Heltec ESP32 variant is shown in the following image:
<img src="https://github.com/MicroMidi/NUX-MIDI-Footswitch/blob/main/images/Nux-Footswitch-Volume-Back.jpg">

This is the breadboard prototype for the generic ESP32 variant with external OLED display:
<img src="https://github.com/MicroMidi/NUX-MIDI-Footswitch/blob/main/images/Nux-Footswitch-GenericESP32.jpg">

# Wiring scheme for Heltec based solution with volume pedal
The following image shows a schematic overview of the wiring from the four push buttons and the analog volume pedal to the Heltec ESP32 module:

<img src="https://github.com/MicroMidi/NUX-MIDI-Footswitch/blob/main/images/Nux-Footswitch-Volume-Wiring-Scheme.jpg">

You can use the PINs that are defined in the demo code or use other PINs of your choice that support digital or analog input signals. Please make sure that the 	positive pole of volume pedal is attached to the 3.3V PIN and NOT to the 5.0V PIN of the ESP32 - otherwise it might destroy the analog-/digtial-converter!

# Wiring scheme for generic ESP32 based solution with external OLED display and volume pedal
The following image shows a schematic overview of the wiring from the OLED display, the four push buttons and the analog volume pedal to the ESP32 device:

<img src="https://github.com/MicroMidi/NUX-MIDI-Footswitch/blob/main/images/Nux-Footswitch-Volume-Wiring-Scheme-ESP32.jpg">

Again, the choice of the selected PINs is up to you and depends on the ESP32 module that you are using. The PINs of the push buttons must support a digital input with pull-up functionality, the PIN for the volume pedal should support an analog input and the OLED display must be connected to the PINs with the I2C-interface of your ESP32 module. 

# Plans for the future
If I find the time I will design a tiny PCB that hosts the ESP32 module and the external components for the seven segment variant. The Heltec OLED version just needs the four push buttons and and optional analog effect pedal connected to the board - so there is no need for an PCB board. The ESP32 variant just needs additional wiring of the OLED display unit.

Stay tuned for further development...
