# NUX-MIDI-Footswitch
# BLE MIDI Footswitch Project for the NUX Mighty Plug / Mighty Air

This project is intended to build a simple wireless footswitch for the NUX Mighty Plug / Mighty Air based on the ESP32 microcontroller with integrated Bluetooth functionality. There are currently two variants of the code:
- A footswitch with two buttons and a seven segment display. In the current version the code supports two buttons: The first button is implemented to increment the effect-number, the second button to decrement the effect-number. A long press on the increment-button sets the effect back to the first effect of the NUX device. A seven segment LED-display shows the current effect that is selected, another optional LED toggles on/off with every button-press event.
- A footswitch with four buttons and an OLED display based on the Helect ESP 32 Wi-Fi kit (https://heltec.org/project/wifi-kit-32/). This version uses two switches to increment and decrement the effects, the other two buttons set the effect to the lowest and highest value. The display shows the connection status, battery level and the connection status.


The code is able to handle MIDI effect-switching-events sent from the NUX Mighty device to be able to synchronize the selected effect between footswitch and NUX device. This ensures that the indicated effect of the footswitch and the NUX Mighty device are always in sync - even if you switch the selected effects at the NUX device.

The client code for both versions uses the powerful BLE-MIDI library from @lathoub (https://github.com/lathoub/Arduino-BLE-MIDI) which leverages the NimBLE-Bluetooth library (https://github.com/h2zero/NimBLE-Arduino) with an extraordinary low memory footprint and a very reliable operation. MIDI communication is implemented by the MIDI library of Francois Best (https://github.com/FortySevenEffects/arduino_midi_library). All these libraries can be found in the Arduino IDE library manager. In addition to this you need the following library for the switch buttons, which can be downloaded here: https://github.com/avandalen/avdweb_Switch

If you want to display the selected effect at the footswitch on a seven-segment display you need to include this library: https://www.arduino.cc/reference/en/libraries/advance-seven-segment/. For the OLED version you need to install the Heltec board software and the Heltec ESP32 Dev-Board library (https://github.com/HelTecAutomation/Heltec_ESP32).

To build the seven segment footswitch hardware on your own you really do not need many components: Mandatory is an ESP32 microcontroller with two push buttons for effect-switching. Optional is a LED 7-segement display with 8 resistors for each LED-segment. Another optional LED connected with a resistor to the ESP32 indicates a button-press-event by toggling on/off. Power supply can be implemented by an USB-cable or an external battery module.

Even less components are needed for implementing the OLED version: The Heltec-Wi-Fi-Kit board, four switches and a case - that's all. This solution can be enhanced by rechargeable LiPo-battery with will be charged via a connected USB cable. A battery connector and charge management are included in the Heltec board.

If I find the time I will design a tiny PCB that hosts the ESP32 and the external components for the seven segment variant. The OLED version just need the four switches connected to the board. Stay tuned for further instructions...
