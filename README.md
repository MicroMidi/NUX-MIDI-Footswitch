# NUX-MIDI-Footswitch
# BLE MIDI Footswitch Project for the NUX Mighty Plug / Mighty Air

This project is intended to build a simple wireless footswitch for the NUX Mighty Plug / Mighty Air based on the ESP32 microcontroller with integrated Bluetooth funtionality. In the current version the code supports two buttons: The first button is implemented to increment the effect-number, the second button to decrement the effect-number. A long press on the increment-button sets the effect back to the first effect of the NUX devicde.

A seven segment LED-display shows the current effect that is selected, another optional LED toogles on/off with every button-press event.

The code is able to handle MIDI effect-switching-events sent from the NUX Mighty device to be able to synchronize the selected effect between footswitch and NUX device. This ensures that the indicated effect of the footswitch and the NUX Mighty device are always in sync - even if you switch the selected effects at the NUX device.

The client code uses the powerful BLE-MIDI library from @lathoub (https://github.com/lathoub/Arduino-BLE-MIDI) which leverages the NimBLE-Bluetooth library (https://github.com/h2zero/NimBLE-Arduino) with an extraordinary low memory footprint and a very reliable operation. In addition to this you need the following library for the switch buttons: https://github.com/avandalen/avdweb_Switch. If you want to display the selected effect at the footswitch on a seven-segment display you need to include this library: https://www.arduino.cc/reference/en/libraries/advance-seven-segment/. By including these four libraries in your Arduino IDE you will be able to compile the client code without errors.

To build the footswitch hardware on your own you really do not need many components: Mandatory is an ESP32 microcontroller with two push buttons for effect-switching. Optional is a LED 7-segement display with 8 resistors for each LED-segment. Another optional LED connected with a resistor to the ESP32 indicates a button-press-evvent by toogling on/off. Power supply can be implemented by an USB-cable or an external battery module.

If I find the time I will design a tiny PCB that hosts the ESP32 and the external components. Stay tuned ...
