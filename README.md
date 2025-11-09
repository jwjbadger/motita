# Motita

A program intended to run on an ESP32-WROOM-32 microcontroller to control two motors with optical encoders to sync the positions of the two. In application, the motors control two engine levelers to keep a lift for large animals (up to 2000 lbs) level while in motion.

The frame and engine levelers can be seen in the following image without the motors attached:

![frame](https://github.com/user-attachments/assets/d6472ff2-ec1c-4cb8-be15-ed3920635ab1)

## Library

The lib file exposes a few structs that make controlling generic motors simple. The `MotorController` struct integrates the motor pulse width modulation (PWM) to control the speed of the motor and the value of the encoders to read the average velocity of the motor given the time period since the velocity was last read. The `PIDController` struct implements generic PID with a few default values that happen to work well with the given motors and refresh rate. A number of methods allow it to be updated with customized values. Additional structs can be found in the library file to make this control more simplistic. The `PIDController` takes a reference to the `MotorController` that must be valid for the lifetime of the `PIDController`. In the future, a generic control trait may be implemented to make the expansion of control algorithms more rapid.

## Software

The current program running on the esp32 may be found in the [main](src/main.rs) file. A timer is taken from the esp32 peripherals, which is then subscribed to so as to provide a periodic interrupt defined by the `PERIOD` constant at the top of the file. Currently, it runs at 120Hz and updates the velocity of both motors based on the input of two buttons with the integral error on the motor being the estimated difference in position between the two motors (because the position is the most important to sync in this particular case).

## Hardware

Pictures and a wiring diagram to be included.
