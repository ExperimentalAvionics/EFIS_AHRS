# EFIS AHRS

Arduino code for AHRS module based on BNO055.

The module is connnected to the rest of the avionics via CAN-Bus.

Details are here: http://experimentalavionics.com/efis-ahrs-ver-2-0/

Unzip libraries.zip to the "libraries" folder in your Arduino environment.

## Release Notes: ##

### 2021-06-05 ###
* Removed Button/LED calibration routine. It has been proven to be non-practical. Calibration controlled from the Display is better. The button/LED has been removed from the later version of PCB anyway.
* Added Moving Average filers for acceleration data for all axis. Reason - suppress engine vibration noise.
* Reduced the rate of the data being sent into the Displays from 20 times per second to 10 times per second