APPS-Analog

This project:

	-The STM32f407G-DISC1 board reads an analog value from the APPS(linear position sensor), converts the signal to digital, runs the signal through a matlab converted torque calculator, and finally outputs the desired torque value.

	-The current torque calculator has a linear map from APPS actuation[%] to requested torque.

	-In reality, the torque output would be linear for as long as the poweroutput allows, then evening out in proportion with the motor speed.
