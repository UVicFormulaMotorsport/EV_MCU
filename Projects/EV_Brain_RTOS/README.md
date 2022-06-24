# EV Brain
Owners: Randon Abrams \
Current State: Unfinished
### Description
Interfaces with the motor controller

The main responsibility of this system is sending torque requests to the motor controller. It reads pedal sensor values, put them through a torque map to get figure out the torque demand %. This percentage is then converted to an actual torque value by comparing to the maximum available torque currently available from the motor. Then the torque value is converted to a setpoint value and sent to the controller. 

The system also needs to check for implausibilites in the pedal sensors.
Implausibilites include:
- Accelerator pedal and brake pedal pressed at the same time
- Accelerator pedal and brake pedal sensor values outside of the proper range which indicates a short or open circuit

If an implausibility is detected, torque requests of 0 are sent to the controller untill the implausibilty is removed.

For data logging, we will need to set up can messages to request the data that we want to log from the controller. All CAN messages going to the controller should come from the EV brain so we can make use of the CAN timeout feature on the motor controller

The system has grown in complexity since it was started, it is probably worth converting it to RTOS to make task management easier. 

### TODO
- Convert the system to RTOS
- Integrate and test the RTD functionality with the RTD board
- Set the pedal sensor ranges and incorporate them into the implausibility checks
- May need to implement cooling control for the motor and motor controller


### References
