# EV Brain
Contact: Benjamin Say, Peter Grant
08-June-2023
### Description
Interfaces with the motor controller, power distribution unit, LCD dashboard, and Insulation Monitoring Device (IMD) for UVic's 2023 EV

The primary function of this system is sending torque requests to the motor controller. Pedal sensor values are read and mapped to an actuation percentage. This percentage is then converted to a torque value by comparing to the maximum available torque at the motor's current RPM. The torque value is then converted to a setpoint value and sent to the controller.

The system also needs to check for implausibilites in the pedal sensors.
Implausibilites include:
- Accelerator pedal and brake pedal pressed at the same time
- Accelerator pedal and brake pedal sensor values outside of the proper range, indicating a short or open circuit

If an implausibility is detected, the system shuts down.

This system will be converted to an RTOS for the 2024 season.

### Motor Controller Information:

To send data to the motor controller, the following data format is used:

   Byte 1   |     Byte 2     |     Byte 3
register ID | byte 0 of data | byte 1 of data

For example, sending data 0xB04F to register 0x3A would look like:
data[3] = {0x3A, 0x4F, 0xB0}

To request data from the motor controller, the data array only contains the register ID.
### PDU Information:

Channel Map:

20A (2x fan, 1x pump):
20A - Pins 1&2 on J1 = Channel 3, Msg: Enable 0x33, Disable 0x23
20A - Pins 3&4 on J1 = Channel 4, Msg: Enable 0x34, Disable 0x24
20A - Pins 6&7 on J1 = Channel 1, Msg: Enable 0x31, Disable 0x21

5A (1x Speaker, 1x Brake Light, 1x Motor Controller Enable, 1x Shutdown Circuit):
5A pin 33 on J2 - channel B, Msg: Enable 0x1B, Disable 0x0B
5A pin 34 on J2 - Channel E, Msg: Enable 0x1E, Disable 0x0E
5A pin 32 on J2 - Channel F, Msg: Enable 0x1F, Disable 0x0F
5A pin 25 on J2 - Channel C, Msg: Enable 0x1C, Disable 0x0C

Wiring:
Blue wire - 20Amp circuits
Grey wire - 5Amp Circuts
Orange wire - Vbat (12V)
Twisted pair:
    yellow - CAN low
    blue - CAN high
Green - GND

Facing connectors:
J1 = Left Connector
J2 = Right Connector

Message Format:
1 Byte: 0b00000000
Bits 0-3: channel select
Bit 4: Enable = 1, Disable = 0
Bit 5: 20A circuits = 1, 5A circuit = 0

### TODO
- implement ready to drive signal
- implement motor RPM data request through CAN to calculate torque
- figure out how the pedal map works
- figure out what PEAK_TORQUE_MODE variable is for
- verify values for constants MAX_CONTINUOUS_TORQUE and MAX_PEAK_TORQUE
- define constants for each PDU channel to make calling sendCANmsg_PDU() easier
- implement sendCANmsg_DASH() function for updating the dash LCD
- decide which torque function to use
- implement plausibility checks
- figure out what pedal map is for
- decide how to check accelerator and brake pedal values against eachother
