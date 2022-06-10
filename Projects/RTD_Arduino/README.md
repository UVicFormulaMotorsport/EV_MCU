# Ready To Drive
Owners: Randon Abrams \
Current State: Unfinished
### Description
Monitors for signals from the EV_Brain system to play a sound when the car is ready to enter its driving state. 

The board has an SD card module where the sound file is stored. The WAV (waveform) file format is used. The file data is translated to a PWM signal which is played through a speaker on the board. 

This board may also be used to translate may also be used to translate messages from the motor controller so they can be logged on out digital dash. Every message from the motor controller has the same CAN ID so the dash is not able to differentiate between different data. To address this, we could use this arduino to monitor for messages from the motor controller and retransmit them with unique CAN IDs which the dash will recognize. 

### TODO
- Test the code on the RTD shield
- Implement CAN message translation