# Charging Manager
Owners: Randon Abrams \
Current State: Unfinished
### Description
Charging manager which interfaces with the Elcon charger. The code runs on an STM32F103 on a custom pcb.  

The system has a CAN tranciever which is used to send periodic CAN messages to the controller which contain target output voltage and current values. It also recieves and parses messages from the charger to monitor for faults. If faults are detected, the shutdown circuit will be triggered. It also monitors the shutdown circuit to detect if it gets triggered by other sources, in that case, a message to stop charging is sent to the charger.

A menu system will be implemented on the LCD and a joystick will be used to navigate through it. The menu will contain preset charging profiles which can be selected and the associated values will be sent over CAN to the charger.

### TODO
- Rewrite the arduino code from Charger_Can to work on the STM
- Implement the menu system