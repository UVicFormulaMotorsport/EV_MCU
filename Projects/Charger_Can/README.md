# Charger CAN
Owners: Randon Abrams \
Current State: Unfinished
### Description
This project is a simpler version of the charing manager which interfaces with the Elcon charger. 

It uses an Arduino and an MCP2515 to send periodic CAN messages to the controller which contain target output voltage and current values. It also recieves and parses messages from the charger to check for faults. If a fault is detected, the shutdown circuit is triggered and a command is sent ot the charger to stop charging. It monitors the shutdown circuit to detect if it gets triggered by other sources, in that case, a message to stop charging is sent to the charger.

The target output voltage and current must be set in the code before uploading. This can be done by changing the values of MAX_VOLTAGE and MAX_CURRENT values near the top of the file

```
#define MAX_VOLTAGE 500
#define MAX_CURRENT 100
```

### TODO
- Test shutdown circuit fault detection
- Add a way start sending messages after uploading code