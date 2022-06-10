#include <mcp2515.h>
#include <SPI.h>

MCP2515 mcp2515(10);

#define START_CHARGING 0
#define STOP_CHARGING 1

#define MAX_VOLTAGE 500
#define MAX_CURRENT 100

#define SHUTDOWN_TRIGGER_PIN 4

bool interrupt = false;

void can_send(int max_voltage, int max_current, int charging_status)
{    
    struct can_frame frame;
    
  // 0x1806E5F4 Message from the "BMS" (F4) to the charger (E5)
  frame.can_id = 0x1806E5F4 | CAN_EFF_FLAG;
  frame.can_dlc = 5;

  // Max voltage high byte
  frame.data[0] = (max_voltage >> 8) & 0xFF;
  // Max voltage low byte
  frame.data[1] = max_voltage & 0xFF;

  // Max current high byte
  frame.data[2] = (max_current >> 8) & 0xFF;
  // Max current low byte
  frame.data[3] = max_current & 0xFF;

  // Charging control 
  frame.data[4] = charging_status & 0xFF;
  
  // Send message
  mcp2515.sendMessage(&frame);
}

void can_receive()
{
  struct can_frame recieved_frame;
  uint8_t irq = mcp2515.getInterrupts();
  
  if (irq & MCP2515::CANINTF_RX0IF)
  {
      if (mcp2515.readMessage(MCP2515::RXB0, &recieved_frame) != MCP2515::ERROR_OK)
      {
          Serial.println("Failed to read message from RXB0");
          while(1);
      }
  }
  else if (irq & MCP2515::CANINTF_RX1IF)
  {
      if (mcp2515.readMessage(MCP2515::RXB1, &recieved_frame) != MCP2515::ERROR_OK)
      {
          Serial.println("Failed to read message from RXB1");
          while(1);
      }
  }
  else
  {
    Serial.println("Interupt triggered but no message recieved");
    while(1);
  }
 
  int voltage = (recieved_frame.data[0] << 8) + recieved_frame.data[1];
  int current = (recieved_frame.data[2] << 8) + recieved_frame.data[3];
  int charger_status = recieved_frame.data[4];
  
  char msg[100];
  sprintf(msg, "Output voltage: %d, Output current %d, Status: %d", voltage, current, charger_status);
  Serial.println(msg);

  if(charger_status != 0)
  {
    // Trigger shutdown circuit
    digitalWrite(SHUTDOWN_TRIGGER_PIN, LOW);
    
    // Send a message to stop charging
    can_send(0,0,STOP_CHARGING);

    // Output a message based on the errors
    if((charger_status & 0x1) == 1)
    {
      Serial.println("Hardware Failure");
    }
    if((charger_status & 0x2) == 1)
    {
      Serial.println("Overtemperature Protection");
    }
    if((charger_status & 0x4) == 1)
    {
      Serial.println("Incorrect Input Voltage");
    }
    if((charger_status & 0x8) == 1)
    {
      Serial.println("Battery Reverse Polarity");
    }
    if((charger_status & 0x16) == 1)
    {
      Serial.println("Communication Time-out");
    }
    while(1);
  }
}

void can_int_handler()
{
  interrupt = true;
}

void shdn_int_handler()
{
  // If the shutdown circuit was triggered,send a can message to stop charging
  can_send(0, 0, STOP_CHARGING);
  while(1);
}

void setup()
{
  // Setup serial
  Serial.begin(9600);
  if(!Serial)
  {
    Serial.println("Serial setup failed");
    while(1);
  }
  Serial.println("Serial setup complete");
 
  // Setup CAN
  mcp2515.reset();
    
  Serial.println("CAN setup start");
  int result = mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  if(result != 0)
  {
    Serial.println(result);
    Serial.println("Set bitrate failed");
    while(1);
  }
  Serial.println("Bitrate ok");
 
  result = mcp2515.setNormalMode();
  if(result != 0)
  {
    Serial.println(result);
    Serial.println("Set mode failed");
    while(1);
  }
  Serial.println("Normal mode ok");
  Serial.println("CAN setup complete");

  // Setup CAN message interrput handler
  attachInterrupt(0, can_int_handler, FALLING);

  // Setting shutdown circtuit trigger pin
  pinMode(SHUTDOWN_TRIGGER_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_TRIGGER_PIN, HIGH);

  // Setting interupt handler for shutdown cirtcuit
  //attachInterrupt(1, shdn_int_handler, RISING);


  Serial.println("Setup finished");
}

void loop() {
  // Send a can message
  if(interrupt == true)
  {
    Serial.println("---------------------------------------------------------");
    can_receive();
    interrupt = false;
  }
  else
  {
    can_send(MAX_VOLTAGE, MAX_CURRENT, START_CHARGING);
  }

  delay(3000);
}
