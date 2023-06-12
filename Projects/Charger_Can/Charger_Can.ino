#include <mcp2515.h>

#include <SPI.h>

MCP2515 mcp2515(10);

#define START_CHARGING 0
#define STOP_CHARGING 1

#define MAX_VOLTAGE 2000
#define MAX_CURRENT 5

#define ENABLE_PIN 4

bool interrupt = false;
int c = 0;
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



void can_int_handler()
{
  interrupt = true; //WHY IS THIS LIKE THIS?? HANDLE THE INTERRUPT
}

void shdn_int_handler()
{
  // If the shutdown circuit was triggered,send a can message to stop charging
  can_send(0, 0, STOP_CHARGING);
  //while(1);
}

void setup()
{
  // Setup serial
  Serial.begin(19200);
  if(!Serial)
  {
    //Serial.println("Serial setup failed");
    //while(1);
  }
  Serial.println("Serial setup complete");
  delay(1000);
  // Setup CAN
  mcp2515.reset();
    
  Serial.println("CAN setup start");
  int result = mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  if(result != 0)
  {
    Serial.println(result);
    Serial.println("Set bitrate failed");
    //while(1);
  }
  Serial.println("Bitrate ok");
 
  result = mcp2515.setNormalMode();
  if(result != 0)
  {
    Serial.println(result);
    Serial.println("Set mode failed");
    //while(1);
  }
  Serial.println("Normal mode ok");
  Serial.println("CAN setup complete");

  // Setup CAN message interrput handler
  //attachInterrupt(0, can_int_handler, FALLING);

  // Setting shutdown circtuit enable pin
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  // Setting interupt handler for shutdown cirtcuit
  //attachInterrupt(1, shdn_int_handler, RISING);


  Serial.println("Setup finished");
}

void loop() {
  // Send a can message
//  if(interrupt == true)
//  {
//    Serial.println("---------------------------------------------------------");
//    can_receive();
//    interrupt = false;
//  }
  
  can_send(MAX_VOLTAGE, MAX_CURRENT, START_CHARGING);
  c++;
  Serial.println(c);
  delay(500);
}
