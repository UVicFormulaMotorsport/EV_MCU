/*
  SD card pins
  ** CS - pin 10
  ** MOSI - pin 11
  ** MISO - pin 12
  ** CLK - pin 13
  * 
  * 
  * Link to the sound https://www.youtube.com/watch?v=kuI-wbTE5PU&ab_channel=SoundLibrary
*/
#include <SPI.h>
#include <SD.h>
#include <TMRpcm.h>

const int chip_select = 10;
const int RTD_sig_pin = 2;
const int RTD_out_pin = 4;
const int speaker_pin = 9;
const int NOT_RTD = 0;
const int RTD = 1;
int CAR_STATE;

TMRpcm tmrpcm;

void setup()
{
  // Setup pins
  pinMode(RTD_sig_pin, INPUT);
  pinMode(RTD_out_pin, OUTPUT);
  pinMode(speaker_pin, OUTPUT);
  
  Serial.println("Setup started");
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while(!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  // Init SD card  
  Serial.print("Initializing SD card...");
  if (!SD.begin(chip_select))
  {
    Serial.println("initialization failed!");
    while (1);
  }

  // Set speaker pin
  tmrpcm.speakerPin = 9;
  // Set volume
  // Volume can be set up to 7 but sound quality decreases
  tmrpcm.setVolume(6); 

  // Set RTD signal interupt
  attachInterrupt(digitalPinToInterrupt(RTD_sig_pin), rtdInterupt, RISING);

  // Set initial car state
  CAR_STATE = NOT_RTD; 
  
  Serial.println("initialization done.");
  Serial.println("Setup Finished");
}
  
void loop()
{
  if(CAR_STATE == RTD)
  {
    // Do stuff
  }
}

void playSound()
{
  Serial.println("Playing sound"); 
  // Play the wav file from the sd card
  tmrpcm.play("sound8.wav");

  // Wait until sound is finished playing
  while(tmrpcm.isPlaying())
  {
    ;  
  }
  
  // Send pulse to STM when finished playing
  digitalWrite(RTD_out_pin, HIGH);
  delay(1);
  digitalWrite(RTD_out_pin, HIGH);
  Serial.println("Sound finished"); 
  return;
}

void rtdInterupt()
{
  if(CAR_STATE == NOT_RTD)
  {
    playSound();
    CAR_STATE = RTD;
  }
  else
  {
    CAR_STATE = NOT_RTD;
  }

  return;
}
