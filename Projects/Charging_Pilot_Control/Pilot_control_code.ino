
//Pin addresses
const int CP_Read = 5;
const int PP_Read = 0;
const int Error_indicator = 2;
const int PWR_enable = 6;
const int J1772_conn = 7;

//Global Variables
enum pp {nc, chrgrConnected, buttonPressed};
pp pp_state = nc;
unsigned int pp_reading = 0;
int incoming_byte = 0;
int max_current = 0;
bool charging_allowed = false;



//Function Prototypes
void determine_current(int pulse_length);
int measure_duty_cycle(); //Determines the state of the J1772 CP signal

void get_pp_state(); //what is the connection state of the pp (NC, button pressed, connected)

void send_uart_msg(); //pretty self explanatory

void setup() {
  // configure all the pins
  pinMode(CP_Read, INPUT);
  pinMode(Error_indicator, OUTPUT);
  pinMode(PWR_enable, OUTPUT);
  pinMode(J1772_conn, OUTPUT);
  
  //initialize serial
  Serial.begin(9600);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0){
    incoming_byte = Serial.read();
    
  }

  switch(pp_state){
    case nc:

    break;
    case chrgrConnected:
      
  
    break;
    case buttonPressed:

    break;
    default:

    break;
  }

}

void get_pp_state(){
  pp_reading = analogRead(A0); //get voltage reading from analog pin
  if(pp_reading > 900){
    pp_state = nc;
    digitalWrite(J1772_conn, LOW);
    digitalWrite(PWR_enable, LOW);
    digitalWrite(Error_indicator, LOW);
  }
  else if(pp_reading > 450){
    pp_state = buttonPressed;
    digitalWrite(J1772_conn, LOW);
    digitalWrite(PWR_enable, LOW);
    digitalWrite(Error_indicator, HIGH);
  }
  else{
    pp_state = chrgrConnected;
    digitalWrite(J1772_conn, HIGH);
    digitalWrite(Error_indicator, LOW);
  }
}

//find the max current using some fun mathematix
void determine_current(int pulse_length){
  if(pulse_length == -1){
    return;
  }
  else if(pulse_length < 70){
    max_current = 0;
  }
  else if(pulse_length < 640){
    max_current = (pulse_length/10);
  }
  else if(pulse_length < 960){
    max_current = (pulse_length/10);
    
  }else{
    max_current = 0;
  }
  
}

//measures how many microseconds it takes for one pulse to happen
int measure_duty_cycle(){
  if(digitalRead(CP_Read) == HIGH){
    while(digitalRead(CP_Read) == HIGH){} //exists so that if the CP signal happens to be high when function called, it will wait until the next pulse to avoid erronious results
  }
  unsigned long i = micros();
  unsigned long j = i;
  while(digitalRead(CP_Read) == LOW){
    i = micros();
    if(i-j>2048){ //check for overflow of counter, returns -1 to indicate error
      return -1;
    }
    if(i-j>1024){ // if it takes more than a millisecond, return 0 as the duty cycle will therefore be zero
      return 0;
      }
    }
    
    i = micros();
    j = i;
    while(digitalRead(CP_Read) == HIGH){
      i = micros();
    }
     if(i-j>2048){
      return -1;
    } else {
      return (int)(i-j);
    }
}

void send_uart_msg(int* status){
  
}
