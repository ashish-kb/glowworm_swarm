#define BOT_ADDRESS 'c'
#define INTENSITY_CHAR 'k'
#define MOTOR_CHAR 'o'
#define MOTOR_ACK 'x'

#define LED_PIN 13

//Motor 1 pin assigment
#define MOTOR1_PIN1 4
#define MOTOR1_PIN2 5
#define MOTOR1_SPEED_PIN 9

//Motor 2 pin assigment
#define MOTOR2_PIN1 6
#define MOTOR2_PIN2 7
#define MOTOR2_SPEED_PIN 10

//Motor bounds
#define MIN_MOTOR_SPEED 20  //speed at which motor starts running
#define MAX_MOTOR_SPEED 255

#define SENSOR1_PIN A0
#define SENSOR2_PIN A1
#define SENSOR3_PIN A2
#define SENSOR4_PIN A3
#define SENSOR5_PIN A4
#define SENSOR6_PIN A5

#define MIN_READING_SENSOR 1024
#define Loffset 0
#define Roffset 0
#define MOV_AVG_NO 10
#define ROT_HALT_DELAY 70
#define TRANS_HALT_DELAY 700
#define SENSOR_TRIM 3000
#define StepReduce 10

int sensor_pins[]={SENSOR1_PIN, SENSOR2_PIN, SENSOR3_PIN, SENSOR4_PIN, SENSOR5_PIN, SENSOR6_PIN};

class Motor{
  private:
    int _motor_pin1, _motor_pin2, _motor_speed_pin;
  public:
    void attachPins(int mp1, int mp2, int msp){
      _motor_pin1=mp1;    
      _motor_pin2=mp2;
      _motor_speed_pin=msp;
      pinMode(_motor_pin1, OUTPUT);
      pinMode(_motor_pin2, OUTPUT);
    }
    void setMotorSpeed(int motor_speed){  //TODO: check if the condition has to be <= or >=
      if(motor_speed>=0){
        digitalWrite(_motor_pin1, HIGH);
        digitalWrite(_motor_pin2, LOW);
      }
      else{
        digitalWrite(_motor_pin1, LOW);
        digitalWrite(_motor_pin2, HIGH);      
      }
      analogWrite(_motor_speed_pin, abs(motor_speed));    
    }
};

/*
Motor::Motor(int mp1, int mp2, int msp){
  _motor_pin1=mp1;
  _motor_pin2=mp2;
  _motor_speed_pin=msp;
  pinMode(_motor_pin1, OUTPUT);
  pinMode(_motor_pin2, OUTPUT);
}
*/

class Sensor{
  private:
    int _sensor_pin, _sensor_value;
  public:
    void attachPin(int sp){
      _sensor_pin=sp;
    }
    int readData(){
      return analogRead(_sensor_pin);
    }

    int readAvgData(){
      int _sensor_value_avg = 0;
      for (int i=0; i<MOV_AVG_NO;i++){
        _sensor_value_avg += analogRead(_sensor_pin);
      }
      _sensor_value_avg = _sensor_value_avg/MOV_AVG_NO;
      return _sensor_value_avg;
    }

};

class Robot{
  private:
    Motor motor_left, motor_right;
    Sensor sensor[6];
  public:
    Robot(){
      motor_left.attachPins(MOTOR1_PIN1, MOTOR1_PIN2, MOTOR1_SPEED_PIN);
      motor_right.attachPins(MOTOR2_PIN1, MOTOR2_PIN2, MOTOR2_SPEED_PIN);
      for(int i=0;i<6;i++)  
        sensor[i].attachPin(sensor_pins[i]);
    }
    void front(int cmdL, int cmdR){
      //int normalised_speed=MIN_MOTOR_SPEED+(cmd*(MAX_MOTOR_SPEED-MIN_MOTOR_SPEED));
    //  int normalised_speed=cmd;
      motor_left.setMotorSpeed(-cmdL);
      motor_right.setMotorSpeed(-cmdR);
      
    }
    
    void back(int cmdL, int cmdR){
      //int normalised_speed=MIN_MOTOR_SPEED+(cmd*(MAX_MOTOR_SPEED-MIN_MOTOR_SPEED));
  //    int normalised_speed=cmd;
      motor_left.setMotorSpeed(cmdL);
      motor_right.setMotorSpeed(cmdR);
    }
    
    void left(int cmdL, int cmdR){
      //int normalised_speed=MIN_MOTOR_SPEED+(abs(cmd)*(MAX_MOTOR_SPEED-MIN_MOTOR_SPEED));
      //int normalised_speed=cmd;  
        motor_left.setMotorSpeed(-cmdL);
        motor_right.setMotorSpeed(cmdR);
    }

    void right(int cmdL, int cmdR){
      //int normalised_speed=MIN_MOTOR_SPEED+(abs(cmd)*(MAX_MOTOR_SPEED-MIN_MOTOR_SPEED));
//      int normalised_speed=cmd;  
        motor_left.setMotorSpeed(cmdL);
        motor_right.setMotorSpeed(-cmdR);
    }

    void halt(){
      //int normalised_speed=MIN_MOTOR_SPEED+(abs(cmd)*(MAX_MOTOR_SPEED-MIN_MOTOR_SPEED));
//      int normalised_speed=cmd;  
        motor_left.setMotorSpeed(0);
        motor_right.setMotorSpeed(0);
    }
    void slowHalt(int valueL, int valueR)
    {
      while(min(valueL,valueR)<=50)
      {
        valueL = valueL - StepReduce;
        valueR = valueR - StepReduce;
        
        motor_left.setMotorSpeed(valueL);
        motor_left.setMotorSpeed(valueR);        
        delay(30);
      }
      motor_left.setMotorSpeed(0);
      motor_right.setMotorSpeed(0);
    }

    int getMaxSensorReading(){
      int max_reading=0;
      for(int i=0;i<6;i++)
      {
        int sensor_reading=sensor[i].readAvgData();
        //Serial.println(sensor_reading);
        if(max_reading<sensor_reading){
          max_reading=sensor_reading;
        }
      }
      return max_reading;
    }
    
    int getMinSensorReading(){
      int min_reading=MIN_READING_SENSOR;
      for(int i=0;i<6;i++)
      {
        int sensor_reading=sensor[i].readAvgData();
        //Serial.println(sensor_reading);
        if(min_reading>sensor_reading){
          min_reading=sensor_reading;
        }
      }
      return min_reading;
    }
    
    void serialFlush(){
      while(Serial.available() > 0) 
      {
        char t = Serial.read();
      }
    }

};
 



Robot swarm_bot;

// the setup routine runs once when you press reset:
void setup()  { 
  digitalWrite(LED_PIN, HIGH);  
  delay(50);
  Serial.begin(115200);
  delay(50);
  Serial.println("Board initialised");

  for (int i=0;i<6;i++)
  {
    Serial.print(analogRead(sensor_pins[i]));
    Serial.print(" ");
  }
  Serial.println(" ");  
 // swarm_bot.halt();
  
} 
char cmd[5];
uint8_t value; 
uint8_t valueL; 
uint8_t valueR; 
uint8_t front_flag=0;
int exc_prevent = 0;
// the loop routine runs over and over again forever:
void loop()  { 
  /*
  if (Serial.available() > 0) {
    Serial.readBytes(cmd, 2);
      for(int i=0;i<2;i++)
  {
    Serial.println(cmd[i]);
  }
  }
*/
  /* 
  while (Serial.available() > 0) {
    // get incoming byte:
    cmd = Serial.read();
    value=cmd;
    Serial.println(cmd);
    Serial.println(value);
   }
   */

  if (Serial.available() > 0) {
//    while (Serial.peek() == BOT_ADDRESS);
    Serial.readBytes(cmd, 5);
//    Serial.println(cmd);
    
    //      Serial.println("Bot a");
      if((cmd[0]==MOTOR_CHAR)&& (cmd[1]==MOTOR_CHAR) && (cmd[2]==MOTOR_CHAR) && (cmd[3]==MOTOR_CHAR) && (cmd[4]==MOTOR_CHAR)){ // move front
//        Serial.print(cmd[2]);
//        Serial.print(cmd[3]);
//        Serial.print(cmd[4]);
        //Serial.println("Move Forward");
        //Serial.println(front_flag);
//        value=(cmd[2]-48)*100+(cmd[3]-48)*10+(cmd[4]-48);
          value = 60;
//        Serial.print(value);
//        Serial.println("translate front");
        Serial.println(MOTOR_ACK);
//        Serial.flush();
//        swarm_bot.serialFlush();
        valueL = min(255,value + Loffset);
        valueR = min(255,value + Loffset);
      if(!front_flag)
        {
          swarm_bot.front(valueL, valueR);
          delay(TRANS_HALT_DELAY);
          front_flag = 1;
//          swarm_bot.serialFlush();
       //   Serial.println("Move Forward");
        // Serial.println(front_flag);
        }
        
//        swarm_bot.slowHalt(valueL, valueR);
        
         swarm_bot.halt();

       
       /*
        digitalWrite(4, LOW);
        digitalWrite(5, HIGH);      
        analogWrite(9, value);
  */
      }

     else if((cmd[0]==BOT_ADDRESS) && (cmd[1]=='l')){ // move left
        value=(cmd[2]-48)*100+(cmd[3]-48)*10+(cmd[4]-48);
//        Serial.print(value);
//        Serial.println("rotate left");
        valueL = min(255,value + Loffset);
        valueR = min(255,value + Loffset);
        swarm_bot.left(valueL, valueR);
        delay(ROT_HALT_DELAY);
        swarm_bot.halt();
      }
      
      else if((cmd[0]==BOT_ADDRESS) && (cmd[1]=='r')){  // move right
        value=(cmd[2]-48)*100+(cmd[3]-48)*10+(cmd[4]-48);
//       Serial.print(value);
//        Serial.println("rotate right");
        valueL = min(255,value + Loffset);
        valueR = min(255,value + Loffset);
        swarm_bot.right(valueL, valueR);
        delay(ROT_HALT_DELAY);
        swarm_bot.halt();
      }

       /*else if(cmd[1]=='b'){  // move back
        value=(cmd[2]-48)*100+(cmd[3]-48)*10+(cmd[4]-48);
//       Serial.print(value);
//        Serial.println("translate back");
        valueL = min(255,value + Loffset);
        valueR = min(255,value + Loffset);
        swarm_bot.back(valueL, valueR);
      }
      */
       else if(cmd[1]=='h'){  // halt
//       value=(cmd[2]-48)*100+(cmd[3]-48)*10+(cmd[4]-48);
//       Serial.print(value);
//        Serial.println("translate back");
        swarm_bot.halt();
      }

      else if((cmd[0]== INTENSITY_CHAR)&&(cmd[1]== INTENSITY_CHAR)&&(cmd[2]== INTENSITY_CHAR)&&(cmd[3]== INTENSITY_CHAR)&&(cmd[4]== INTENSITY_CHAR)){
        //Serial.print(value);
        swarm_bot.serialFlush();
        Serial.println(SENSOR_TRIM+swarm_bot.getMinSensorReading());
//        Serial.flush();
        front_flag = 0;
//        swarm_bot.serialFlush();
      }
      
  }
}


