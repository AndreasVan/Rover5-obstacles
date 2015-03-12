
// Control of a Rover5 robot - Last update: AndreasVan 2015-03-01 Vers. 3.4
// Dagu Rover 5 2WD Tracked Chassis + Explorer Controller Board for Dagu Rover 5 2WD
// Micro controller = Arduino Mega 2560
// Detecting obstacles with an SR04 ultrasonic sensor mounted on servo and IR sensors
// this code is public domain, enjoy!

#include <NewPing.h>          //library SR04 ultrasonic sensor
#include <Servo.h>            //library Servo (SR04 on servo)
#include <rover_ir_sensors.h> //library ir sensors

#define TRIGGER_PIN 9         //SR04 sensor  
#define ECHO_PIN    8         //SR04 sensor
#define MAX_DISTANCE 200      //sensor distance
Servo myservo;  
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))

const int PWN1 = 6;           //right PWN
const int DIR1 = 7;           //right DIR
const int PWN2 =11;           //left PWN
const int DIR2 =12;           //left DIR
const int redLed = 5;         //robot drives forward
const int greenLed = 4;       //robot drives backward
const int yellowLed = 3;      //robot turns left
const int blueLed = 2;        //robot turns right
const int NUM_IR_SENSORS = 4;
const int IR_Led_Pins[ NUM_IR_SENSORS ] = { A0, A0, A1, A1 };   
const int IR_Sensor_Pins[ NUM_IR_SENSORS ] = { A3, A2, A4, A5 };
const int Close_Range_IR_Value = 150;

RoverIRSensors IRSensors( 
    IR_Led_Pins[ 0 ], IR_Led_Pins[ 1 ],
    IR_Led_Pins[ 2 ], IR_Led_Pins[ 3 ],
    IR_Sensor_Pins[ 0 ], IR_Sensor_Pins[ 1 ],
    IR_Sensor_Pins[ 2 ], IR_Sensor_Pins[ 3 ] );

int uS;                     //value of SR04 ultrasonic sensor
int distance;               //distance in cm of ultrasonic sensor 
int pos = 90;               //start position of servo = 90
int servoDirection = 0;     //sweeping left or right
int MoveDirection = 0;      //0 = forward, 1 = backward, 2 = left, 3 = right
int lastMoveDirection;      //last direction of the robot
int distanceCenter;     
int distanceLeft;
int distanceRight;
int servoDelay = 55;        //servo sweep speed
const int speedLeft = 120;  //motor speed left
const int speedRight = 120; //motor speed right
long previousMillis = 0;
const int interval = 1000;  //time to switch between the MoveDirections when it detects an obstacle

void setup() {                
  pinMode(PWN1, OUTPUT);  
  pinMode(DIR1, OUTPUT); 
  pinMode(PWN2, OUTPUT); 
  pinMode(DIR2, OUTPUT);
  analogWrite(PWN1, 0);
  analogWrite(PWN2, 0);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(redLed, OUTPUT); 
  pinMode(greenLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  Serial.begin(9600);       //to use the serial monitor
  myservo.attach(10);       //servo on pin 10
  myservo.write(pos);       //center servo
  delay(2000);              //start delay
}

void loop() {
  
    IRSensors.takeReadings();  // read from the IR sensors  
    int frontLeftIR = IRSensors.lastFrontLeftReading();
    int frontRightIR = IRSensors.lastFrontRightReading();
    int rearLeftIR = IRSensors.lastRearLeftReading();
    int rearRightIR = IRSensors.lastRearRightReading();
        
    Serial.print(frontLeftIR);
    Serial.print(frontRightIR);
    Serial.print(rearLeftIR);
    Serial.print(rearRightIR);
    Serial.print("");
    

  sweepServo();                //servo sweep function
  
  getDistance();               //ultrasonic sensor distance 
  
    if (pos >= 15 && pos <= 45)
  {
    distanceRight = distance;  //measured distance servo right = distanceRight
  }
  if (pos >= 135 && pos <= 165)
  {
    distanceLeft = distance;   //measured distance servo left = distanceLeft
  }   
  if (pos > 70 && pos < 110)
  {
    distanceCenter = distance; //measured distance servo centred  = distanceCenter
  } 
  if (distanceCenter >= 25)    //no obstacle to see for miles, straight on
  {
    MoveDirection = 0;         //move forward
  }
 //     if (rearLeftIR >= Close_Range_IR_Value && rearRightIR >= Close_Range_IR_Value)    //no obstacle to see for miles, straight on
 // {
 //   MoveDirection = 0;         //move forward
 // }
   else                         //obstacle detected, turn left or right
  {
     if (distanceRight < distanceLeft)  
    {
      MoveDirection = 2;      //turn left = 2
    }
//    if (rearRightIR <= Close_Range_IR_Value)  
//    {
//      MoveDirection = 2;      //turn left = 2
//    }
    if (distanceLeft < distanceRight) 
    {
      MoveDirection = 3;      //turn right = 3
    }
//    if (rearLeftIR >= Close_Range_IR_Value) 
//    {
//      MoveDirection = 3;      //turn right = 3
//    }
    if (distanceLeft <= 5 && distanceCenter <= 5 || distanceRight <= 5 && distanceCenter <= 5)
    {
      MoveDirection = 1;      //  turn back = 1
    }
//    if (frontLeftIR <= Close_Range_IR_Value && frontRightIR <= Close_Range_IR_Value)
//    {
//      MoveDirection = 1;      //  turn back = 1
//    }
  }

  unsigned long currentMillis = millis();  //timer

  if(MoveDirection == 0 && MoveDirection == lastMoveDirection)  
  {
    forward();
    lastMoveDirection = MoveDirection;
  }
  if(MoveDirection == 0 && MoveDirection != lastMoveDirection && currentMillis - previousMillis > interval )
  {  
    forward();
    lastMoveDirection = MoveDirection;
    previousMillis = currentMillis;
  }
  if(MoveDirection == 1 && MoveDirection == lastMoveDirection)
  {
    backward();
    lastMoveDirection = MoveDirection;
  }
  if(MoveDirection == 1 && MoveDirection != lastMoveDirection && currentMillis - previousMillis > interval )
  {  
    backward();
    lastMoveDirection = MoveDirection;
    previousMillis = currentMillis;
  }
  if(MoveDirection == 2 && MoveDirection == lastMoveDirection)
  {
    left();
    lastMoveDirection = MoveDirection;
  }
  if(MoveDirection == 2 && MoveDirection != lastMoveDirection && currentMillis - previousMillis > interval )
  {  
    left();
    lastMoveDirection = MoveDirection;
    previousMillis = currentMillis;
  }
  if(MoveDirection == 3 && MoveDirection == lastMoveDirection)
  {
    right();
    lastMoveDirection = MoveDirection;
  }
  if(MoveDirection == 3 && MoveDirection != lastMoveDirection && currentMillis - previousMillis > interval )
  {  
    right();
    lastMoveDirection = MoveDirection;
    previousMillis = currentMillis;
  }
}

void forward()
{
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);   
  analogWrite(PWN1, speedLeft);
  analogWrite(PWN2, speedRight);
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, LOW);
  digitalWrite(blueLed, LOW);
}

void stop()
{
  digitalWrite(DIR1, LOW);   
  digitalWrite(DIR2, LOW);   
  analogWrite(PWN1, 0);
  analogWrite(PWN2, 0);
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, LOW);
  digitalWrite(blueLed, LOW);
}

void backward()
{
  digitalWrite(DIR1, HIGH);  
  digitalWrite(DIR2, HIGH);   
  analogWrite(PWN1, speedLeft-10); 
  analogWrite(PWN2, speedRight-10); 
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, HIGH);
  digitalWrite(yellowLed, LOW);
  digitalWrite(blueLed, LOW);
}

void left()
{
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, HIGH);   
  analogWrite(PWN1, speedLeft-30); 
  analogWrite(PWN2, speedRight+30); 
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, HIGH);
  digitalWrite(blueLed, LOW);
}

void right()
{
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);   
  analogWrite(PWN1, speedLeft+30);  
  analogWrite(PWN2, speedRight-30); 
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, LOW);
  digitalWrite(blueLed, HIGH);
}

void getDistance()
{
  runEvery(20)                      //loop for ultrasonic measurement
  {
    uS = sonar.ping();
    distance = uS / US_ROUNDTRIP_CM;
    if (uS == NO_ECHO)              // if the sensor did not get a ping        
    {
      distance = MAX_DISTANCE;      //so the distance must be bigger then the max vaulue of the sensor
    }
    Serial.print("Ping: ");         //to check distance on the serial monitor
    Serial.print(distance); 
    Serial.println("cm");
  }
}

void sweepServo()
{
  runEvery(servoDelay)                      //this loop determines the servo position
  {
    if(pos < 165 && servoDirection == 0)    //servo to the left
    {                                  
      pos = pos + 5;                        
    } 
    if(pos > 15 && servoDirection == 1)     //servo to the right
    {                                
      pos = pos - 5;
    }    
  }   
  if (pos == 165 )    
  {
    servoDirection = 1;                     //changes direction
  }
  if (pos == 15 )     
  {
    servoDirection = 0;                     //changes direction
  }   
  myservo.write(pos);                       //move servo
}
