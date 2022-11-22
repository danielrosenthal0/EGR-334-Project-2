 
//EGR 334 Project 2
//smartWander.ino smartWander.cpp
//implement all the layers of the subsumption archiecture or state machine for smart wander which includes:
//random wander, collide and runaway so that the robot explores the world without hitting anything

#include <Arduino.h>                // include Arduino library
#include <wpi-32u4-lib.h>           // include WPI robot library
#include <Rangefinder.h>            // include sonar range finder
#include <Chassis.h>                // define robot chassis
//TO DO: update rangefinder and chassis inputs
Rangefinder rangefinder(11, 4);     // create instance of sonar echo pin 11, trigger pin 4
Chassis chassis(7.0, 1440, 14.9);   // Declares a chassis object with nominal dimensions-adjust these based on your Lab 2 values
#define LED_PIN 13                  // LED connected to builtin pin 13
//TO DO: Update pin assignment for LEDs
#define grn_LED 5                   // red LED on pin 5
#define red_LED 6                   // green LED on pin 
#define blue_LED 3
int robotSpeed = 10;                //set robot sp6eed
float distance;                     //sonar distance in cm
float inches;                       //sonar distance in inches
float error;                        //error = desired - actual distance
//TO DO: Update distances based on your Lab 3 Table 1 values
int wrnDistance = 18;               //obstacle warning distance
int dgrDistance = 12;               //obstacle danger distance

enum ROBOT_STATE {ROBOT_IDLE, ROBOT_WANDER,ROBOT_AVOID}; // Define the robot states
ROBOT_STATE robotState = ROBOT_IDLE;  // initialze robt in wander state


void setLED(bool value)             //function to turn on and off LED on pin 13 (note variable names in lines 14-17)
{
  digitalWrite(LED_PIN, value);
}

void readSonar(){                         //helper function to read ultrasonic range sensor
  distance = rangefinder.getDistance();   //distance in cm
  inches = distance*0.393701;             //distance in inches
  
}

void idle(void)                         //helper function to stop the motors
{
  
  setLED(LOW);                      // turn off LED on pin 13
  chassis.idle();                   //stop motors 
  
}


void drive(float dist, float speed)   //helper function to drive a set distance (cm, cm/s)
{
  
  setLED(HIGH);                           // turn on LED on pin 13
  //chassis.setWheelSpeeds(speed, speed);   //MOVES WHEELS FOREVER
  chassis.driveFor(dist, speed);
  
}

void turn(float ang, float speed)     //helper function to turn a set angle (deg, deg/s)
{
  //Serial.println("turn()");
  setLED(HIGH);
  //chassis.setWheelSpeeds(speed, speed);
  chassis.turnFor(ang,speed);
}



//TO DO: Modify avoid Obstacle behavior with collide and avoid primitive behaviors
void avoidObstacle(){ 
  
  analogWrite(blue_LED, 255);
  analogWrite(red_LED, 255);
  analogWrite(grn_LED, 255);

  drive(-40,20);
  while (!chassis.checkMotionComplete()) {delay(1);}
  turn(180,30);
  while (!chassis.checkMotionComplete()) {delay(1);}

//TO DO: Robot runAway behavior to move away proportional to obstacle.
}

void handleMotionComplete(void) // Used to check if the motions above are complete
{
  idle();
}


void setup() // runs this once at beginning
{
  Serial.begin(115200);               //begin serial communication for debuggint
  chassis.init();                     //initialize the chassis (which also initializes the motors)
  chassis.setMotorPIDcoeffs(5, 0.5);  //PID controller for driving robot chassis
  rangefinder.init();  
                // Call init() to set up the rangefinder
  analogWrite(red_LED,255);
  analogWrite(grn_LED,255);
  analogWrite(blue_LED,255);
  delay(1000);
  analogWrite(red_LED, 0);
  analogWrite(grn_LED, 0);
  analogWrite(blue_LED, 0);
  delay(500);                           //insert delay to get robot off test stand before moving on floor
  

}

void loop() //run continuously on the microcontroller
{
  srand(millis());
  int randBlue = rand() %255;
  int randGreen = rand() % 254;
  int randRed = rand() % 253;
  analogWrite(red_LED, randRed);
  analogWrite(grn_LED, randGreen);
  analogWrite(blue_LED, randBlue);
  readSonar();
  delay(100);                                  //read code and sonar recharge delay
  if (inches < wrnDistance) {
    robotState = ROBOT_AVOID;
  }
  

  // A basic state machine with the three basice states to start with
  switch(robotState)
  {
    case ROBOT_IDLE: 
       if(chassis.checkMotionComplete()) handleMotionComplete(); 
       break;
   
    case ROBOT_AVOID: 
        avoidObstacle();
        robotState = ROBOT_IDLE;
       break;
    default:
      break;
  }
}