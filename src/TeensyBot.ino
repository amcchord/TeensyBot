#include <Adafruit_NeoPixel.h>
#include <PulsePosition.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>


//Trim for RC Inputs
const int rcMin = 1099;
const int rcMax = 1920;
int rcScale = rcMax - rcMin;
#define FAILSAFE false //Failsafe is disabled for now
#define DEADBAND 10 //If thrust values are within +/-10 of 0 assume they are 0
#define REJECTTHRESH 2200 //Rc values above this number are considered invalid

//Create some global variables to store the state of RC Reciver Channels
double rc1 = 0; // Turn
double rc2 = 0; // Thrust
double rc3 = 0; // Weapon Power
double rc4 = 0; // Weapon Directon
double rc5 = 0; // RainbowMode!
double rc6 = 0; // Failsafe (Not used yet)

//Define the PPM decoder object
PulsePositionInput myIn;

//Define the ports that control the motors
//Motor Driver Outputs
#define LEFTTHROTTLE 22
#define LEFTDIRECTION 15
#define RIGHTTHROTTLE 23
#define RIGHTDIRECTION 16


//AHRS Vars
#define BNO055_SAMPLERATE_DELAY_MS (10)
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);
bool inverted = 0;
double targetHeading = 0;
double targetBalance = 5;

double eulerX = 0;
double eulerY = 0;
double eulerZ = 0;


//Define Variables we'll be connecting to
double consKp=9, consKi=60, consKd=0.20;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
//Define the aggressive and conservative Tuning Parameters


double dKp=1.0, dKi=0.05, dKd=.17;
double driveGoal, driveIn, driveOut;
PID drivePID(&driveIn, &driveOut, &driveGoal, dKp, dKi, dKd, DIRECT);

bool pidEnabled = false;


//Some globals for handling mode switching
int lastMode = 0;
int pixelTicker = 0;

void setup() {

  myIn.begin(6); // Start reading the data from the RC Reciever on Pin 6
  Serial.begin(9600);

  pinMode(13,OUTPUT); //Just make sure we can use the onboard LED for stuff
  pinMode(LEFTTHROTTLE, OUTPUT); //Tell the controller we want to use these pins for output
  pinMode(LEFTDIRECTION, OUTPUT);
  pinMode(RIGHTTHROTTLE, OUTPUT);
  pinMode(RIGHTDIRECTION, OUTPUT);

  analogWrite(LEFTTHROTTLE, 0);
  digitalWrite(LEFTDIRECTION, 0);
  analogWrite(RIGHTTHROTTLE, 0);
  digitalWrite(RIGHTDIRECTION, 0);

  Wire.begin();
  if(!bno.begin())
  {
    Serial.println("AHRS: ERROR");
  }
  Serial.println("AHRS: Calibrating");

  //Lets wait 1 second everything to come online and be ready before hitting the main loop
  //This delay prevents motor jerk at power up and should reduce connector sparking
  digitalWrite(13, HIGH);
  delay(5000);
  digitalWrite(13, LOW);

  // leftPixels.begin();
  // leftPixels.show(); // Initialize all pixels to 'off'
  // rightPixels.begin();
  // rightPixels.show(); // Initialize all pixels to 'off'
  myPID.SetOutputLimits(-245, 245);
  drivePID.SetOutputLimits(-15,15);
  driveGoal = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  drivePID.SetMode(AUTOMATIC);
  drivePID.SetSampleTime(10);

}

void loop() { //The main program loop;
  updateChannels();

  //Scale the raw RC input
  int thrust = round(((rc2 - rcMin)/rcScale) * 500) - 250; //Cast to -250-0-250
  int turn = round(((rc1 - rcMin)/rcScale) * 500) - 250; //Cast to -250-0-250
  int weapon = round(((rc3 - rcMin)/rcScale) * 250); //Cast to 0-250
  int direction = round(((rc4 - rcMin)/rcScale) * 10) - 5; //Cast to -5-0-5;
  int ledMode = round(((rc5 - rcMin)/rcScale) * 6);

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  eulerZ = euler.z();
  eulerX = euler.x();
  eulerY = euler.y();


  //Apply Deadband Correction
  if (thrust < DEADBAND && thrust > (DEADBAND * -1)){
    thrust = 0;
  }
  if (turn < DEADBAND && turn > (DEADBAND * -1)){
    turn = 0;
  }

  //If we have a serial port attached we can debug our inputs.
  Serial.print("Thrust: ");
  Serial.print(thrust);
  Serial.print(" Turn: ");
  Serial.print(turn);
  Serial.print(" X,Y,Z ");
  Serial.print(eulerX);
  Serial.print(" ");
  Serial.print(eulerY);
  Serial.print(" ");
  Serial.print(eulerZ);
  Serial.println("");

  if (eulerZ < 45 && eulerZ > -20){
    //Upside Down Mode
    simpleDrive(thrust * -1, turn * -1);

  }
  else {
    //Normal Mode
    simpleDrive(thrust, turn);

  }


//  pidDrive();
//  pidBalance();
  delay(10);

}




void pidBalance(int turn){
  double currentAngle =  eulerZ -90;
  Input = currentAngle;
  myPID.Compute();

  int correctedOut = 0;
  if (Output > 0){
    correctedOut = Output + 40;
  }
  else if (Output < 0){
    correctedOut = Output - 40;
  }
  else {
    correctedOut = Output;
  }


  Serial.print(" trgt: ");
  Serial.print(Setpoint);
  Serial.print(" cur: ");
  Serial.print(currentAngle);
  Serial.print(" resp: ");
  Serial.print(correctedOut);

  Serial.print(" dG: ");
  Serial.print(driveGoal);
  Serial.print(" dI: ");
  Serial.print(driveIn);



    if (turn < 5 && turn > -5){ //Don't apply drive corrections during steering
      driveIn = driveIn + ((correctedOut ) / 400.0);
    } else {
      driveIn = driveIn + (((correctedOut ) / 400.0) / (turn / 2)); //Apply them slightly less depending on steering strength.
    }
    simpleDrive(correctedOut, turn );


}


void pidDrive(int thrust){
  driveGoal = driveGoal + thrust/10;
  if (driveGoal < -25){
    driveGoal = -25;
  }
  else if (driveGoal > 25){
    driveGoal = 25;
  }

  if (driveIn < -25){
    driveIn = -25;
  }
  else if (driveIn > 25){
    driveIn = 25;
  }
  if (driveIn > 20 && driveGoal > 20){
    driveIn = driveIn - 20;
    driveGoal = driveGoal - 20;
  }
  if (driveIn < -20 && driveGoal < -20){
    driveIn = driveIn + 20;
    driveGoal = driveGoal + 20;
  }

  drivePID.Compute();
  Setpoint = driveOut;

}

//This function does the steering interpretation from 2 channels.
//Thrust is how fast you want to go. +255 max forward -255 is max reverse
//Turn is how hard do you want to turn.
void simpleDrive(double thrust, double turn){
  int left = 0;
  int right = 0;

  //This is where the turning logic is.. That's it.
  left = thrust + turn;
  right = thrust - turn;

  right = right * -1; //Invert Right Drive motor

  //Safety checks!
  if (left > 255){
    left = 255;
  }
  else if (left < -255){
    left = -255;
  }

  //If the left motor needs to go forward.
  if (left > 0){
    analogWrite(LEFTTHROTTLE, left);
    digitalWrite(LEFTDIRECTION, 0);

  } else { //Left motor needs to spin backward
    analogWrite(LEFTTHROTTLE, left * -1); //Flip the speed to positive
    digitalWrite(LEFTDIRECTION, 1);
  }


  //Same thing for the right side
  if (right > 255){
    right = 255;
  }
  else if (right < -255){
    right = -255;
  }

  if (right > 0){
    analogWrite(RIGHTTHROTTLE, right);
    digitalWrite(RIGHTDIRECTION, 0);

  } else {
    analogWrite(RIGHTTHROTTLE, right * -1); //Flip the speed to positive
    digitalWrite(RIGHTDIRECTION, 1);
  }
}

//Read in the channels from the RC reciver
void updateChannels(){

  int num = myIn.available();
  if (num > 0) {

    int rc1t = myIn.read(1);
    int rc2t = myIn.read(2);
    int rc3t = myIn.read(3);
    int rc4t = myIn.read(4);
    int rc5t = myIn.read(5);
    int rc6t = myIn.read(6);

    //Don't register weird outliers!
    if (rc1t > 0 && rc1t < REJECTTHRESH){
      rc1 = rc1t;
    }
    if (rc2t > 0 && rc2t < REJECTTHRESH){
      rc2 = rc2t;
    }
    if (rc3t > 0 && rc3t < REJECTTHRESH){
      rc3 = rc3t;
    }
    if (rc4t > 0 && rc4t < REJECTTHRESH){
      rc4 = rc4t;
    }
    if (rc5t > 0 && rc5t < REJECTTHRESH){
      rc5 = rc5t;
    }
    if (rc6t > 0 && rc6t < REJECTTHRESH){
      rc6 = rc6t;
    }

    if (rc6 > 2000 && FAILSAFE){ //Will shutdown is reciever is programed correctly for failsafe
      rc1 = 0;
      rc2 = 0;
      rc3 = 0;
      rc4 = 0;
      rc5 = 0;
      rc6 = 0;
    }
  }
}


void enablePID(){
  driveGoal = 0;
  Setpoint = 0;

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  drivePID.SetMode(AUTOMATIC);
  drivePID.SetSampleTime(10);
  pidEnabled = true;
}
void disablePID(){
  driveGoal = 0;

  myPID.SetMode(MANUAL);
  myPID.SetSampleTime(10);
  drivePID.SetMode(MANUAL);
  drivePID.SetSampleTime(10);
  pidEnabled = false;
}
