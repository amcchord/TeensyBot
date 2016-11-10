#include <Adafruit_NeoPixel.h>
#include <PulsePosition.h>
#include <Servo.h>



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
#define LEFTTHROTTLE 21
#define LEFTDIRECTION 19
#define RIGHTTHROTTLE 22
#define RIGHTDIRECTION 16
#define WEAPONTHROTTLE 23
#define WEAPONDIRECTION 18

//Using a speedcontroller instead
#define WEAPONSERVO 23
Servo myservo;  


//Define the blinking lights
Adafruit_NeoPixel leftPixels = Adafruit_NeoPixel(8, 10, NEO_GRBW + NEO_KHZ800);  //LED Count then output Pin
Adafruit_NeoPixel rightPixels = Adafruit_NeoPixel(8, 3, NEO_GRBW + NEO_KHZ800);

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


  //We don't need these because we are using a ESC
  // pinMode(WEAPONTHROTTLE, OUTPUT);
  // pinMode(WEAPONDIRECTION, OUTPUT);


   myservo.attach(23);

  analogWrite(LEFTTHROTTLE, 0);
  digitalWrite(LEFTDIRECTION, 0);
  analogWrite(RIGHTTHROTTLE, 0);
  digitalWrite(RIGHTDIRECTION, 0);


  //Lets wait 1 second everything to come online and be ready before hitting the main loop
  //This delay prevents motor jerk at power up and should reduce connector sparking
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);

  leftPixels.begin();
  leftPixels.show(); // Initialize all pixels to 'off'
  rightPixels.begin();
  rightPixels.show(); // Initialize all pixels to 'off'


}

void loop() { //The main program loop;
  updateChannels();

  //Scale the raw RC input
  int thrust = round(((rc2 - rcMin)/rcScale) * 500) - 250; //Cast to -250-0-250
  int turn = round(((rc1 - rcMin)/rcScale) * 500) - 250; //Cast to -250-0-250
  int weapon = round(((rc3 - rcMin)/rcScale) * 180); //Cast to 0-180 //We are using the servo library here
  int direction = round(((rc4 - rcMin)/rcScale) * 10) - 5; //Cast to -5-0-5;
  int ledMode = round(((rc5 - rcMin)/rcScale) * 6);


  //Apply Deadband Correction
  if (thrust < DEADBAND && thrust > (DEADBAND * -1)){
    thrust = 0;
  }
  if (turn < DEADBAND && turn > (DEADBAND * -1)){
    turn = 0;
  }
  if (weapon < DEADBAND){
    weapon = 0;
  }

  //If we have a serial port attached we can debug our inputs.
  Serial.print("Thrust: ");
  Serial.print(thrust);
  Serial.print(" Turn: ");
  Serial.print(turn);
  Serial.print(" Weapon: ");
  Serial.print(weapon);
  Serial.print(" Direction: ");
  Serial.print(direction);
  Serial.print(" LedMode: ");
  Serial.println(ledMode);


  simpleDrive(thrust, turn);
  //weaponControl(weapon, direction);
  myservo.write(weapon);
  ledMagic(ledMode);

}

//Controls the speed data to the weapon speed controller.
void weaponControl(int speed, int direction){
  if (direction < 0){ //Set direction based on positive or negative
    speed = speed * -1;
  }
  if (speed > 0){
    analogWrite(WEAPONTHROTTLE, speed);
    digitalWrite(WEAPONDIRECTION, 0);

  } else { //Left motor needs to spin backward
    analogWrite(WEAPONTHROTTLE, speed * -1); //Flip the speed to positive
    digitalWrite(WEAPONDIRECTION, 1);
  }

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

//Make the dual NeoPixel Strips do stuff!
void ledMagic(int mode){
  if (mode == 0 && mode != lastMode){ //Turn the lights off!
    leftPixels.begin();
    rightPixels.begin();
    colorFill(leftPixels.Color(0,0,0,0), leftPixels);
    colorFill(rightPixels.Color(0,0,0,0), rightPixels);
  }
  if (mode == 1 && mode != lastMode){ //Turn the lights white!
    leftPixels.begin();
    rightPixels.begin();
    colorFill(leftPixels.Color(0,0,0,255), leftPixels);
    colorFill(rightPixels.Color(0,0,0,255), rightPixels);
  }

  if (mode == 2){ //Set the color of the lights based on the throttle.
    leftPixels.begin();
    rightPixels.begin();
    colorFill(Wheel(round(((rc2 - rcMin)/rcScale) * 250)), leftPixels);
    colorFill(Wheel(round(((rc2 - rcMin)/rcScale) * 250)), rightPixels);
  }


  else if (mode == 3){ //Rainbow Flashers
    leftPixels.begin();
    rightPixels.begin();
    for(int i=0; i< leftPixels.numPixels(); i++) {
      if (pixelTicker % leftPixels.numPixels() == i){
          leftPixels.setPixelColor(i, Wheel(pixelTicker));
      }
      else {
          leftPixels.setPixelColor(i, leftPixels.Color(0,0,0));
      }
    }
    leftPixels.show();
    for(int i=0; i< rightPixels.numPixels(); i++) {
      if (pixelTicker % rightPixels.numPixels()  == i){
          rightPixels.setPixelColor(i, Wheel(pixelTicker));
      }
      else {
          rightPixels.setPixelColor(i, rightPixels.Color(0,0,0));
      }
    }
    rightPixels.show();

  }
  else if (mode == 4){ //Syncronized Rainbow Pulse!
    leftPixels.begin();
    rightPixels.begin();
    colorFill(Wheel(pixelTicker*4), leftPixels);
    colorFill(Wheel(pixelTicker*4), rightPixels);
  }
  else if (mode == 5){ //Desyncronized Raindbow Pulse
    leftPixels.begin();
    rightPixels.begin();
    colorFill(Wheel(pixelTicker*4), leftPixels);
    colorFill(Wheel(pixelTicker*-4), rightPixels);
  }
  else if (mode == 6){ //Rainbow Wheel!
    leftPixels.begin();
    rightPixels.begin();
    for(int i=0; i< leftPixels.numPixels(); i++) {
      leftPixels.setPixelColor(i, Wheel(((i * 256 / leftPixels.numPixels()) + pixelTicker*3) & 255));
    }
    for(int i=0; i< rightPixels.numPixels(); i++) {
      rightPixels.setPixelColor(i, Wheel(((i * 256 / rightPixels.numPixels()) + pixelTicker*3) & 255));
    }
    leftPixels.show();
    rightPixels.show();
  }



  lastMode = mode;
  pixelTicker++;
  if (pixelTicker > 256){
    pixelTicker = 0;
  }

  //NeoPixel Pushing makes a mess of the PPM timers.. so we can use this trick to reset them.
  if (lastMode != 1 && lastMode != 0){
    delay(30);
  }
}

// Fill the dots one after the other with a color
void colorFill(uint32_t c, Adafruit_NeoPixel strip) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return leftPixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return leftPixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return leftPixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
