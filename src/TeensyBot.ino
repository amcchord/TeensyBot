#include <Adafruit_NeoPixel.h>
#include <PulsePosition.h>
#include <Servo.h>



//Trim for RC Inputs
const int rcMin = 1099;
const int rcMax = 1920;
int rcScale = rcMax - rcMin;
#define FAILSAFE false //Failsafe is disabled for now
#define DEADBAND 10 //If thrust values are within +/-10 of 0 assume they are 0

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

//Define the blinking lights
Adafruit_NeoPixel leftPixels = Adafruit_NeoPixel(8, 10, NEO_GRBW + NEO_KHZ800);  //LED Count then output Pin
Adafruit_NeoPixel rightPixels = Adafruit_NeoPixel(8, 3, NEO_GRBW + NEO_KHZ800);


void setup() {

  myIn.begin(6); // Start reading the data from the RC Reciever on Pin 6
  Serial.begin(9600);

  pinMode(13,OUTPUT); //Just make sure we can use the onboard LED for stuff
  pinMode(LEFTTHROTTLE, OUTPUT); //Tell the controller we want to use these pins for output
  pinMode(LEFTDIRECTION, OUTPUT);
  pinMode(RIGHTTHROTTLE, OUTPUT);
  pinMode(RIGHTDIRECTION, OUTPUT);
  pinMode(WEAPONTHROTTLE, OUTPUT);
  pinMode(WEAPONDIRECTION, OUTPUT);

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
  colorFill(leftPixels.Color(0,128,0), leftPixels);
  rightPixels.begin();
  rightPixels.show(); // Initialize all pixels to 'off'
  colorFill(rightPixels.Color(128,0,0), rightPixels);

}

void loop() { //The main program loop;
  updateChannels();

  //Scale the raw RC input
  int thrust = round(((rc2 - rcMin)/rcScale) * 500) - 250; //Cast to -250-0-250
  int turn = round(((rc1 - rcMin)/rcScale) * 500) - 250; //Cast to -250-0-250
  int weapon = round(((rc3 - rcMin)/rcScale) * 250); //Cast to 0-250
  int direction = round(((rc4 - rcMin)/rcScale) * 10) - 5; //Cast to -5-0-5;

  //Apply Deadband
  if (thrust < DEADBAND && thrust > (DEADBAND * -1)){
    thrust = 0;
  }
  if (turn < DEADBAND && turn > (DEADBAND * -1)){
    turn = 0;
  }
  if (weapon < DEADBAND){
    weapon = 0;
  }


  Serial.print("Thrust: ");
  Serial.print(thrust);
  Serial.print(" Turn: ");
  Serial.print(turn);
  Serial.print(" Weapon: ");
  Serial.print(weapon);
  Serial.print(" Direction: ");
  Serial.println(direction);

  simpleDrive(thrust, turn);
  weaponControl(weapon, direction);

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

void updateChannels(){
  int num = myIn.available();
  if (num > 0) {
    rc1 = myIn.read(1);
    rc2 = myIn.read(2);
    rc3 = myIn.read(3);
    rc4 = myIn.read(4);
    rc5 = myIn.read(5);
    rc6 = myIn.read(6);

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



//These functions are for making fun colors!

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< leftPixels.numPixels(); i++) {
      leftPixels.setPixelColor(i, Wheel(((i * 256 / leftPixels.numPixels()) + j) & 255));
      rightPixels.setPixelColor(i, Wheel(((i * 256 / leftPixels.numPixels()) + j) & 255));
    }
    leftPixels.show();
    rightPixels.show();
    delay(wait);
  }
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait, Adafruit_NeoPixel strip) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
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
