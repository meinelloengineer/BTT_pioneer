// --------- General variables
unsigned long currentMillis;
unsigned long previousMillis;
unsigned long previousAngle;
unsigned long currentAngle;
unsigned long anglediff;

/**************************************************************************/
// -------------For the GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// Choose two Arduino pins to use for software serial
#define PIN_RXD 3
#define PIN_TXD 2
//Default baud of NEO-6M is 9600
#define GPS_BAUD 9600
// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(PIN_RXD, PIN_TXD);
TinyGPSPlus tinyGps;

/**************************************************************************/
// -------------For the Button and LEDs
#define LED_RED 11
#define LED_GREEN 12
#define BUTTON_PIN 13

/**************************************************************************/
// -------------For the two DC Motors in the back
#define EnA 10
#define EnB 5
#define In1 9
#define In2 8
#define In3 7
#define In4 6
int x;

/**************************************************************************/
// -------------For stepper motor
#define PUL 14
#define DIR 15

/**************************************************************************/
void setup() {
  // Start the Arduino hardware serial port at 115200 baud
  Serial.begin(115200);
  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPS_BAUD);
  // Set modes for button and LEDs
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  //Serial.setTimeout(1);
  // Initialise the sensors
  compassSetup();
  // Set modes for DC Motors
  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  // Set modes for stepper Motor
  pinMode(PUL,OUTPUT); // ustaw Pin9 jako PUL
  pinMode(DIR,OUTPUT); // ustaw Pin8 jako DIR
}
/**************************************************************************/

void loop() {
  byte buttonState = digitalRead(BUTTON_PIN);
  //compassCalibrate();
  while (buttonState == HIGH) {
    GPS_read_write(); // send GPS signal until button is pressed
    buttonState = digitalRead(BUTTON_PIN);
    //Serial.println(getCompassHeading());
  }
  // ------------- Lets gooo (Button has been pressed)
  digitalWrite(LED_RED, LOW); //turn red LED off
  digitalWrite(LED_GREEN, HIGH); //turn green LED on
  int angle = 0; 
  delay(5000); //wait for 5 seconds
  // ------------- write left Pi leg
  angle = forward(20000, angle); //drive for xx microseconds 00
      stop();
      GPS_read_write();
      delay(1000);
      GPS_read_write();
  // ------------- write upper Pi part
  angle = left_turn(90); //turn for xx degrees
      stop();
      GPS_read_write();
      delay(1000);
      GPS_read_write();
  angle = forward(6000, angle); //drive for xx microseconds
      stop();
      GPS_read_write();
      delay(2000);
      GPS_read_write();
  backward(20000, angle); //drive for xx microseconds
      stop();
      GPS_read_write();
      delay(1000);
      GPS_read_write();
  angle = forward(6000, angle); //drive for xx microseconds
      stop();
      GPS_read_write();
      delay(2000);
      GPS_read_write();
  // ------------- write right Pi leg
  angle = left_turn(90); //turn for xx degrees
      stop();
      GPS_read_write();
      delay(1000);
      GPS_read_write();
  angle = forward(16000, angle); //drive for xx microseconds
      stop();
      GPS_read_write();
      delay(1000);
      GPS_read_write();
  angle = left_curve(100); //turn for xx degrees
  GPS_read_write();
  GPS_read_write();
  // ------------- Stop the Rover and data transmission
  stop();
  digitalWrite(LED_RED, HIGH);
  while (2 > 1) {
    Serial.println(" ");
    //********* nothing
  }
}

/**************************************************************************/
// Motor
void stop(){
  // turn all motors off
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}

void backward(int drive_time, int angle){
  // Motor A 
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  analogWrite(EnA, 200); // speed usually between 150 - max. 255
  // Motor B
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  analogWrite(EnB, 200); // speed usually between 150 - max. 255
  delay(500); // half a second initial boost
  // set actual speed - going slow
  analogWrite(EnA, 80); // speed usually between 150 - max. 255
  analogWrite(EnB, 80); // speed usually between 150 - max. 255
  previousMillis = millis();
  currentMillis = previousMillis;
  int headingdiff = 0;
  int f = 0;
  float i = currentMillis/500;
  while (abs((long)previousMillis - (long)currentMillis) <= drive_time) {
    GPS_read_write();
    currentAngle = getCompassHeading();
    headingdiff = (long)previousAngle - (long)currentAngle;
    if (headingdiff>180){
    headingdiff = 360 - (long)previousAngle - (long)currentAngle;
  }
    if (currentMillis / 100 > i && headingdiff < 0 && f>-6){
      frontwheel_right(1);
      i++;
      f = f-1;
      currentMillis = millis();
    }
    if (currentMillis / 100 > i && headingdiff > 0 && f<6) {
      frontwheel_left(1);
      f = f+1;
      i++;
      currentMillis = millis();
    }
    currentMillis = millis();
  }
  if (f>0){
    frontwheel_right(f);
  }
  else{
    frontwheel_left(f);
  }
}

int forward(int drive_time, int angle){
  previousAngle = getCompassHeading();
  // Motor A
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  analogWrite(EnA, 200); // speed usually between 150 - max. 255
  // Motor B
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  analogWrite(EnB, 200); // speed usually between 150 - max. 255
  delay(500); // half a second initial boost
  // set actual speed - going slow
  analogWrite(EnA, 80); // speed usually between 150 - max. 255
  analogWrite(EnB, 80); // speed usually between 150 - max. 255
  previousMillis = millis();
  currentMillis = previousMillis;
  int i = currentMillis/100;
  int headingdiff = 0;
  int f = 0;
  while (abs((long)previousMillis - (long)currentMillis) <= drive_time) {
    GPS_read_write();
    delay(50);
    currentAngle = getCompassHeading();
    headingdiff = (long)previousAngle - (long)currentAngle;
    if (headingdiff>180){
    headingdiff = 360 - (long)previousAngle - (long)currentAngle;
  }
    if (currentMillis / 100 > i && headingdiff < 0 && f>-6){
      frontwheel_left(1);
      i++;
      f = f-1;
      currentMillis = millis();
    }
    if (currentMillis / 100 > i && headingdiff > 0 && f<6) {
      frontwheel_right(1);
      f = f+1;
      i++;
      currentMillis = millis();
    }
    currentMillis = millis();
  }
  if (f>0){
    frontwheel_left(f);
  }
  else{
    frontwheel_right(f);
  }
  return currentAngle;
}

int left_turn(int turn_degrees)  //ein Motor vorwärts, eine rückwärts
{
  frontwheel_left(40);
  delay(1000);
  previousAngle = getCompassHeading();
  int headingdiff = 0;
  while (abs(headingdiff) <= turn_degrees-5) {
    // Motor A
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  analogWrite(EnA, 100); // speed usually between 150 - max. 255
  // Motor B
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  analogWrite(EnB, 255); // speed usually between 150 - max. 255
  delay(250);
  analogWrite(EnA, 0);
  analogWrite(EnB, 0);
  GPS_read_write();
  delay(4000);
  currentAngle = getCompassHeading();
    headingdiff = (long)previousAngle - (long)currentAngle;
    if (headingdiff>180){
    headingdiff = 360 - (long)previousAngle - (long)currentAngle;
  }
  }
  delay(1000);
  frontwheel_right(40); //return front wheel to position 0
  return currentAngle;
}

int left_curve(int turn_degrees)  //ein Motor vorwärts, eine rückwärts
{
  frontwheel_left(10);
  previousAngle = getCompassHeading();
  delay(1000);
  // Motor A
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  analogWrite(EnA, 100);
  // Motor B
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  analogWrite(EnB, 200);
  
  currentAngle = previousAngle;
  int headingdiff = 0;
  while (abs(headingdiff) <= turn_degrees) {
    currentAngle = getCompassHeading();
    headingdiff = (long)previousAngle - (long)currentAngle;
    if (headingdiff>180){
    headingdiff = 360 - (long)previousAngle - (long)currentAngle;
  }
    GPS_read_write();
  }
  stop();
  delay(1000);
  frontwheel_right(10); //return front wheel to position 0
  return currentAngle;
}

/**************************************************************************/
// GPS
void GPS_read_write() {
  while (gpsSerial.available() > 0) {
    tinyGps.encode(gpsSerial.read());
    if (tinyGps.location.isUpdated()) {
      Serial.print("Latitude = ");
      Serial.print(tinyGps.location.lat(), 6);
      Serial.print(";Longitude = ");
      Serial.print(tinyGps.location.lng(), 6);
      Serial.println("");
      break;
    }
  }
}
/**************************************************************************/
//// Frontwheel

void frontwheel_left(int deg){
  digitalWrite(15,LOW); // turning front wheel left
  for(x = 0; x < round(deg*8/1.8); x++) { // going 'deg' degrees via 8/1.8-microsteps
    digitalWrite(14,HIGH);
    delayMicroseconds(500);
    digitalWrite(14,LOW);
    delayMicroseconds(500);
  }
}

void frontwheel_right(int deg){
  digitalWrite(15,HIGH); // turning front wheel right
  for(x = 0; x < round(deg*8/1.8); x++) { // going x degrees via 8/1.8-microsteps
    digitalWrite(14,HIGH);
    delayMicroseconds(500);
    digitalWrite(14,LOW);
    delayMicroseconds(500);
  }
}

