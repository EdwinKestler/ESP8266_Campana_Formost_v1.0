#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>


#define GREEN pixels.Color(0,255,0)
#define ORANGE pixels.Color(255,98,0)
#define YELLOW pixels.Color(255,255,0)
#define RED pixels.Color(255,0,0)
#define BLUE pixels.Color(0,0,255)
#define Blanco pixels.Color(255,255,255)
#define NONE pixels.Color(0,0,0)

Servo myservo;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void SET_ALARM() {
  pixels.begin();
  myservo.attach(SERVO_PIN);
  myservo.write(91);
}

void Reset_Ring(){
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, NONE);
    pixels.show();
  }
}

void Start_Ring() {

  for (int pos = 65; pos <= 115; pos += 1) {
    myservo.write(pos);
    delay(3);
  }

  for (int pos = 115; pos >= 65; pos -= 1) {
    myservo.write(pos);
    delay(3);
  }
}

void SET_GREEN() {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, GREEN);
    pixels.show();   
    delay(55);
  }
     delay(111);
  Reset_Ring();

  
}

void SET_RED() {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, RED);
    pixels.show();
    delay(55);
  }
  Start_Ring();
  Reset_Ring();
}

void SET_ORANGE() {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, ORANGE);
    pixels.show();
    delay(55);
  }
   Start_Ring();
 Reset_Ring();
}

void SET_YELLOW() {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, YELLOW);
    pixels.show();
    delay(55);
  }
   delay(111);
  Reset_Ring();
}

void SET_DEFAULT() {
  for (int i = 0; i < 3; i++) {
    pixels.setPixelColor(i, Blanco);
    pixels.show();
    delay(55);
  }
  for (int i = 3; i < 7; i++) {
    pixels.setPixelColor(i, BLUE);
    pixels.show();
    delay(55);
  }
  for (int i = 7; i <11; i++) {
    pixels.setPixelColor(i, Blanco);
    pixels.show(); 
    delay(55);
  }
  for (int i = 11; i <15; i++) {
    pixels.setPixelColor(i, BLUE);
    pixels.show(); 
    delay(55);
  }

  pixels.setPixelColor(15, Blanco);
    pixels.show(); 
    delay(55);
  
}



void ALARM_ESTATE(int ESTATE){
  switch (ESTATE){
    case 0:
    SET_DEFAULT();
    break;
    case 1:
    SET_GREEN();
    break;
    case 2:
    SET_YELLOW();
    break;
    case 3:
    SET_ORANGE();
    break;
    case 4:
    SET_RED();
    break;
  }
  
}