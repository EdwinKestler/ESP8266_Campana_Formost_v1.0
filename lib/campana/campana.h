#include <Arduino.h>
#include <Servo.h>

#define SERVO_PIN D3    // control pin for servo motor
#define MIN_ANGLE 90   // minimum servo position
#define MAX_ANGLE 150  // maximum servo position
unsigned long nextMillis = 0;
int angle = 0;         // amount to angle the servo
int rings = 0;         // amount of rings
int state = 0;         // for the state machine
Servo servo;

// define states for the state machine 
#define PULSE_ON       0
#define WAIT_PULSE_ON  1
#define PULSE_OFF      2
#define WAIT_PULSE_OFF 3

void Rings(int rings)
{
    // read the rings
    if (rings > 0) 
    {
        Serial.println(rings);
    }
    
    // state machine to control, which action to perform
    if (rings > 0) 
    {
        switch (state) 
        {
            case PULSE_ON:
            angle = MAX_ANGLE;                // send servo to max position
            nextMillis = millis() + 300;     // wait 300 ms
            state = WAIT_PULSE_ON;
            break;
            
            case WAIT_PULSE_ON:
            if (millis() > nextMillis) 
            {
                state = PULSE_OFF;             // time is up
            }
            break;

            case PULSE_OFF:
            angle = MIN_ANGLE;                // send servo to min position
            nextMillis = millis() + 1000;    // wait 1 second between two rings
            state = WAIT_PULSE_OFF;
            break;
            
            case WAIT_PULSE_OFF:
            if (millis() > nextMillis) 
            {
                state = PULSE_ON;              // time is up
                rings--;                       // one ring is done
            }
            break;
        }
    }
    servo.write(angle);
    //Servo::refresh();
}