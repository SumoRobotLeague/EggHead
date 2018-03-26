/********************************************************************
EggHead - Easter Egg Hunting Robot

Author - Eric Ryan Harrison <me@ericharrison.info>
http://github.com/SumoRobotLeague/EggJead/ contains the complete robot.

Our code is open source, MIT License (details at the end).
Please use our code in your projects and tell your friends about us!
The MRK-2 robot kit is available from http://www.SumoRobotLeague.com
********************************************************************/

// To calculate the surface threshold you need to use, run
// the code found in SensorCalibration.ino on your robot with the
// Serial Monitor open. Whatever number it spits out, you'll need to pick
// a lower number and put it here. If it says something like "456", you'll want
// to pick a number like 300. Take a look at the output of the test code
// over your boundary lines (tape) and make sure this number is slightly higher
// than that number. Generally paper and tape create very low numbers like 50.
#define SurfaceThreshold 200
#define eggDistance 20 // distance in centimeters to find an egg

/********************* CODE BELOW *********************/

#include "Motor.h"
#include "Pitches.h"

#define led 13

// Hardware pins
#define button 2
#define buzzer 3

// Motor pins
#define leftMotorSpeed 5
#define rightMotorSpeed 6
#define leftMotorDirection 7
#define rightMotorDirection 8

// Ultrasonic sensor pins
#define echoPin A0
#define pingPin 10

// IR Sensor pins
#define leftSensor  A1
#define rightSensor A2
#define IREmitter   4

// create our motor object
Motor motor;

int buttonState = 1; // 1 = up, 0 = down
int state       = 0; // 0 = idle, 1 = running, 2 = found egg

// competition configuration variables
int turnDirection      = 1; // 1 = left, 2 = right
unsigned long turnTime = 0; // holds the return from millis()
int searchTime         = 2000;

void setup() {
	Serial.begin(9600);

	// pin setup
	pinMode(button, INPUT_PULLUP);
	pinMode(buzzer, OUTPUT);

	// IR sensors
	pinMode(leftSensor,  INPUT_PULLUP);
	pinMode(rightSensor, INPUT_PULLUP);
	pinMode(IREmitter,   OUTPUT);

	// ultrasonic sensors
	pinMode(pingPin, OUTPUT);
	pinMode(echoPin, INPUT);

	// Motor setup
	motor.setupRight(rightMotorSpeed, rightMotorDirection);
	motor.setupLeft(leftMotorSpeed, leftMotorDirection);

	// Turn on our IR Emitter
	digitalWrite(IREmitter, HIGH);
}

void loop() {
	if ( buttonState == 1 && digitalRead(button) == 0 ) {
		buttonState = 0;
	}
	if ( buttonState == 0 && digitalRead(button) == 1 ) {
		buttonState = 1;

		if ( state == 0 ) {
			// we were in idle mode, so let's begin our
			// competition countdown
			state = 1;
			Serial.println("EGGS!");
			Serial.println("Good luck!");
		} else {
			state = 0;
			motor.left(0);
			motor.right(0);
			Serial.println("Robot entering idle mode.");
		}
	}

	if ( state == 1 ) {
		hunt();
	} else if ( state == 2 ) {
		playSong();
	}
}

void hunt() {
  // read our sensor values to determine what we
  // need our robot to do.
  int distance = msToCm( ping() );
  int leftIR   = analogRead(leftSensor);
  int rightIR  = analogRead(rightSensor);

  if ( leftIR < SurfaceThreshold || rightIR < surfaceThreshold ) {
	// if we have detected a ring border, abort!
	abortBackup();
  } else if ( distance < eggDistance ) {
	// We have detected an enemy. Attack!
	goToEgg();
  } else {
	// We don't see an enemy, search until we find one.
	search();
  }
}

// This function is a simple search algorithm that
// attempts to locate the opponent before moving
// into attack mode.
void search() {
	// our time has exceeded our configured searchTime, reset.
	if ( turnTime != 0 && ((millis() - turnTime) > searchTime) ) {
		turnTime = 0;
	}

	// get random direction and store the start time
	if ( turnTime == 0 ) {
		turnDirection = random(1, 3);
		turnTime  = millis();
	}

	// start our turn, 1 = left, 2 = right
	if ( turnDirection == 1 ) {
		motor.left(255);
		motor.right(-255);
	} else {
		motor.left(-255);
		motor.right(255);
	}
}

void goToEgg() {
	motor.left(255);
	motor.right(255);
}

// The abortBackup() function is an interrupting function that
// is called whenever the infrared sensors detect the
// ring border. This function will immediately halt
// other control function operations and move in reverse
// at full speed.
void abortBackup() {
	motor.left(-255);
	motor.right(-255);
	delay(2000);
}

void playSong() {
	playNote(NOTE_G6, 200, 15);
	delay(200);
	playNote(NOTE_G5, 200, 15);
	delay(200);
	playNote(NOTE_G4, 200, 15);
	delay(200);
	playNote(NOTE_G3, 200, 15);
	delay(200);
	playNote(NOTE_G2, 200, 15);
	delay(200);
	playNote(NOTE_G1, 200, 15);
	delay(200);

	state = 0;
}

/**********************/
/** Helper Functions **/
/**********************/
// Note-playing buzzer helper function.
void playNote(int note, int duration, int rest) {
	tone(buzzer, note, duration);
	delay(rest);
}

// Helper function to manage our ultrasonic sensor.
long ping() {
	long duration;
	digitalWrite(pingPin, LOW);
	delayMicroseconds(2);
	digitalWrite(pingPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(pingPin, LOW);
	duration = pulseIn(echoPin, HIGH, 5500);  //setting the timeout to 5500 is helpful but will return 0 whenever pulseIn() times out
	if (duration == 0) {
		duration = 5500;  //here we change the 0 value to the actual timeout limit
	}
	return duration;
}

// Helper function to return the distance to an object
// detected by the ultrasonic sensor in centimeters.
long msToCm(long microseconds) {
	return microseconds / 29 / 2;
}

// Simple blink function called in loop() whenever a state
// change is made by a user button press.
void blink(int blinks) {
	for ( int i = 0; i <= blinks; i++ ) {
		digitalWrite(led, HIGH);
		delay(500);
		digitalWrite(led, LOW);
		delay(500);
	}
}
/*********************************************************************************************************************
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'*******************************************************************************************************************/
