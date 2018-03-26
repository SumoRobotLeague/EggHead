/********************************************************************
EggHeadSurface Calibrator

Author - Eric Ryan Harrison <me@ericharrison.info>
http://github.com/SumoRobotLeague/EggJead/ contains the complete robot.

Our code is open source, MIT License (details at the end).
Please use our code in your projects and tell your friends about us!
The MRK-2 robot kit is available from http://www.SumoRobotLeague.com
********************************************************************/

// IR Sensor pins
#define leftSensor  A1
#define rightSensor A2
#define IREmitter   4

void setup() {
	Serial.begin(9600);

	// IR sensors
	pinMode(leftSensor,  INPUT_PULLUP);
	pinMode(rightSensor, INPUT_PULLUP);
	pinMode(IREmitter,   OUTPUT);

	// Turn on our IR Emitter
	digitalWrite(IREmitter, HIGH);
}

void loop() {
	int leftIR   = analogRead(leftSensor);
	int rightIR  = analogRead(rightSensor);

	int avg = ( leftIR + rightIR ) / 2;

	Serial.print("Threshold to use: ");
);
	Serial.println(avg);
}
