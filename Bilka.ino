/*
 * Closed loop control of the take-up reel for the laser cutter
 *
 * This uses an external stepper connected on pins 8,9,10 and 11
 *
 * An ultrasonic sensor with the triggger on pin 5 and the echo on pin 6
 *
 * and a 16x2 LCD connected through i2c with SCL on A5 and SDA on A4
 *
 */

#include <TimerOne.h>
#include <NewPing.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <Wire.h>


#define TRIG_PIN 2
#define ECHO_PIN 3
#define ECHO_INT 0

void Timer1Callback(void);
void echoCheck();

#define TARGET_DIST 50	// how far in cm is the desired ZERO hight
#define LIMIT_DIST 15	// how much deviation from ZERO is ok

// init the ultra sonic library
NewPing sonar(TRIG_PIN, ECHO_PIN, 200);

// initialize the stepper library on pins 8 and 9
AccelStepper stepper(AccelStepper::DRIVER, 8, 9);

// Init the i2c LCD control at address 0x27
LiquidCrystal_I2C lcd(0x27,16,2);


const unsigned int pingSpeed = 250;	// How frequently are we going to send out a ping (in milliseconds).


// Manage LCD update speed relative to Ping Speed
const unsigned int displayUpdateSpeed = 1000;
const unsigned int numPingsPerDisplayInterval = displayUpdateSpeed / pingSpeed;
unsigned int numPingsSinceLastDisplay = 0;

unsigned long pingTimer = 0;	// Holds the next ping time.

uint8_t syncByte = 0;			// sync LCD callback and main loop

#define SPEED_RPS (1.125)					// 67.5 RPM
#define FULL_STEPS_PER_REV (200.0)			// This is a characteristic of the motor
#define DRIVER_MICROSTEP_FACTOR (2.0)		// We are half-stepping

#define ACCELERATION (3.0)					// Revs per sec^2

typedef enum
{
	MOTOR_NOT_RUNNING,
	MOTOR_RUNNING_UP,
	MOTOR_RUNNING_DOWN
} MotorStatus_t;



char motorStatusPrefix[] = "Status: ";

char const *motorStatusMsgs[] =
{
//   012345678901
	"Stopped",
	"Up     ",
	"Down   "
};

MotorStatus_t motorStatus = MOTOR_NOT_RUNNING;

char distString[32];


void DisplayMotorStatus(void) {
	// to the lcd in the top row	
	lcd.setCursor(0,0);
	lcd.print(motorStatusPrefix);
	lcd.print(motorStatusMsgs[motorStatus]);

	// and out to the serial port
	Serial.println(motorStatusMsgs[motorStatus]);
}

void DisplaySecondLine(char* message) {
	// to the lcd in the bottom row	
	lcd.setCursor(0,1);
	lcd.print(message);

	// and out to the serial port
	Serial.println(message);
}
	
// This is the callback fro the timer1 interrupt which we
// have set up to trigger every 100uS. When the run call
// has real work to do it delays quite a bit for the
// timing of the step for the motor so se shut off the
// interrupt and callback for the duration
void Timer1Callback(void) {
	Timer1.stop();
	stepper.run();
	Timer1.resume();
}




void setup() {
	// get serial port going
	Serial.begin (115200);
	
	// set stepper motor speed in steps per second
	stepper.setMaxSpeed(SPEED_RPS * FULL_STEPS_PER_REV * DRIVER_MICROSTEP_FACTOR);


	// Set Acceleration in steps per sec^2
	stepper.setAcceleration(ACCELERATION * FULL_STEPS_PER_REV * DRIVER_MICROSTEP_FACTOR);

	// Set up timer1 to trigger every 100 uS and
	// set the callback for when it occurs
	Timer1.initialize(100);
	Timer1.attachInterrupt(Timer1Callback); // blinkLED to run every 0.15 seconds

	// lcd init
	lcd.init();
	lcd.backlight();
}


void loop() {

	if (millis() >= pingTimer) {		// pingSpeed milliseconds since last ping, do another ping.
		pingTimer += pingSpeed;			// Set the next ping time.
		digitalWrite(12, HIGH);
		sonar.ping_timer(echoCheck);	// Send out the ping, calls "echoCheck" function every
										// 24uS where you can check the ping status.

	}

	// detemine if the sensor got a reading
	// then update the LCD
	if (syncByte) {
		syncByte = 0;

		// do we need to update the display
		if (++numPingsSinceLastDisplay >= numPingsPerDisplayInterval) {
			numPingsSinceLastDisplay = 0;
			DisplayMotorStatus();
			DisplaySecondLine(distString);
		}
	}

}


// Timer2 interrupt calls this function
// every 24uS where you can check the ping status.
void echoCheck() {
	long dist;

	if (sonar.check_timer()) {		// This is how you check to see if the ping was received.
		dist = sonar.ping_result / US_ROUNDTRIP_CM - TARGET_DIST;

		if ((motorStatus == MOTOR_RUNNING_UP) && (dist < 0)) {
			stepper.stop();
			motorStatus = MOTOR_NOT_RUNNING;
		}
		else if ((motorStatus == MOTOR_RUNNING_DOWN) && (dist > 0)) {
			stepper.stop();
			motorStatus = MOTOR_NOT_RUNNING;
		} else if ((dist > LIMIT_DIST) && (motorStatus != MOTOR_RUNNING_UP)) {
			stepper.setCurrentPosition(0);
			stepper.moveTo(-100000);
			motorStatus = MOTOR_RUNNING_UP;
		}
		else if ((dist < -LIMIT_DIST) && (motorStatus != MOTOR_RUNNING_DOWN)) {
			stepper.setCurrentPosition(0);
			stepper.moveTo(100000);
			motorStatus = MOTOR_RUNNING_DOWN;
		}

		// make a nice string to display that has the distance
		sprintf(distString, "Dist:  %3ld cm", dist);

		// all done, let the main loop know
		syncByte = 1;
	}
}
