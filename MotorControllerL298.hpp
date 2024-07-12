/* * * * * * * * * * * * * * * * * * * * * * *
 * MOTOR CONTROLLER CLASS
 * For the L298N shield
 *
 * Code by:  Hubert Bannwarth
 * Email:    hbannw@gmail.com
 * based on the code from Simon Bluett ( hello@chillibasket.com )
 * Version:  1.0
 * Date:     27th February 2022
 * Copyright (C) 2020, MIT License
 * * * * * * * * * * * * * * * * * * * * * * */

#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

// MOTOR CONTROLLER CLASS
class MotorController {
public:
	// Constructor
	MotorController(uint8_t _dirAPin, uint8_t _pwmPin, uint8_t _dirBPin, bool _brkEnabled = true);
	
	// Functions
	void setSpeed(int pwmValue);

	// Default destructor
	~MotorController();

private:
	uint8_t dirAPin, pwmPin, dirBPin;
	bool reverse, brake, brakeEnabled;
};


/**
 * Default Constructor
 * 
 * @param  (_dirPin) Digital pin used for motor direction
 * @param  (_pwmPin) Digiral pin for PWM motor speed control
 * @param  (_brkPin) Digital pin to enable/disable the breaks
 * @param  (_brkEnabled) Should the break be used?
 */
MotorController::MotorController(uint8_t _dirAPin, uint8_t _pwmPin, uint8_t _dirBPin, bool _brkEnabled) {
	dirAPin = _dirAPin;
	pwmPin = _pwmPin;
	dirBPin = _dirBPin;
	brakeEnabled = _brkEnabled;

	pinMode(dirAPin, OUTPUT);     // Motor Direction
	pinMode(dirBPin, OUTPUT);     // Motor Brake
	//digitalWrite(dirBPin, HIGH);

	reverse = true;
	/*if (brakeEnabled) {
		digitalWrite(brkPin, HIGH);
		brake = true;
	} else {
		digitalWrite(brkPin, LOW);
		brake = false;
	}*/
}


/**
 * Default Destructor
 */
MotorController::~MotorController() {
	// Empty
}


/**
 * Set a new motor speed
 * 
 * @param  (pwmValue) The PWM value of the new speed
 * @note   Negative PWM values will cause the motor to move in reverse
 * @note   A PWM value of 0 will enable the breaks
 */
void MotorController::setSpeed(int pwmValue) {
/*  if (pwmValue != 0) {
  Serial.print(pwmValue);
  Serial.print("-");
  }*/
	// Bound the PWM value to +-255
	if (pwmValue > 255) pwmValue = 255;
	else if (pwmValue < -255) pwmValue = -255;
	
	// Forward direction
	if (pwmValue > 0 && reverse) {
		digitalWrite(dirAPin, HIGH);
		digitalWrite(dirBPin, LOW);
		reverse = false;

		// Release the brake
/*		if (brake) {
			digitalWrite(brkPin, LOW);
			brake = false;
		}*/

	// Reverse direction
	} else if (pwmValue < 0 && !reverse) {
		digitalWrite(dirAPin, LOW);
		digitalWrite(dirBPin, HIGH);
		reverse = true;

		// Release the brake
/*		if (brake) {
			digitalWrite(brkPin, LOW);
			brake = false;
		}
*/
	// If there is no movement, engage the brake
	} else if (brakeEnabled && !brake) {
//		digitalWrite(brkPin, HIGH);
//		brake = true;
	}
	
	// Send PWM value
	analogWrite(pwmPin, abs(pwmValue));
}


#endif /* MOTOR_CONTROLLER_HPP */