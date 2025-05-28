/**
 * Wall-e Animation Presets
 * 
 * @file      animations.ino
 * @brief     Servo motor animations for various Wall-E movements
 * @author    Simon Bluett
 * @copyright MIT license
 * @version   2.9
 * @date      7th August 2020
 *
 * Added eybrows control by hbannw 27th February 2024
 * 
 * Preset animations for all the servo motors can be defined here.
 * The animations consist of a set of position instructions for each
 * servo motor, along with a specified time for how long that position
 * should be held until the next animation instruction is executed.
 *
 * To add your own animation, follow these steps:
 * 1. Add a new 'case' statement with the number you want to associate with your animation.
 * 2. Follow the same format as below to add new movement instructions
 *    queue.push({time, head rotation, neck top, neck bottom, eye right, eye left, arm left, arm right, eyebrow left, eyebrow right})
 *    > The time needs to be a number in milliseconds
 *    > Each servo position value needs to be an integer (whole) number between 0-100
 *    > 0 = LOW servo position, 100 = HIGH servo position as specified in the wall-e_calibration sketch
 *    > If you want the servo to be disabled/not updated by a specific instruction, use -1
 * 3. Make sure that your 'case' statement ends with the "break;" command 
 */

#ifndef ANIMATIONS_INO
#define ANIMATIONS_INO

void playAnimation(int animationNo) {
    Serial.print(F("Animation n°"));
    Serial.println(animationNo);

	switch (animationNo) {
		case 0:
			// --- Reset Servo positions ---
			//          time,head,necT,necB,eyeR,eyeL,armL,armR,EybL,EybR
			queue.push({1000,  50,  10,  0,   0,   0,  40,  40,  0,  0});
			break;

		case 1:
			// --- Bootup Eye Sequence ---
			//          time,head,necT,necB,eyeR,eyeL,armL,armR,EybL,EybR
			queue.push({2000,  50,  45,  90,  40,  40,  40,  40,100,100});
			queue.push({ 700,  50,  45,  90,  40,   0,  40,  40,100,100});
			queue.push({ 700,  50,  45,  90,   0,   0,  40,  40,1,1});
			queue.push({ 700,  50,  45,  90,   0,  40,  40,  40,0,0});
			queue.push({ 700,  50,  45,  90,  40,  40,  40,  40,0,0});
			queue.push({ 400,  50,  45,  90,   0,   0,  40,  40,0,0});
			queue.push({ 400,  50,  45,  90,  40,  40,  40,  40,0,0});
			queue.push({2000,  50,   0,  60,  40,  40,  40,  40,0,0});
			queue.push({1000,  50,   0,  60,   0,   0,  40,  40,0,0});
			break;

		case 2:
			// --- Inquisitive motion sequence ---
			//          time,head,necT,necB,eyeR,eyeL,armL,armR,EybL,EybR
			queue.push({3000,  48,  40,   0,  35,  45,  60,  59,0,0});
			queue.push({1500,  48,  40,  20, 100,   0,  80,  80,0,0});
			queue.push({3000,   0,  40,  40, 100,   0,  80,  80,0,0});
			queue.push({1500,  48,  60, 100,  40,  40, 100, 100,100,100});
			queue.push({1500,  48,  40,  30,  45,  35,   0,   0,0,0});
			queue.push({1500,  34,  34,  10,  14, 100,   0,   0,0,0});
			queue.push({1500,  48,  60,  20,  35,  45,  60,  59,0,0});
			queue.push({3000, 100,  20,  50,  40,  40,  60, 100,0,0});
			queue.push({1500,  48,  15,   0,   0,   0,   0,   0,0,0});
			queue.push({1000,  50,  10,   0,   0,   0,  40,  40,0,0});
			break;

		default:
			Serial.println(F("Invalid animation requested"));
			break;
	}


}

#endif /* ANIMATIONS_INO */