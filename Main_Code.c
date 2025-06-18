
//*** GENERAL VIEW OF THE PROJECT ***//

/*
Create a simple robot that can handle obstacle and keep moving forward. Using PWM to control the DC Motor based on the data received from the distance sensors attached at the front of the robot.
*/

//*** SOFTWARE DESIGN ** //

/*

1. Set GPIO as the output to control 2 DC Motors ( 4-24 V ) (PA4 and PA5)

2. Set GPIO as the input from the distance sensors (Sharp GP2Y0A21YK0F) (Right now, PE3 and PE2 for AIN0 and AIN1)

3. Set the PWM function using Systick to automate and control the DC motors PWM
 - Using Systick to control the PWM at right and left DC motors
 - Minimum PWM is 30% on each DC Motor ( High 30%, Low 70%), and Maximum PWM is 60% on each the DC motor, standard is 45%
 - We set the NVIC Reload value for the H+L is 16000, because we need 1000 cycle per second or 1 KHz to make the rotation smoother
 - We will get the analog value from the distance sensor and convert it to digital value in another function, based on that statement, we will control the PWM if it should turn right or left or stay
 - We will use this main formula to determine the 'turn', Error = DistanceLeft - DistanceRight

4. Create the 2 ADC Function to convert the analog value to digital value so we can use that to determine the robot's judgment
 - Based on the empirical calibration, we found that contant number for ADC value to Real distance is 241814, so the formula is Distance = 241814 / ADC value from sensor
 - Because we are using TM4C1294XL, we use 12 bit ADC so the ADC value range will be from 0 to 4095 ( cause of 12 bit, therefore 4096 possibilities can happen)
 - After conversion, we will return the Distance value so it can be used to control each the DC motor PWM

5. After distance value is returned by the ADC function, make Robot_Logic function
 - Based on the previous point (3), based on the error, if error is 0 then there is no need to turn right/left ( keep going straight )
 - Lets say if error is 10 , because left is bigger than right (+) ( probably because right sensor is closer to the wall/obstacle of course) the Right DC motor PWM will be decreased and left Dc motor PWM will be increased 
 - IF error is 0, turn back the DC motor into standard PWM so it will move straight
 - Lets say if error is -10 , right is bigger than left (-), the left DC motor PWM will be decreased and Right DC motor PWM will be increased
 - The problem is if the track is relatively safe to move straight but there are 2 walls with different distance from the sensor, we need to find the limitation for the robot logic to work smoothly without bug

6. Polish the software as needed

*/

//*** HARDWARE DESIGN AND BOM ***//

/*

1. PCB will be designed in KiCad and printed to subtitute breadboards and jumpers

2. DC Motor Circuit
	- 2 DC Motors 3-12V Model F130( actuators )
	- 2 Distance Sensors Sharp GP2Y0A21YK0F ( sensors )
	- 3 LiOn Battery 3.71 V ( power source ) and one set of 3-battery slot
	- 2 Capacitors 10 micro farad ( Controlling noise from input and output )
	- 1 regulator L7805CV ( Regulating voltage from 11.13 v source to Vin MCU and sensors )
	- 2 Transistors MOSFET IRLB3034 ( Logic control from MCU to Motors )
	- 2 Diodes LN4002 ( Protect MCU )
	- 2 220 Ohm resistors ( protect GPIO to gate Transistor)
	- 2 10k Ohm resistors ( pull down so it will stabilize motor while MCU turns on )
	
3. Robot Body and Attachment
	- 2 wheels attached to the DC Motors 78MM Dual Shaft( choose with the good friction and low load for the motor to spin)
	- 2 WD Casis kit
	- To be continued

*/




#include <stdint.h>



//*** ADDRESS ***//








