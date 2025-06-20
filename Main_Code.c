
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
 - If error is 0, turn back the DC motor into standard PWM so it will move straight
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

//CLOCK
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608)) //RCGCGPIO Address offset 0x608
#define SYSCTL_RCGCADC_R        (*((volatile uint32_t *)0x400FE638)) //RCGCADC Address offset 0x638
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08)) //Peripheral Ready GPIO Address offset 0xA08
#define SYSCTL_PRADC_R          (*((volatile uint32_t *)0x400FEA38)) //Peripheral Ready ADC Address offset 0xA38

//GPIO A

#define GPIO_PORTA_BASE					(*((volatile uint32_t *)0x40004000)) //Base address for Port A
#define GPIO_PORTA_DATA_R       (*((volatile uint32_t *)0x400043FC)) //Offset 0x3fc
#define GPIO_PORTA_DIR_R        (*((volatile uint32_t *)0x40004400)) //Offset 0x400
#define GPIO_PORTA_PUR_R        (*((volatile uint32_t *)0x40004510)) //Offset 0x510
#define GPIO_PORTA_DEN_R        (*((volatile uint32_t *)0x4000451C)) //Offset 0x51c
#define GPIO_PORTA_DR8R					(*((volatile uint32_t *)0x40004508)) //Offset 0x508

//NVIC
#define NVIC_STCTRL_R						(*((volatile uint32_t *)0xE000E010)) //Offset 0x010
#define NVIC_STRELOAD_R					(*((volatile uint32_t *)0xE000E014)) //Offset 0x014
#define NVIC_STCURRENT_R				(*((volatile uint32_t *)0xE000E018)) //Offset 0x018
#define NVIC_SYS_PRI3_R  				(*((volatile uint32_t *)0xE000E40C)) //PRI3 because SysTick interrupt ( OFFSET 0X40C)
	
	
//GPIO E
#define GPIO_PORTE_BASE				 (*((volatile uint32_t *)0x40024000)) //Base address for Port E from page 104 Datasheet TM4C1294XL
#define GPIO_PORTE_DATA_R			 (*((volatile uint32_t *)0x400243FC)) //Offset 0x3fc
#define GPIO_PORTE_DIR_R			 (*((volatile uint32_t *)0x40024400)) //Offset 0x400
#define GPIO_PORTE_DEN_R			 (*((volatile uint32_t *)0x4002451C)) //Offset 0x51C
#define GPIO_PORTE_AMSEL_R		 (*((volatile uint32_t *)0x40024528)) //Offset 0x528
#define GPIO_PORTE_AFSEL_R		 (*((volatile uint32_t *)0x40024420)) //Offset 0x420

//ADC0
#define ADC0_BASE				 	     (*((volatile uint32_t *)0x40038000)) //Base address for ADC0 from page 1077 Datasheet TM4C1294XL
#define ADC0_ACTSS				 	   (*((volatile uint32_t *)0x40038000)) //Offset 0x000
#define ADC0_EMUX				 	     (*((volatile uint32_t *)0x40038014)) //Offset 0x014
#define ADC0_SSPRI				 	   (*((volatile uint32_t *)0x40038020)) //Offset 0x020
#define ADC0_SSMUX2				 	   (*((volatile uint32_t *)0x40038080)) //Offset 0x080
#define ADC0_SSCTL2				 	   (*((volatile uint32_t *)0x40038084)) //Offset 0x084
#define ADC0_IM				 	       (*((volatile uint32_t *)0x40038008)) //Offset 0x008
#define ADC0_PC				 	       (*((volatile uint32_t *)0x40038FC4)) //Offset 0xFC4
#define ADC0_PSSI			 	   		 (*((volatile uint32_t *)0x40038028)) //Offset 0x028
#define ADC0_RIS				 	   	 (*((volatile uint32_t *)0x40038004)) //Offset 0x004
#define ADC0_SSFIFO2				 	 (*((volatile uint32_t *)0x40038088)) //Offset 0x088
#define ADC0_ISC				 	     (*((volatile uint32_t *)0x4003800C)) //Offset 0x00C






//**FUNCTIONS**//
// Enable global interrupts
void EnableInterrupts(void) {
    __asm("CPSIE I");  // CPSIE I = Clear Interrupt Disable bit, enabling interrupts
}

// Wait for interrupt 
void WaitForInterrupts(void) {
    __asm("WFI");  // WFI = Wait For Interrupt instruction
}


void GPIOA_Init (void)
{
	SYSCTL_RCGCGPIO_R |= 0x01;  //0000 0001 , turn on Port A clock
	GPIO_PORTA_DIR_R |= 0x30; //0011 0000, PA4 and PA5 as output
	GPIO_PORTA_DEN_R |= 0x30; //0011 0000, PA4 and PA5 use digital funct
	GPIO_PORTA_DR8R |= 0x30; //0011 0000, PA4 and PA5 has 8mA drive ( for dc motor amps)
}

void GPIOE_Init (void)
{
	SYSCTL_RCGCGPIO_R |= 0x10; //0001 000"1", turn on port E clock yet not disturbing previous setup on Port A
	GPIO_PORTE_DEN_R &= ~0xFF; //0000 0000, make everything 0
	GPIO_PORTE_DIR_R &= ~0x0C; //1111 0011, make PE3 and PE2 input
	GPIO_PORTE_AFSEL_R |= 0x0C; // 0000 1100, PE3 and PE2 alternative funct on
	GPIO_PORTE_AMSEL_R |= 0x0C; //0000 1100, PE3 and PE2 analog funct on
	
}

void ADC0_Init_SoftwareTrigger (void)
{
	SYSCTL_RCGCADC_R |= 0x01; //0000 0001, turn on ADC0
	while((SYSCTL_PRGPIO_R & 0x0C) != 0x0c); // loop for stabilization, if false then proceed, if true then loop is still looping
	ADC0_PC &= ~0xF; //reset all bits to 0 for ADCPC reg
	ADC0_PC |= 0x01; //set to 128 Khz, why? cause delay 112 Tadc, with average 13 Tadc for conversion to happen, so if ADC clock 16 Mhz / 125 = 128 kHz
	ADC0_SSPRI |= 0x1023; // 1 for SS3, 0 for SS2, 2 for SS1, 3 for SS0. Smaller the number, higher the priority so SS2 is the highest
	ADC0_ACTSS &= ~0x04; //Disable Sample Sequencer 2 ( SS2)
	ADC0_EMUX &= ~0x0F00; //Software Trigger for seq 2
	ADC0_SSMUX2 = 0x01; //1 for AIN1 because PE2 at bit 0-3 at step 0, 0 for AIN0 because PE3 at bit 4-7 at step 1
	ADC0_SSCTL2 = 0x66; // IE0 and END0 for step 0 and IE1 and END1 for step 1
	ADC0_IM &= ~0x04; // disable interrupt mask for SS2
	ADC0_ACTSS |= 0x04; //enable sample sequencer 2 again
	
}

void SysTick_Init (void)
{
	NVIC_STCTRL_R = 0;
	NVIC_STRELOAD_R = 7200 -1; //Standard PWM is 45% from 16000 cycles per second or 1kHz from 16 mHz 
	NVIC_STCURRENT_R = 0;
	NVIC_STCTRL_R = 0x07;
}


