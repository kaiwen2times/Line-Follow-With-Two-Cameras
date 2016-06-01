/*
* Freescale Cup linescan camera code
*
*	This method of capturing data from the line
*	scan cameras uses a flex timer module, periodic
*	interrupt timer, an ADC, and some GPIOs.
*	CLK and SI are driven with GPIO because the FTM2
*	module used doesn't have any output pins on the
* 	development board. The PIT timer is used to 
*  control the integration period. When it overflows
* 	it enables interrupts from the FTM2 module and then
*	the FTM2 and ADC are active for 128 clock cycles to
*	generate the camera signals and read the camera 
*  output.
*
*	PTB8			- camera CLK
*	PTB23 		- camera SI
*  ADC0_DP1 	- camera AOut
*
* Author:  Alex Avery
* Created:  11/20/15
* Modified:  11/23/15
*/

#include "MK64F12.h"
#include "uart.h"
#include "stdio.h"
#include <math.h>
#include "pwm.h"

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
//	(camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME .0075f

void init_FTM2(void);
void init_GPIO(void);
void init_PIT(void);
void init_ADC0(void);
void init_ADC1(void);
void FTM2_IRQHandler(void);
void PIT0_IRQHandler(void);
void ADC0_IRQHandler(void);
void ADC1_IRQHandler(void);
void motor1Write(unsigned int DutyCycle, int dir); // left motor
void motor2Write(unsigned int DutyCycle, int dir); // right motor
void servoWrite(float DutyCycle); // 6 right turn, 9 left turn
void InitPWM(void);
void filterData(void);
void servoCap(void);
void indexCap(void);
void motorCap(int speed);
int abs(int num);

// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
//		ADC reads start
int pixcnt = -2;
// clkval toggles with each FTM interrupt
int clkval = 0;
// line stores the current array of camera data
int wideCamera[128];
int narrowCamera[128];
int diff[128];

// sampling frequency, the lower the faster
int sample = 1;
int lcv;
int dir = 0;

// These variables are for streaming the camera
//	 data over UART
int capcnt = 0;
char str[100];

// ADC0VAL holds the current ADC value
int ADC0VAL;
int ADC1VAL;

// motor logic variable
float servoRightCap = 9.0;
float servoLeftCap = 6.0;
float kpServo = 0.07;

//float kd = 5;
float servoSpeed;
int motor1Speed;
int motor2Speed;
int maxMotorSpeed = 100 ;
int minMotorSpeed = 0;
int idealIndex = 60;
int idealMotorSpeed = 50;
float kpMotor = 1.2;
float brake = 0.7;
int vtec = 15;

// error variables
int error;
int lastError;
int windowSize = 20;

// diff related variables
int currentIndex;
int maxIndex;
int maxIndexCap = 120;
int minIndex;
int minIndexCap = 17;
int max;
int min;
int leftMin;
int rightMin;
int threshold = 25000;

// debug flag
int debug = 0;

int main()
{
	if(debug)
	{
		sample = 100;
	}
	
	uart_init();
	init_GPIO(); // For CLK and SI output on GPIO
	init_FTM2(); // To generate CLK, SI, and trigger ADC
	init_ADC0();
	init_ADC1();
	init_PIT();	// To trigger camera read based on integration time
	InitPWM();

	for(;;)
	{
		if (capcnt >= sample)
		{
			// send the array over uart
			if(0)
			{
				GPIOB_PCOR |= (1 << 22);
				sprintf(str,"%i\n\r",-1); // start value
				put(str);
				for (lcv = 0; lcv < 127; lcv++)
				{
					sprintf(str,"%i\n\r", narrowCamera[lcv]);
					put(str);
				}
				sprintf(str,"%i\n\r",-2); // end value
				put(str);
				GPIOB_PSOR |= (1 << 22);
			}

			// current line calculation
			max = 0;
			min = 0;
			// calculate diff
			for(lcv=0; lcv<128; lcv++)
			{
				diff[lcv] = wideCamera[lcv+1] - wideCamera[lcv];
			}

			for (lcv=3; lcv<124; lcv++)
			{
				if (diff[lcv] > max)
				{
					max = diff[lcv];
					maxIndex = lcv;
				}
				else if (diff[lcv] < min)
				{
					min = diff[lcv];
					minIndex = lcv;
				}
			}
			currentIndex = (int)(maxIndex + minIndex) / 2;
			indexCap();
			// end

			// PID algorithm
			error = currentIndex - idealIndex;
			lastError = (error <= 3 && error >= -3 ) ? -vtec : abs(error) * brake;
			servoSpeed = (float)7.5 - error * kpServo;
			motor1Speed = idealMotorSpeed - kpMotor * error - lastError;
			motor2Speed = idealMotorSpeed + kpMotor * error - lastError;
			
			servoCap();
			motor1Speed = (motor1Speed > maxMotorSpeed) ? maxMotorSpeed : motor1Speed;
			motor1Speed = (motor1Speed < minMotorSpeed) ? minMotorSpeed : motor1Speed;
			motor2Speed = (motor2Speed > maxMotorSpeed) ? maxMotorSpeed : motor2Speed;
			motor2Speed = (motor2Speed < minMotorSpeed) ? minMotorSpeed : motor2Speed;
			
			
			// stop at the finish line
			leftMin = rightMin = narrowCamera[idealIndex];
	
			for(lcv=idealIndex; lcv>idealIndex-windowSize; lcv--)
			{
				if (narrowCamera[lcv] < leftMin)
				{
					leftMin = narrowCamera[lcv];
				}
			}
			
			for(lcv=idealIndex; lcv<idealIndex+windowSize; lcv++)
			{
				if (narrowCamera[lcv] < rightMin)
				{
					rightMin = narrowCamera[lcv];
				}
			}
			
			if(leftMin < threshold && rightMin < threshold)
			{
				motor1Write(0, dir);
				motor2Write(0, dir);
				servoWrite(7.5);
				return 0;
			}

			// debug code
			if(debug)
			{
				sprintf(str,"error: %i\n\r", error);
				put(str);
				sprintf(str,"lastError: %i\n\r", lastError);
				put(str);
				sprintf(str,"leftMin: %i\n\r", leftMin);
				put(str);
				sprintf(str,"rightMin: %i\n\r", rightMin);
				put(str);
				put("\n\r");
			}

			motor1Write(motor1Speed, dir);
			motor2Write(motor2Speed, dir);
			servoWrite(servoSpeed);
	
			capcnt = 0;
		}
	} //for
} //main

void servoCap(void)
{
	if(servoSpeed > servoRightCap)
	{
		servoSpeed = servoRightCap;
	}
	else if(servoSpeed < servoLeftCap)
	{
		servoSpeed = servoLeftCap;
	}
}

void indexCap(void)
{
	if(currentIndex > maxIndexCap)
	{
		currentIndex = maxIndexCap;
	}
	else if(currentIndex < minIndexCap)
	{
		currentIndex = minIndexCap;
	}
}

int abs(int num)
{
	if(num > 0)
	{
		return num;
	}
	else
	{
		return num * (-1);
	}
}

/* ADC0 Conversion Complete ISR  */
void ADC0_IRQHandler(void)
{
	// Reading ADC0_RA clears the conversion complete flag
	ADC0VAL = ADC0_RA;
}

/* ADC1 Conversion Complete ISR  */
void ADC1_IRQHandler(void)
{
	// Reading ADC0_RA clears the conversion complete flag
	ADC1VAL = ADC1_RA;
}

/* 
* FTM2 handles the camera driving logic
*	This ISR gets called once every integration period
*		by the periodic interrupt timer 0 (PIT0)
*	When it is triggered it gives the SI pulse,
*		toggles clk for 128 cycles, and stores the line
*		data from the ADC into the line variable
*/
void FTM2_IRQHandler(void)
{ 	//For FTM timer
	// Clear interrupt
  	FTM2_SC &= ~FTM_SC_TOF_MASK;
	
	// Toggle clk
	clkval = !clkval;
	GPIOB_PTOR = (1<<9);
	// Line capture logic
	if ((pixcnt >= 2) && (pixcnt < 256))
	{
		if (!clkval)
		{	// check for falling edge
			// ADC read (note that integer division is 
			//  occurring here for indexing the array)
			wideCamera[pixcnt/2] = ADC0VAL;
			narrowCamera[pixcnt/2] = ADC1VAL;
		}
		pixcnt += 1;
	} 
	else if (pixcnt < 2)
	{
		if (pixcnt == -1)
		{
			GPIOB_PSOR |= (1 << 23); // SI = 1
		} 
		else if (pixcnt == 1) 
		{
			GPIOB_PCOR |= (1 << 23); // SI = 0
			// ADC read
			wideCamera[0] = ADC0VAL;
			narrowCamera[0] = ADC1VAL;
		} 
		pixcnt += 1;
	} 
	else 
	{
		GPIOB_PCOR |= (1 << 9); // CLK = 0
		clkval = 0; // make sure clock variable = 0
		pixcnt = -2; // reset counter
		// Disable FTM2 interrupts (until PIT0 overflows
		//   again and triggers another line capture)
		FTM2_SC &= ~(FTM_SC_TOIE_MASK);
	}
}

/* PIT0 determines the integration period
*		When it overflows, it triggers the clock logic from
*		FTM2. Note the requirement to set the MOD register
* 	to reset the FTM counter because the FTM counter is 
*		always counting, I am just enabling/disabling FTM2 
*		interrupts to control when the line capture occurs
*/
void PIT0_IRQHandler(void)
{
	capcnt += 1;
	// Clear interrupt
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	
	// Setting mod resets the FTM counter
	FTM2->MOD = 200;
	
	// Enable FTM2 interrupts (camera)
	FTM2_SC |= FTM_SC_TOIE_MASK;
}


/* Initialization of FTM2 for camera */
void init_FTM2()
{
	// Enable clock
	SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

	// Disable Write Protection
	FTM2_MODE |= FTM_MODE_WPDIS_MASK;
	
	// Set output to '1' on init
	//FTM2_OUTINIT |= FTM_OUTINIT_CH2OI_MASK;
	FTM2_OUTINIT |= FTM_OUTINIT_CH0OI_MASK;
	
	// Initialize the CNT to 0 before writing to MOD
	FTM2_CNT = 0;
	
	// Set the Counter Initial Value to 0
	FTM2_CNTIN = 0;
	
	// Set the period (~10us)
	FTM2->MOD = 200;
	
	// 50% duty
	//FTM2->MOD = 100; FTM2_C0V
	FTM2_C0V = 100;
	// Set edge-aligned mode
	FTM2_C0SC |= FTM_CnSC_MSB_MASK;
	FTM2_C0SC |= FTM_CnSC_MSA_MASK;
	
	// Enable High-true pulses
	// ELSB = 1, ELSA = 0
	FTM2_C0SC |= FTM_CnSC_ELSB_MASK;
	FTM2_C0SC &= ~FTM_CnSC_ELSA_MASK;
	
	// Enable hardware trigger from FTM2
	FTM2_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;
	
	// Don't enable interrupts yet (disable)
	FTM2_SC &= ~FTM_SC_TOIE_MASK;
	
	// No prescalar, system clock
	FTM2_SC |= FTM_SC_PS(0x00);
	FTM2_SC |= FTM_SC_CLKS(0x01);
	
	// Set up interrupt
	//FTM2_SC |= FTM_SC_TOIE_MASK;
	
	NVIC_EnableIRQ(FTM2_IRQn);
}

/* Initialization of PIT timer to control 
* 		integration period
*/
void init_PIT(void)
{
	// Setup periodic interrupt timer (PIT)
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

	// Enable clock for timers
	//PIT_MCR &= ~PIT_MCR_MDIS_MASK;

	// Enable timers to continue in debug mode
	PIT_MCR = PIT_MCR_FRZ_MASK; // In case you need to debug
	
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	//PIT_LDVAL0 = DEFAULT_SYSTEM_CLOCK;
	PIT_LDVAL0 = (uint32_t)(INTEGRATION_TIME * DEFAULT_SYSTEM_CLOCK);
	
	// Enable timer interrupts
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
	
	// Enable the timer
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
 
	// Clear interrupt flag
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT0_IRQn);
}


/* Set up pins for GPIO
* 	PTB9 		- camera clk
*	PTB23		- camera SI
*	PTB22		- red LED
*/
void init_GPIO(void)
{
	// Enable LED and GPIO so we can see results
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

	// Configure the Signal Multiplexer for GPIO
    PORTB_PCR22 = PORT_PCR_MUX(1);
    PORTB_PCR23 = PORT_PCR_MUX(1);
    PORTB_PCR9 = PORT_PCR_MUX(1);

	// Switch the GPIO pins to output mode
	GPIOB_PDDR = (1<<22) | (1<<23) | (1<<9);
	
	// Turn off the LED
	GPIOB_PDOR = (1<<22) | (1<<23) | (1<<9);
}

/* Set up ADC for capturing camera data */
void init_ADC0(void)
{
    unsigned int calib;
    // Turn on ADC0
    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	
	// Single ended 16 bit conversion, no clock divider
	ADC0_CFG1 |= ADC_CFG1_ADIV(0x00);
    ADC0_CFG1 |= ADC_CFG1_MODE(0x03);
    
    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC0_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
    calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
    calib = calib >> 1; calib |= 0x8000;
    ADC0_PG = calib;
    
    // Select hardware trigger.
    ADC0_SC2 |= ADC_SC2_ADTRG_MASK;
    
    // Set to single ended mode	
	ADC0_SC1A = 0;
    ADC0_SC1A |= ADC_SC1_AIEN_MASK;
    ADC0_SC1A &= ~ADC_SC1_DIFF_MASK;
    ADC0_SC1A |= ADC_SC1_ADCH(0x01);
	
	// Set up FTM2 trigger on ADC0
	SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0x0A); // FTM2 select
	SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK; // Alternative trigger en.
	SIM_SOPT7 &= ~SIM_SOPT7_ADC1PRETRGSEL_MASK; // Pretrigger A
	
	// Enable NVIC interrupt
    NVIC_EnableIRQ(ADC0_IRQn);
}


/* Set up ADC for capturing camera data */
void init_ADC1(void)
{
    unsigned int calib;
    // Turn on ADC1
    SIM_SCGC3 |= SIM_SCGC3_ADC1_MASK;
	
	// Single ended 16 bit conversion, no clock divider
	ADC1_CFG1 |= ADC_CFG1_ADIV(0x00);
    ADC1_CFG1 |= ADC_CFG1_MODE(0x03);
    
    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC1_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC1_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC1_CLP0; calib += ADC1_CLP1; calib += ADC1_CLP2;
    calib += ADC1_CLP3; calib += ADC1_CLP4; calib += ADC1_CLPS;
    calib = calib >> 1; calib |= 0x8000;
    ADC1_PG = calib;
    
    // Select hardware trigger.
    ADC1_SC2 |= ADC_SC2_ADTRG_MASK;
    
    // Set to single ended mode	
	ADC1_SC1A = 0;
    ADC1_SC1A |= ADC_SC1_AIEN_MASK;
    ADC1_SC1A &= ~ADC_SC1_DIFF_MASK;
    ADC1_SC1A |= ADC_SC1_ADCH(0x01);
	
	// Set up FTM2 trigger on ADC1
	SIM_SOPT7 |= SIM_SOPT7_ADC1TRGSEL(0x0A); // FTM2 select
	SIM_SOPT7 |= SIM_SOPT7_ADC1ALTTRGEN_MASK; // Alternative trigger en.
	SIM_SOPT7 &= ~SIM_SOPT7_ADC1PRETRGSEL_MASK; // Pretrigger A
	
	// Enable NVIC interrupt
    NVIC_EnableIRQ(ADC1_IRQn);
}
