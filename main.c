#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>  //Exact-width integer types
#include <ti\devices\msp432p4xx\driverlib\driverlib.h>  // Driver library
#include <motor.h>
#include <tachometer.h>
#include <bumpSensors.h>
#include <uart.h>

#define CLOCK_HF		48000000 	// Hz
#define CLOCK_LF		32000 		// Hz
#define SAMPLE_FREQ		100			// Hz. ADC can't take more than 24 MHz per sample. (P6.0 and P6.1 are each 1 sample). 50 Hz per sample is enough I think

#define HEARTBEAT_FREQ	2  			// unit: Hz, heart beat LED blinking frequency for debugging

#define RED_LED 		GPIO_PIN0  // heart beat LED
#define GREEN_LED		GPIO_PIN1
#define BLUE_LED		GPIO_PIN2

#define LEFT_MOTOR		0
#define RIGHT_MOTOR		1
#define MOTOR_FORWARD	1
#define MOTOR_STOP		0
#define MOTOR_BACKWARD	-1

void initDevice_HFXT();
void initHeartBeatLED();
void initTimer();
void initADC14();

uint32_t clockMCLK, clockSMCLK, clockACLK;
uint8_t currentLED = RED_LED;
int backingUp = 0;

const char *terminalDisplayText = 	"\r\nTI-RSLK MAX Motor Control Demo\r\n"
									"  C: Change LED Color, S: Change Direction, I: Increase DutyCycle,\r\n"
									"  D: Decrease DutyCycle, V: Display Speed, H: Help\r\n"
									"> ";

void main(void)
{
	char str[ 100 ];
	uint8_t data;

	initDevice_HFXT();
	initHeartBeatLED();
	initUART();
	initTimer();
	initADC14();
	initMotors( clockSMCLK );
	initBumpSensors( clockMCLK );
	initTachometers( clockSMCLK );

	Interrupt_enableMaster();
	Timer32_startTimer( TIMER32_0_BASE, false );

	// Start timer for tachometer speed measurements
	startTacho();

	//Initial display on terminal.
	uart0_transmitStr( terminalDisplayText );

	// Start duty cycle monitoring
	Timer32_startTimer( TIMER32_1_BASE, false );

	while(1)
	{
		if(UART_getInterruptStatus( EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG ) )
		{
			data = UART_receiveData( EUSCI_A0_BASE);
			UART_clearInterruptFlag( EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG );

			switch(data)
			{
				case 'C':
				case 'c':
					currentLED = (currentLED == BLUE_LED) ? RED_LED : (currentLED << 1); //cycle through three colors
					uart0_transmitStr("Change blinking LED color.\r\n> ");
					break;

				case 'H':
				case 'h':
					uart0_transmitStr( terminalDisplayText );
					break;

				case 'S':
				case 's':
					switchDirection( LEFT_MOTOR );
					switchDirection( RIGHT_MOTOR );

					sprintf( str, "Change motor direction: %i, %i.\r\n> ", getMotorDirection( LEFT_MOTOR ), getMotorDirection( RIGHT_MOTOR ) );
					uart0_transmitStr( str );
					break;

				case 'I':
				case 'i':
					increasePWM( LEFT_MOTOR );
					increasePWM( RIGHT_MOTOR );

					sprintf( str, "Increase PWM duty cycle: %i, %i.\r\n> ", getMotorDutyCycle( LEFT_MOTOR ), getMotorDutyCycle( RIGHT_MOTOR ) );
					uart0_transmitStr( str );
					break;

				case 'D':
				case 'd':
					decreasePWM( LEFT_MOTOR );
					decreasePWM( RIGHT_MOTOR );

					sprintf( str, "Decrease PWM duty cycle: %i, %i.\r\n> ", getMotorDutyCycle( LEFT_MOTOR ), getMotorDutyCycle( RIGHT_MOTOR ) );
					uart0_transmitStr( str );
					break;
				case 'V':
				case 'v':
					sprintf( str, "Left RPS = %.1f\r\nLeft Direction = %d\r\nRight RPS = %.1f\r\nRight Direction = %d\r\n\n",
							 getSpeed( LEFT_MOTOR ) / 60, getTachoDirection( LEFT_MOTOR ),
							 getSpeed( RIGHT_MOTOR ) / 60, getTachoDirection( RIGHT_MOTOR ) );
					uart0_transmitStr(str);
					break;
			} //end of switch
		} //end of if

		// True when a bump button is pressed.
		if( bumpStateSet() )
		{
			// Check if bump button is still pressed
			int bumpState = checkBumpState();
			if( bumpState == 0 )
			{
				uart0_transmitStr( "Button released.\r\n\n" );

				// Restart motors
				startMotor( LEFT_MOTOR );
				startMotor( RIGHT_MOTOR );


				// Turn away from wall example
				// Save previous duty cycles
//				int tempLeft = getMotorDutyCycle( LEFT_MOTOR );
//				int tempRight = getMotorDutyCycle( RIGHT_MOTOR );
//
//				// Turn to the left by stopping the left motor and turning on the right motor
//				setMotorDutyCycle( LEFT_MOTOR, 0 );
//				setMotorDutyCycle( RIGHT_MOTOR, 30 );
//
//				// Wait a bit as it turns away from the wall.
//				// This should be done with a timer with period set relative to the tachometer reading.
//				int i = 0;
//				for( i = 0; i < 1000000; i++ );
//
//				// Set motors back to previous state
//				setMotorDutyCycle( LEFT_MOTOR, tempLeft );
//				setMotorDutyCycle( RIGHT_MOTOR, tempRight );

			}
			else
			{
				// Stop motors while bump sensor is active
//				pauseMotor( LEFT_MOTOR );
//				pauseMotor( RIGHT_MOTOR );



				// Move away from wall
				backingUp = 1;
				setMotorDirection( LEFT_MOTOR, MOTOR_BACKWARD );
				setMotorDirection( RIGHT_MOTOR, MOTOR_BACKWARD );

				// Wait just a second while it moves away from the wall
				int i = 0;
				for( i = 0; i < 1000000; i++ );

				// Reset motor direction
				setMotorDirection( LEFT_MOTOR, MOTOR_FORWARD );
				setMotorDirection( RIGHT_MOTOR, MOTOR_FORWARD );
				backingUp = 0;

				// Pause motors until button is released.
				// In practical application, a turn could be added after the motor is restarted
				pauseMotor( LEFT_MOTOR );
				pauseMotor( RIGHT_MOTOR );

			}
		}
	} //end of while
}

void initDevice_HFXT()
{
	WDT_A_holdTimer();

	PCM_setPowerState( PCM_AM_DCDC_VCORE1 );
	FlashCtl_setWaitState( FLASH_BANK0, 1 );
	FlashCtl_setWaitState( FLASH_BANK1, 1 );

	FPU_enableModule();
	FPU_enableLazyStacking();

	GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_PJ, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION );
	CS_setExternalClockSourceFrequency( CLOCK_LF, CLOCK_HF );
	CS_startHFXT(false);

	CS_initClockSignal( CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1 );
	CS_initClockSignal( CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_8 );
	CS_initClockSignal( CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_16 );

	clockMCLK = CS_getMCLK();
	clockSMCLK = CS_getSMCLK();
}

void initADC14()
{
    ADC14_enableModule();
    ADC14_initModule( ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, ADC_NOROUTE );

	//Configure P6.1 as A14 and P6.0 as A15
	GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION );

	// Configure non-repeating multi-sequence memory storage in registers 0 and 1
	ADC14_configureMultiSequenceMode( ADC_MEM0, ADC_MEM1, false );

	// Store P6.1 (A14) reading in MEM0 and P6.0 (A15) reading in MEM1
	ADC14_configureConversionMemory( ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, false );
	ADC14_configureConversionMemory( ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A15, false );

    //See TechRef Section 20.2.6 for sample timing consideration.
    ADC14_enableSampleTimer( ADC_MANUAL_ITERATION );
    ADC14_setSampleHoldTime( ADC_PULSE_WIDTH_4, ADC_PULSE_WIDTH_4 );

    ADC14_enableConversion();
}

void initHeartBeatLED()
{
	GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 );

	Timer32_initModule( TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE );
	Timer32_setCount( TIMER32_0_BASE, clockMCLK / HEARTBEAT_FREQ );
	Timer32_enableInterrupt( TIMER32_0_BASE );
	Interrupt_enableInterrupt( INT_T32_INT1 );
}

void initTimer()
{
    Timer32_initModule( TIMER32_1_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE );
    Timer32_setCount( TIMER32_1_BASE, clockMCLK / SAMPLE_FREQ );
    Timer32_enableInterrupt( TIMER32_1_BASE );
    Interrupt_enableInterrupt( INT_T32_INT2 );
}

void T32_INT1_IRQHandler()
{
	Timer32_clearInterruptFlag( TIMER32_0_BASE );

	if(GPIO_getInputPinValue( GPIO_PORT_P2, RED_LED | GREEN_LED | BLUE_LED ) )
		GPIO_setOutputLowOnPin( GPIO_PORT_P2, RED_LED | GREEN_LED | BLUE_LED );
	else
		GPIO_setOutputHighOnPin( GPIO_PORT_P2, currentLED );
}

void T32_INT2_IRQHandler()
{
	Timer32_clearInterruptFlag( TIMER32_1_BASE );

	uint16_t dataInt[ 2 ] = { 0 };
	float dutyCycle, steering;

	//Read from ADC.
	ADC14_toggleConversionTrigger(); //ADC14SC is reset automatically

	// Get data from ADC registers
	ADC14_getMultiSequenceResult( dataInt );


	//dutyCycle = 200 * ( float ) dataInt[ 0 ] / 0x3FFF - 100;	// Calculate on range of [ -100, 100 ]. <0 means backwards, >0 means forwards
	dutyCycle = 100 * ( float ) dataInt[ 0 ] / 0x3FFF;
	steering = 100 * ( float ) dataInt[ 1 ] / 0x3FFF;


	// TODO: Currently this is impossible to be exacly 100:100. Fix this so that it's easier. Maybe a buffer range
	int rightDC = 100 * ( ( int ) steering >= 50 ? 1 : ( steering / ( 100 - steering ) ) );
	int leftDC = 100 * ( ( int ) steering <= 50 ? 1 : ( ( 100 - steering ) / steering ) );

	// Used to read input and output data for the RF modules
//	uint8_t dataIn = GPIO_getInputPinValue( GPIO_PORT_P4, GPIO_PIN2 );
//  uint8_t dataOut = GPIO_getInputPinValue( GPIO_PORT_P4, GPIO_PIN0 );

	char str[ 100 ];

	// This is used to test the RF data transmission. P6.1 is the voltage reading and P4.0 is the output pin
//	snprintf( str, 100, "P6.1 Data In = %0.3f; P4.0 Data Out = %i   \r", data, dataOut );

	// Used to show the duty cycle and steering ratio using potentiometers
	snprintf( str, 100, "Duty Cycle = %i%%; Duty Cycle Ratio (L:R): %i%%: %i:%i    \r", ( int ) dutyCycle, ( int )steering, leftDC, rightDC );
	uart0_transmitStr( str );

	setMotorDutyCycle( LEFT_MOTOR, ( int ) dutyCycle );
	setMotorDutyCycle( RIGHT_MOTOR, ( int ) dutyCycle );

	// TODO: setMotorDirection based on sign of dutyCycle when using the first equation
	// TODO: Implement steering using leftDC and rightDC
}

