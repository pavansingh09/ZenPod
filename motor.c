#include "MKL25Z4.h"                    // Device header
#include "RTE_Components.h"             // Component selection
#include "system_MKL25Z4.h"             // Keil::Device:Startup
#include "math.h"

// PORT C Define
#define SERVO_SHIFT (0) // PORTC relay connected to Servo.
#define LED_BLUE (3) // PORTC LED is meditation is still active
#define LED_GREEN (7) // PORTC led if inhaling/exhaling

//define 4 LEDS on PORT A
#define LED1 (1) //PTA1
#define LED2 (2) //PTA2
#define LED3 (4) //PTA4
#define LED4 (5) //PTA5

// number of LEDS for med time
#define MED_LED_NUM (4) 
//#define 4 pin Switches on PORT D
#define SW_SWITCH (4) // starts the meditation
#define PLUS_SWITCH (5)
#define MINUS_SWITCH (0)

// Delays for state machine
#define HOLD_DELAY (2) // delay between all states
#define STATE_DELAY (1)

// State machine masks
#define inhaleMask (0xF0) // grab inhale time
#define exhaleMask (0x0F) // grab exhale time

// Misc timers
#define MED_TIME_DEFAULT (1200) // default number of seconds

#define MASK(x) (1ul << x)

// Prototypes
void initPins(void);
void initSysTick(void);
void breathStateMachine(void);
void handleLedTimes(void);
void handleHeatPWM(void);
void handleMeditationStatus(void);
void setMedOnOffSettings(short);
void setLedMask(short);

// Arrays for init
static short portCGpio[3] = {SERVO_SHIFT, LED_BLUE, LED_GREEN};
static short portAGpio[4] = {LED1, LED2, LED3, LED4}; // LED1 indicates lowest time remaining
static short portDGpio[3] = {MINUS_SWITCH, PLUS_SWITCH, SW_SWITCH};

// Global values for breathing
static short breathTime = 0; // timer for inhaling and exhaling
static short delayTime = 0; // value for delay between states
static short inMeditation = 0;
static short inhaleExhaleSettings[3] = {0x22, 0x32, 0x43}; // Inhale/Exhale timer
static short breathingSetting = 0; // default
static short prev_medState = 0; // Have we transistioned states?

// Globals for Meditation Time LEDS.
static uint16_t medTimeCurrent = MED_TIME_DEFAULT; // 20 minute default
static uint16_t medTimeMax = MED_TIME_DEFAULT; // max time 
// enums
// State of breathing
enum breathState
{
	noBreath =0,
	inhaling,
	holding,
	exhaling
};	

// Enum state globals
static short currentBreathState = noBreath; // init the breath state
static short prevBreathState = noBreath; // Used to determine if we've changed states

// Time remaining in meditation cycle
enum medTimeState
{
	off = 0,
	percent25,
	percent50,
	percent75,
	percent100
};

// Enum state globals
static short currentMedState; // init the breath state
static short prevMedState; // Used to determine if we've changed states

static short medLedMask = 0; // contains 4 bits that will determine the LEDs states


int main()
{
	initPins(); // init speaker port
	initSysTick(); // init pit timer
	while(1)
	{
		handleMeditationStatus();
		prev_medState = inMeditation;
	}
}

// Inits the pins for the board
void initPins()
{
	SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTA_MASK);
	
	// Configuring buttons
	for(int i = 0; i < sizeof(portDGpio)/(sizeof(short)); i++)
	{
		PORTD->PCR[MASK(portDGpio[i])] &= ~(PORT_PCR_MUX_MASK | PORT_PCR_IRQC_MASK);
		PORTD->PCR[MASK(portDGpio[i])] |= (PORT_PCR_MUX(1) | PORT_PCR_IRQC(10) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK); // gpio, check falling edge, pullup resistor set
		PTD->PDDR &= (uint8_t)~MASK(portDGpio[i]); // set pin direction as input
	}
	// configuring Servo related GPIOs
	for(int i = 0; i < sizeof(portCGpio)/(sizeof(short)); i++)
	{
		PORTC->PCR[portCGpio[i]] |= PORT_PCR_MUX(1);
		PTC->PDDR |= MASK(portCGpio[i]);
		PTC->PDOR |= MASK(portCGpio[i]);
		PTC->PCOR |= MASK(portCGpio[i]);
	}
	// configuring PORT A GPIO LEDS
	for(int i = 0; i < sizeof(portAGpio)/(sizeof(short)); i++)
	{
		PORTC->PCR[portAGpio[i]] |= PORT_PCR_MUX(1);
		PTC->PDDR |= MASK(portAGpio[i]);
		PTC->PDOR |= MASK(portAGpio[i]);
		PTC->PCOR |= MASK(portAGpio[i]);
	}
}

// Inits the Systick to keep track of internal clock
void initSysTick()
{
	SysTick->LOAD = (48000000L/3); // lower sysclock to 16Mhz to fit in Load
	NVIC_SetPriority(SysTick_IRQn, 3);
	NVIC_ClearPendingIRQ(SysTick_IRQn);
	NVIC_EnableIRQ(SysTick_IRQn);
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; // Enable interrupts and timer
}

// Handles meditation state
void handleMeditationStatus()
{
	if(inMeditation != prev_medState)
	{
		setMedOnOffSettings(inMeditation); 
	}
	if(PORTD->ISFR & MASK(SW_SWITCH))
		{
			// Toggle meditation
			if(inMeditation)
			{
				inMeditation = 0;
				setMedOnOffSettings(inMeditation);
			}
			else
			{
				inMeditation = 1;
				setMedOnOffSettings(inMeditation);
			}
			PORTD->ISFR &= 0xffffffff; // clear button flag
		}
}

// Sets the settings based on meditation on or off
void setMedOnOffSettings(short state)
{
	if(state == 0)
	{
		breathTime = 0;
		PTC->PCOR |= (MASK(SERVO_SHIFT) | MASK(LED_BLUE) | MASK(LED_GREEN));
	}
	else
	{
		PTC->PSOR |= MASK(LED_BLUE);
	}
}

// determines the states for the breathing statemachine
void breathStateMachine()
{
	switch(currentBreathState)
	{
		case noBreath:
			delayTime++;
			if(delayTime >= STATE_DELAY)
			{
				delayTime = 0;
				currentBreathState = inhaling;
			}
			break;
		case inhaling:
			breathTime++; // increment the seconds
			if(breathTime >= (exhaleMask & inhaleExhaleSettings[breathingSetting]))
			{
				currentBreathState = noBreath;
				breathTime = 0;
			}
			break;
		case holding:
			delayTime++;
			if(delayTime >= HOLD_DELAY)
			{
				delayTime = 0;
				currentBreathState = exhaling;
			}
			break;
		case exhaling:
			breathTime++; // increment the seconds
			if(breathTime >= (exhaleMask & inhaleExhaleSettings[breathingSetting]))
			{
				currentBreathState = noBreath;
				breathTime = 0;
			}
			break;
		default:
			currentBreathState = noBreath;
			delayTime = 0;
			break;
	}
	prevBreathState = currentBreathState;
	if((prevBreathState != currentBreathState) && (currentBreathState == inhaling || currentBreathState == exhaling))// do we turn on the servo motor? If we are inhaling or exhaling we do
	{
		PTC->PSOR |= MASK(SERVO_SHIFT); // toggle servo motor
		PTC->PSOR |= MASK(LED_GREEN);
	}
	else
	{
		PTC->PCOR |= MASK(SERVO_SHIFT); // when not inhaling or exhaling
		PTC->PCOR |= MASK(LED_GREEN);
	}
}

// Contains the logic for Meditation timed leds.

void handleLedTimes()
{
	// As per default, each LED will represent 100/numLeds percent
	// of total meditation time. 
	// OR 
	// MeditationTime(minutes)/numLEDS (TBD). This is be 20 minutes by default
	// Max time is 2 hours. 
	
	float currentMedRatio = (float)medTimeCurrent / (float)medTimeMax;
	
	currentMedState = ceil(currentMedRatio *  MED_LED_NUM); // if 0.01 round to 1
	
	// LEVELS
	/*
	0.01 -> 0.25 1 LED
	0.26 -> 0.50 2 LED
	0.51 -> 0.75 3 LED
	0.76 -> 1.00 4 LED	
	*/
	// We will know the new current state prior to switch statement
	if(prevMedState != currentMedState)
	{
		switch(currentMedState)
		{
			case off:
				medLedMask = 0;
			break;
			case percent25:
				medLedMask = 0x8;
			break;
			case percent50:
				medLedMask = 0xC;
			break;
			case percent75:
				medLedMask = 0xE;
			break;
			case percent100:
				medLedMask = 0xF;
			break;
			default:
				medLedMask = 0xF;
				// full
			break;
		}
		
		setLedMask(medLedMask);
		// update the LEDS
	}
	else
	{
		// Do nothing, LEDs are in correct state
	}
	prevMedState = currentMedState; // Save the previous state.
}

void setLedMask(short mask)
{
	for(int i = 0; i < MED_LED_NUM; i++)
	{
		if(1 & mask)
		{			
			PTA->PSOR |= MASK(portAGpio[(MED_LED_NUM - 1) - i]); // if I = 0, select 3
		}
		else
		{
			PTA->PCOR |= MASK(portAGpio[(MED_LED_NUM - 1) - i]);
		}
		mask = mask >> 1UL; // shift the mask
	}
}

void SysTick_Handler()
{
	// if we are in an intermmediat state, we delay breathTime
	if(inMeditation)
	{
		medTimeCurrent--;
		if(medTimeCurrent == 0)
		{
			inMeditation = 0;
		}
		breathStateMachine();
	}
	handleLedTimes();
}
