#include "MKL25Z4.h"                    // Device header
#include "RTE_Components.h"             // Component selection
#include "system_MKL25Z4.h"             // Keil::Device:Startup
#define LEDHeater (1)			//Simulating Heater as LED on D PIN 0, This requires TPM0 and Channel 0 "Pavan"
#define PWM_PERIOD (48000)		//Heater PWM "Pavan"
#include "math.h"

// PORT C Define
#define SERVO_SHIFT (0) // PORTC relay connected to Servo.
#define LED_MED (3) // PORTC LED is meditation is still active
#define LED_SERVO (7) // PORTC led if inhaling/exhaling

//define 4 LEDS on PORT A
#define LED1 (5) //PTA1
#define LED2 (12) //PTA2
#define LED3 (2) //PTA4
#define LED4 (1) //PTA5

// number of LEDS for med time
#define MED_LED_NUM (4) 
//#define 4 pin Switches on PORT D
#define SW_SWITCH (4) // starts the meditation
#define PLUS_SWITCH (0)
#define MINUS_SWITCH (5)

// Delays for state machine
#define HOLD_DELAY (1) // delay between all states
#define STATE_DELAY (0)

// State machine masks
#define inhaleMask (0x00F0) // grab inhale time
#define exhaleMask (0x000F) // grab exhale time

// Misc timers
#define MED_TIME_DEFAULT (60) // default number of seconds

#define MASK(x) (1ul << x)

// Prototypes
void initPins(void);
void initSysTick(void);
void breathStateMachine(void);
void handleLedTimes(short); // timer, forced plus? forced minus?
void handleHeatPWM(void);	//May not need this one anymore
void handleMeditationStatus(void);
void setMedOnOffSettings(short);
void setLedMask(short);
void handleSwitches(void);
void initHeater (uint16_t);	//"Pavan"
void setHeaterPWMDutyCycle();	//"Pavan"

// Arrays for init
static short portCGpio[2] = {SERVO_SHIFT, LED_MED};
static short portAGpio[4] = {LED1, LED2, LED3, LED4}; // LED1 indicates lowest time remaining
static short portDGpio[3] = {MINUS_SWITCH, PLUS_SWITCH, SW_SWITCH};

// Global values for breathing
static short breathTime = 0; // timer for inhaling and exhaling
static short delayTime = 0; // value for delay between states
static short inMeditation = 0;
static short inhaleExhaleSettings[3] = {0x22, 0x32, 0x43}; // Inhale/Exhale timer
static short breathingSetting = 0; // default
static short prev_medState = 0; // Have we transistioned states?
static short InhaleExhaleSetting = 0;	// setting for inhale/exhale  "Pavan"

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

enum medTransitionType
{
	_timer,
	_plus,
	_minus
};

// Enum state globals
static short currentMedState = percent100; // init the breath state
static short prevMedLedState = off; // Used to determine if we've changed states

static short medLedMask = 0; // contains 4 bits that will determine the LEDs states


int main()
{
	initPins(); // init speaker port
	initSysTick(); // init pit timer
	initHeater (PWM_PERIOD);	//Initliaze the heater "Pavan"
	while(1)
	{
		handleSwitches();
		handleMeditationStatus();
		setHeaterPWMDutyCycle();	//"Pavan"
	}
}

// Inits the pins for the board
void initPins()
{
	SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTA_MASK);
	
	// Configuring buttons
	for(int i = 0; i < sizeof(portDGpio)/(sizeof(short)); i++)
	{
		PORTD->PCR[portDGpio[i]] &= ~(PORT_PCR_MUX_MASK | PORT_PCR_IRQC_MASK);
		PORTD->PCR[portDGpio[i]] |= (PORT_PCR_MUX(1) | PORT_PCR_IRQC(10) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK); // gpio, check falling edge, pullup resistor set
		PTD->PDDR &= (uint8_t)~MASK(portDGpio[i]); // set pin direction as input
	}
	// configuring Servo related GPIOs
	for(int i = 0; i < sizeof(portCGpio)/(sizeof(short)); i++)
	{
		PORTC->PCR[portCGpio[i]] |= PORT_PCR_MUX(1);
		PTC->PDDR |= MASK(portCGpio[i]);
		PTC->PCOR |= MASK(portCGpio[i]);
	}
	// configuring PORT A GPIO LEDS
	for(int i = 0; i < sizeof(portAGpio)/(sizeof(short)); i++)
	{
		PORTA->PCR[portAGpio[i]] |= PORT_PCR_MUX(1);
		PTA->PDDR |= MASK(portAGpio[i]);
		PTA->PCOR |= MASK(portAGpio[i]);
	}
}

// Inits the Systick to keep track of internal clock
void initSysTick()
{
	SysTick->LOAD = (48000000L/24); // lower sysclock to 1Mhz to fit in Load
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
		prev_medState = inMeditation;
}

void handleSwitches()
{
	// Handles PLUs/MINUS switches
	if(PORTD->ISFR & MASK(PLUS_SWITCH))
	{
		handleLedTimes(_plus);
		PORTD->ISFR &= 0xffffffff; // clear button flag
	}
	else if(PORTD->ISFR & MASK(MINUS_SWITCH))
	{
		handleLedTimes(_minus);
		PORTD->ISFR &= 0xffffffff; // clear button flag
	}
	else
	{
		// none
	}
	
	// Handles Meditation On/Off switch
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
		PTC->PCOR |= (MASK(SERVO_SHIFT) | MASK(LED_MED));
	}
	else
	{
		handleLedTimes(_timer);
		PTC->PSOR |= MASK(LED_MED);
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
			if(breathTime >= (inhaleMask & inhaleExhaleSettings[breathingSetting]) >> 4UL)
			{
				currentBreathState = holding;
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
	if((prevBreathState != currentBreathState))
	{
		if((currentBreathState == inhaling || currentBreathState == exhaling))// do we turn on the servo motor? If we are inhaling or exhaling we do
		{
			PTC->PSOR |= MASK(SERVO_SHIFT); // toggle servo motor
		}
		else
		{
			PTC->PCOR |= MASK(SERVO_SHIFT); // when not inhaling or exhaling
		}
	}
	else
	{
		// continue with current outputs
	}
	prevBreathState = currentBreathState;
}


// Contains the logic for Meditation timed leds.

void handleLedTimes(short transitionType)
{
	// As per default, each LED will represent 100/numLeds percent
	// of total meditation time. 
	// OR 
	// MeditationTime(minutes)/numLEDS (TBD). This is be 20 minutes by default
	// Max time is 2 hours. 
	
	float currentMedRatio = 0;
	
	switch(transitionType)
	{
		case _timer:
			// do not change time calculation
			currentMedRatio = (float)medTimeCurrent / (float)medTimeMax;
			currentMedState = ceil(currentMedRatio *  MED_LED_NUM);
		break;
		case _plus:
			// increment only if at 3 or less
			if(currentMedState < 4)
			{
				currentMedState++;
				medTimeCurrent = medTimeMax * (float)((float)currentMedState/(float)MED_LED_NUM); // set the current time
			}
			// do plus
		break;
		case _minus:
			// decrement only if at greater than 0
			if(currentMedState > 0)
			{
				currentMedState--;
				medTimeCurrent = medTimeMax * (float)((float)currentMedState/(float)MED_LED_NUM);
			}
			// do minus
		break;
		default:
			// do timer
			currentMedRatio = (float)medTimeCurrent / (float)medTimeMax;
			currentMedState = ceil(currentMedRatio *  MED_LED_NUM);
		break;
	}
	
	// LEVELS
	/*
	0.01 -> 0.25 1 LED
	0.26 -> 0.50 2 LED
	0.51 -> 0.75 3 LED
	0.76 -> 1.00 4 LED	
	*/
	// We will know the new current state prior to switch statement
	if(prevMedLedState != currentMedState)
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
		if(currentMedState == 0)
		{
			inMeditation = 0;
			medTimeCurrent = medTimeMax; // reset to max
		}
		// update the LEDS
	}
	else
	{
		// Do nothing, LEDs are in correct state
	}
	prevMedLedState = currentMedState; // Save the previous state.
}


// Sets the LEDS to the proper state

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

// Handles the timer

void SysTick_Handler()
{
	// if we are in an intermmediat state, we delay breathTime
	if(inMeditation)
	{
		medTimeCurrent--;
		breathStateMachine();
		handleLedTimes(_timer);
	}
}

//Function is added by Pavan
void initHeater  (uint16_t period)
{
	// enable the clock for PORTD
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;  									
	
	// Init the Heater PWM (Here LED is simulated as Heater)
	PORTD->PCR[LEDHeater] &= ~PORT_PCR_MUX_MASK;
	// Select the PIN for ALT4/TPM functionality
	PORTD->PCR[LEDHeater] |= PORT_PCR_MUX(4);
	// Selecting the LED pins data direction registers is not advised here. 
	// The PWM functionality does not run in thst case
	
	//enable the clock for TPM
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	//Configure TPM
	
	//Set clock source for tpm : 48 MHz
	SIM	->	SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	// Selecting the clock from the board, internal clock is set to use
	SIM -> 	SOPT2 |= SIM_SOPT2_TPMSRC (1);

	SIM-> SOPT2 &= ~SIM_SOPT2_PLLFLLSEL_MASK;
	SIM->	SOPT2 |= SIM_SOPT2_PLLFLLSEL(0);
	
	//Load the counter and MOD, this is active low PWM signal
	TPM0-> MOD = period-1;
	
	//Set TPM count direction to up with a divide by 2 prescaler

	TPM0->SC &= ~((TPM_SC_CPWMS_MASK) | (TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC  = TPM_SC_CMOD(1) | TPM_SC_PS(2) | TPM_SC_TOIE_MASK | TPM_SC_CPWMS_MASK;
	
	//Continue operation in debug mode
	TPM0->CONF  |= TPM_CONF_DBGMODE(3);
	
	// Configured as Edge Aligned PWM, 	
	//Set channel TPM0 Channel 0 to edge-aligned low true PWM
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK));
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_ELSA(0) |  TPM_CnSC_MSB(1)  | TPM_CnSC_MSA(0));
	
}	// End of Init Heater function

//Function is added by Pavan
void setHeaterPWMDutyCycle (void) 
{

//detrmine the inhale exhale setting
//Inhale timinmg could independently determine the setting uniquely 
//static short inhaleExhaleSettings[3] = {0x22, 0x32, 0x43}; 									// Inhale/Exhale timer

InhaleExhaleSetting = ((inhaleMask & inhaleExhaleSettings[breathingSetting]) >> 4UL);

short intHeatingLevel = 0;
short intMinHeatingLevel = 0;
short intMaxHeatingLevel = 5;
short intOF = 0; 
uint32_t HeaterPWM = PWM_PERIOD;

switch(InhaleExhaleSetting)
    {   
	case 0: 	//No heating requested
		HeaterPWM = PWM_PERIOD;
		break;
	case 1: 		//20% PWM
         	HeaterPWM = (InhaleExhaleSetting) * PWM_PERIOD/intMaxHeatingLevel;
         	break;
	case 2: 		//40% PWM
  		HeaterPWM = (InhaleExhaleSetting) * PWM_PERIOD/intMaxHeatingLevel;
         	break;
	case 3: 		//60% PWM
		HeaterPWM = (InhaleExhaleSetting) * PWM_PERIOD/intMaxHeatingLevel;
         	break;
	case 4: 		//80% PWM
 		HeaterPWM = (InhaleExhaleSetting) * PWM_PERIOD/intMaxHeatingLevel;
         	break;
	case 5: 		//Maximum hearting with 100% PWM, unused in this program
 		HeaterPWM = (InhaleExhaleSetting) * PWM_PERIOD/intMaxHeatingLevel;
         	break;
	default: 	//Indetermined state, Unknown state, hold the last value
 		HeaterPWM = PWM_PERIOD;
 		break;
    } //End of the switch case

		TPM0->CONTROLS[1].CnV=HeaterPWM;
	
} // End of setHeaterPWMDutyCycle function
