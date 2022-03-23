#include "MKL25Z4.h"                    // Device header
#include "RTE_Components.h"             // Component selection
#include "system_MKL25Z4.h"             // Keil::Device:Startup

#define SERVO_SHIFT (0) // PORTC relay connected to Servo.
#define HZDELAY (20000) // alternate frequency
#define HOLD_DELAY (2) // delay between all states
#define STATE_DELAY (1)
#define inhaleMask (0xF0) // grab inhale time
#define exhaleMask (0x0F) // grab exhale time
#define MASK(x) (1ul << x)
#define PIT_DELAY ((10485760 * 1) - 1) // ~ 1 second PIT delay
#define SW_SWITCH (4) // starts the meditation

void initPins(void);
void initSysTick(void);

static short breathTime = 0; // timer for inhaling and exhaling
static short delayTime = 0; // value for delay between states
static short inMeditation = 0;
static short inhaleExhaleSettings[3] = {0x22, 0x32, 0x43}; // Inhale/Exhale timer
static short breathingSetting = 0; // default

enum breathState
{
	noBreath =0,
	inhaling,
	holding,
	exhaling
};	

static short currentState = noBreath; // init the breath state
static short prevState = noBreath; // Used to determine if we've changed states

int main()
{
	initPins(); // init speaker port
	initSysTick(); // init pit timer
	while(1)
	{
		if(PORTD->ISFR & MASK(SW_SWITCH))
		{
			// Toggle meditation
			if(inMeditation)
			{
				inMeditation = 0;
				breathTime = 0;
				PTC->PCOR |= MASK(SERVO_SHIFT);
			}
			else
			{
				inMeditation = 1;
			}
			PORTD->ISFR &= 0xffffffff; // clear button flag
		}
		
		//delay(2500);
	}
}

void initPins()
{
	SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK);
	PORTC->PCR[SERVO_SHIFT] |= PORT_PCR_MUX(1);
	PORTD->PCR[SW_SWITCH] &= ~(PORT_PCR_MUX_MASK | PORT_PCR_IRQC_MASK);
	PORTD->PCR[SW_SWITCH] |= (PORT_PCR_MUX(1) | PORT_PCR_IRQC(10) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK); // gpio, check falling edge, pullup resistor set
	PTD->PDDR &= (uint8_t)~MASK(SW_SWITCH); // set pin direction as input
	PTC->PDDR |= MASK(SERVO_SHIFT);
	PTC->PDOR |= MASK(SERVO_SHIFT);
	PTC->PCOR |= MASK(SERVO_SHIFT);
}

void initSysTick()
{
	SysTick->LOAD = (48000000L/3); // lower sysclock to 16Mhz to fit in Load
	NVIC_SetPriority(SysTick_IRQn, 3);
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; // Enable interrupts and timer
}

void SysTick_Handler()
{
	// if we are in an intermmediat state, we delay breathTime
	if(inMeditation)
			{
				switch(currentState)
				{
					case noBreath:
						delayTime++;
						if(delayTime >= STATE_DELAY)
						{
							delayTime = 0;
							currentState = inhaling;
						}
						break;
					case inhaling:
						breathTime++; // increment the seconds
						if(breathTime >= (exhaleMask & inhaleExhaleSettings[breathingSetting]))
						{
							currentState = noBreath;
							breathTime = 0;
						}
						break;
					case holding:
						delayTime++;
						if(delayTime >= HOLD_DELAY)
						{
							delayTime = 0;
							currentState = exhaling;
						}
						break;
					case exhaling:
						breathTime++; // increment the seconds
						if(breathTime >= (exhaleMask & inhaleExhaleSettings[breathingSetting]))
						{
							currentState = noBreath;
							breathTime = 0;
						}
						break;
					default:
						currentState = noBreath;
						delayTime = 0;
						break;
				}
				prevState = currentState;
				if((prevState != currentState) && (currentState == inhaling || currentState == exhaling))// do we turn on the servo motor? If we are inhaling or exhaling we do
				{
					PTC->PSOR |= MASK(SERVO_SHIFT); // toggle servo motor
				}
				else
				{
					PTC->PCOR |= MASK(SERVO_SHIFT); // when not inhaling or exhaling
				}
			}
}
