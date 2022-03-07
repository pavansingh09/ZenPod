#include "MKL25Z4.h"                    // Device header
#include "RTE_Components.h"             // Component selection
#include "system_MKL25Z4.h"             // Keil::Device:Startup

#define SERVO_SHIFT (0) // PORTC relay connected to Servo.
#define HZDELAY (20000) // alternate frequency
#define MASK(x) (1ul << x)
#define PIT_DELAY ((10485760 * 1) - 1) // ~ 1 second PIT delay
#define SW_SWITCH (4) // starts the meditation
#define BREATH_DELAY (2) // 2 seconds between inhale and exhale

void initPins(void);
void initSysTick(void);

static short breathTime = 0;
static short inMeditation = 0;

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
	SysTick->LOAD = (48000000L/16); // lower sysclock to 3Mhz to fit in Load
	NVIC_SetPriority(SysTick_IRQn, 3);
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; // Enable interrupts and timer
}

void SysTick_Handler()
{
	if(inMeditation)
			{
				breathTime++; // increment the seconds
				if(breathTime >= BREATH_DELAY)
				{
					PTC->PTOR |= MASK(SERVO_SHIFT); // toggle servo motor
					breathTime = 0;
				}
			}
}
