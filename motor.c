#include "MKL25Z4.h"                    // Device header
#include "RTE_Components.h"             // Component selection
#include "system_MKL25Z4.h"             // Keil::Device:Startup

#define SERVO_SHIFT (0) // PORTC relay connected to Servo.
#define LED_BLUE (3) // PORTC LED is meditation is still active
#define LED_GREEN (7) // PORTC led if inhaling/exhaling
#define HZDELAY (20000) // alternate frequency
#define MASK(x) (1ul << x)
#define SW_SWITCH (4) // starts the meditation
#define BREATH_DELAY (2) // 2 seconds between inhale and exhale

void initPins(void);
void initSysTick(void);
static short portCGpio[3] = {SERVO_SHIFT, LED_BLUE, LED_GREEN};
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
				PTC->PCOR |= (MASK(SERVO_SHIFT) | MASK(LED_BLUE) | MASK(LED_GREEN));
			}
			else
			{
				inMeditation = 1;
				PTC->PSOR |= MASK(LED_BLUE);
			}
			PORTD->ISFR &= 0xffffffff; // clear button flag
		}
		
		//delay(2500);
	}
}

void initPins()
{
	SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK);
	PORTD->PCR[SW_SWITCH] &= ~(PORT_PCR_MUX_MASK | PORT_PCR_IRQC_MASK);
	PORTD->PCR[SW_SWITCH] |= (PORT_PCR_MUX(1) | PORT_PCR_IRQC(10) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK); // gpio, check falling edge, pullup resistor set
	PTD->PDDR &= (uint8_t)~MASK(SW_SWITCH); // set pin direction as input
	for(int i = 0; i < sizeof(portCGpio)/(sizeof(short)); i++)
	{
		PORTC->PCR[portCGpio[i]] |= PORT_PCR_MUX(1);
		PTC->PDDR |= MASK(portCGpio[i]);
		PTC->PDOR |= MASK(portCGpio[i]);
		PTC->PCOR |= MASK(portCGpio[i]);
	}
}

void initSysTick()
{
	SysTick->LOAD = (48000000L/16); // lower sysclock to 3Mhz to fit in Load
	NVIC_SetPriority(SysTick_IRQn, 3);
	NVIC_ClearPendingIRQ(SysTick_IRQn);
	NVIC_EnableIRQ(SysTick_IRQn);
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
				PTC->PTOR |= MASK(LED_GREEN);
				breathTime = 0;
			}
		}
}
