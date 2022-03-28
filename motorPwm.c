#include "MKL25Z4.h"                    // Device header
#include "RTE_Components.h"             // Component selection
#include "system_MKL25Z4.h"             // Keil::Device:Startup

#define SERVO_SHIFT (0) // PORTC relay connected to Servo.
#define HZDELAY (20000) // alternate frequency
#define MASK(x) (1ul << x)
#define PWM_PERIOD (48000)
#define SW_SWITCH (4) // starts the meditation
#define BREATH_DELAY (2) // 2 seconds between inhale and exhale

void initSwitch(void);
void initServoPwm(void);
void controlPwm(void);

static short breathTime = 0;
static short inMeditation = 0;

int main()
{
	initSwitch(); // init speaker port
	initServoPwm(); // init pit timer
	while(1)
	{
		//PTC->PSOR |= MASK(SERVO_SHIFT);
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
				PTC->PSOR |= MASK(SERVO_SHIFT);
				//controlPwm();
			}
			PORTD->ISFR &= 0xffffffff; // clear button flag
		}
		
		//delay(2500);
	}
}

void initSwitch()
{
	SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK);
	PORTD->PCR[SW_SWITCH] &= ~(PORT_PCR_MUX_MASK | PORT_PCR_IRQC_MASK);
	PORTD->PCR[SW_SWITCH] |= (PORT_PCR_MUX(1) | PORT_PCR_IRQC(10) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK); // gpio, check falling edge, pullup resistor set
}

void initServoPwm()
{
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	
	PORTC->PCR[SERVO_SHIFT] &= PORT_PCR_MUX_MASK;
	PORTC->PCR[SERVO_SHIFT] |= PORT_PCR_MUX(1);
	
	PTC->PDOR |= MASK(SERVO_SHIFT); // controlling output register
	PTC->PDDR |= MASK(SERVO_SHIFT); // setting direction register
	
	SIM->SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK);
	
	TPM0->MOD = PWM_PERIOD - 1;
	
	TPM0->SC = TPM_SC_PS(7);
	
	TPM0->CONF |= TPM_CONF_DBGMODE(3);
	
	TPM0->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	
	TPM0->CONTROLS[1].CnV = 0;
	
	TPM0->SC |= TPM_SC_CMOD(1);
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
