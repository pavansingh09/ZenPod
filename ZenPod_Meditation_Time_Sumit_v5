#include <stdio.h>
#include <MKL25Z4.h>
#include "MKL25Z4.h" 

void Delay(volatile unsigned int);
static int d1 = 0;
int Final = 0;
// Device header
//#define 4 LEDS on PORT A
#define LED1 (1) //PTA1
#define LED2 (2) //PTA2
#define LED3 (4) //PTA4
#define LED4 (5) //PTA5

//#define 4 pin Switches on PORT D
#define SW1 (4) //PTD4
#define SW2 (5) //PTD4
#define MASK(x) (1UL << (x))

void Delay(volatile unsigned int time_del) {
	while (time_del--){
		;
	}
}

int main () { //Main function starts here
	//Enable clock to  PORT A and PORT D
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	//Make GPIO pins for ZenPod timer
	PORTA->PCR[LED1]&= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[LED1] |= PORT_PCR_MUX(1);
	PORTA->PCR[LED2]&= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[LED2] |= PORT_PCR_MUX(1);
	PORTA->PCR[LED3]&= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[LED3] |= PORT_PCR_MUX(1);
	PORTA->PCR[LED4]&= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[LED4] |= PORT_PCR_MUX(1);
		
	//Set ports to Inputs and Outputs
	PTA->PDDR |= MASK(LED1)|MASK(LED2)|MASK(LED3)|MASK(LED4);
	PTD->PDDR &= ~MASK(SW1);
	PTD->PDDR &= ~MASK(SW2);
	
	// Configure port peripheral. Select GPIO and Interrupts on raising edge
	//PORTE->PCR[SW2] = PORT_PCR_MUX(1)|PORT_PCR_IRQC(9)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
	PORTD->PCR[SW1] = PORT_PCR_MUX(1)|PORT_PCR_IRQC(9);
	PORTD->PCR[SW2] = PORT_PCR_MUX(1)|PORT_PCR_IRQC(9);

	//Clear all the fields of LED's
	PTA->PCOR = MASK(LED1)|MASK(LED2)|MASK(LED3)|MASK(LED4);
	
	while(1)
	{
		//assigning count to d1 variable based on switch press
		Delay(700000);
		if (PORTD->ISFR & MASK(SW1))
		{	
			if (d1 <= 3 )
			{
				d1 = d1 + 1;
				PORTD->ISFR = 0xffffffff;
				Delay(700000);
			}
			else
			{
				d1 = 4;
				Delay(700000);
			}
		 }
		if (PORTD->ISFR & MASK(SW2))
		{	
			if (d1 > 0)
			{
				d1 = d1 - 1;
				PORTD->ISFR = 0xffffffff;
				Delay(700000);
			}
			else
			{
				d1 = 0;
				Delay(700000);
			}
		 }
    //Passing on d1 value from above step to Final for FSM
    Final = d1;	
    Delay(900000);		 
		switch(Final)
        {   
					  //FSM based on switch pressed, this shows time oncrement or decrement 
            case 0:
							  //Time = 0mins
						    PTA->PCOR = MASK(LED1)| MASK(LED2)|MASK(LED3)|MASK(LED4);
								Delay(700000);
                break;
			      case 1:
							  //Time = 5mins
			          PTA->PSOR = MASK(LED1);
						    PTA->PCOR = MASK(LED2)|MASK(LED3)|MASK(LED4);
								Delay(700000);
                break;
            case 2: 
							  //Time = 10mins
			          PTA->PSOR = MASK(LED1)|MASK(LED2);
						    PTA->PCOR = MASK(LED3)|MASK(LED4);
						    Delay(700000);
                break;
						case 3: 
							  //Time = 15mins
			          PTA->PSOR = MASK(LED1)|MASK(LED2)|MASK(LED3);
						    PTA->PCOR = MASK(LED4);
						    Delay(700000);
                break;
						case 4: 
							  //Time = 10mins
						    PTA->PSOR = MASK(LED1)|MASK(LED2)|MASK(LED3)|MASK(LED4);
                Delay(700000);
						    break;

        }
	}
}
