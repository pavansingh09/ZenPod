#include "MKL25Z4.h"                    // Device header
#include "system_MKL25Z4.h"             // Keil::Device:Startup
#include <stdint.h> 

#define 
#define MASK(x) (uint32_t)(1UL << x);

static void gpioInit(void);
static void uart0Init(void);

void gpioInit()
{
	
	
}

void uart0Init()
{
	SIM_SCGC5 |= MASK(
	
}