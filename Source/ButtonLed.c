// ButtonLed.c: starter file for CECS447 Project 1 Part 1
// Runs on TM4C123, 
// Omar Fayoumi
// September 4, 2024
// Port B bits 5-0 outputs to the 6-bit DAC
// Port D bits 3-0 inputs from piano keys: CDEF:doe ray mi fa,negative logic connections.
// Port F is onboard LaunchPad switches and LED
// SysTick ISR: PF3 is used to implement heartbeat

#include "tm4c123gh6pm.h"
#include <stdint.h>
#include "ButtonLed.h"

// Constants
#define PORTF_BUTTONS (SW1 | SW2)
#define NVIC_EN0_PORTF 0x40000000  // enable PORTF edge interrupt
#define NVIC_EN0_PORTD 0x00000008  // enable PORTD edge interrupt
#define NVIC_EN0_PORTE 0x00000010  // enable PORTE edge interrupt

#define PORTD_BUTTONS 0x0FUL // CDEF 0F = 0000 1111
#define PORTE_BUTTONS 0x07UL // GAB	 07 = 0000 0111

#define PORTF_PRI_BITS 	0x00E00000
#define PORTF_INT_PRI   5UL

#define PORTD_PRI_BITS	0xE0000000
#define PORTD_INT_PRI		6UL

#define PORTE_PRI_BITS 	0x000000E0
#define PORTE_INT_PRI	  4UL
// Golbals
volatile uint8_t curr_mode=PIANO;  // 0: piano mode, 1: auto-play mode

//---------------------Switch_Init---------------------
// initialize onboard switch and LED interface
// Input: none
// Output: none 
void ButtonLed_Init(void){ 
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;
	while((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOF) != SYSCTL_RCGC2_GPIOF);
	GPIO_PORTF_LOCK_R = 0x4C4F434B;         // Unlock PORTF CR
	GPIO_PORTF_CR_R |= PORTF_BUTTONS;       // Enable editing to PF4 and PF0
	GPIO_PORTF_AMSEL_R &= ~PORTF_BUTTONS;   // not analog
	GPIO_PORTF_PCTL_R &= ~0x000F000F;       // regular GPIO
	GPIO_PORTF_DIR_R &= ~PORTF_BUTTONS;     // input for PF4 and 0
	GPIO_PORTF_AFSEL_R &= ~PORTF_BUTTONS;   // disable alternate functions
	GPIO_PORTF_PUR_R |= PORTF_BUTTONS;      // pull up both PF0 and 4 
	GPIO_PORTF_DEN_R |= PORTF_BUTTONS;			// digital enable on PE0 and 4
	GPIO_PORTF_IS_R &= ~PORTF_BUTTONS;      // EDGE innterupts
  GPIO_PORTF_IBE_R &= ~PORTF_BUTTONS;     // not BOTH edges
	GPIO_PORTF_IEV_R &= ~PORTF_BUTTONS;			// falling edge event
	GPIO_PORTF_ICR_R |= PORTF_BUTTONS;      // clear flags
	GPIO_PORTF_IM_R |= PORTF_BUTTONS;       // Arm interrupts
	NVIC_PRI7_R = (NVIC_PRI7_R&~PORTF_PRI_BITS)|PORTF_INT_PRI<<21; // PORTF Interrupt priority bits: 23-21, priority set to 5
	NVIC_EN0_R |= NVIC_EN0_PORTF;           // enable bit 30 interrupt
	
}

//---------------------PianoKeys_Init---------------------
// initialize onboard Piano keys interface: PORT D 0 - 3 are used to generate 
// tones: CDEF:doe ray mi fa
// No need to unlock. Only PD7 needs to be unlocked.
// Input: none
// Output: none 
void PianoKeys_Init(void){ 
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD;
	while((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOD) != SYSCTL_RCGC2_GPIOD);
	GPIO_PORTD_LOCK_R = 0x4C4F434B;
	GPIO_PORTD_CR_R |= PORTD_BUTTONS;
	GPIO_PORTD_AMSEL_R &= ~PORTD_BUTTONS;  // disable analog
	GPIO_PORTD_PCTL_R &= ~0x0FFFFFFFUL;    // regular GPIO
	GPIO_PORTD_AFSEL_R &= ~PORTD_BUTTONS;  // disable alternate function select
	GPIO_PORTD_DIR_R &= ~PORTD_BUTTONS;    // set to input
	GPIO_PORTD_PUR_R |= PORTD_BUTTONS;     // set pull-up resistors
	GPIO_PORTD_DEN_R |= PORTD_BUTTONS;     // digital enable PD6-0
	GPIO_PORTD_IS_R = ~PORTD_BUTTONS;			 // edge interrupts
	GPIO_PORTD_IBE_R |= PORTD_BUTTONS;	   // both edges
	GPIO_PORTD_ICR_R |= PORTD_BUTTONS;     // clear flags
	GPIO_PORTD_IM_R |= PORTD_BUTTONS;      // arm interrupt
	NVIC_PRI0_R = (NVIC_PRI0_R&~PORTD_PRI_BITS)|PORTD_INT_PRI<<29; // PORTD interrupt prio bits: 31-29, prio set to 6
	NVIC_EN0_R |= NVIC_EN0_PORTD;          // enable bit 4 interrupt
	
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;
	while((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOE) != SYSCTL_RCGC2_GPIOE);
	GPIO_PORTE_LOCK_R = 0x4C4F434B;
	GPIO_PORTE_CR_R |= PORTE_BUTTONS;
	GPIO_PORTE_AMSEL_R &= ~PORTE_BUTTONS;  // disable analog
	GPIO_PORTE_PCTL_R &= ~0x00000FFFUL;    // regular GPIO
	GPIO_PORTE_AFSEL_R &= ~PORTE_BUTTONS;  // disable alternate function select
	GPIO_PORTE_DIR_R &= ~PORTE_BUTTONS;    // set to input
	GPIO_PORTE_PUR_R |= PORTE_BUTTONS;     // set pull-up resistors
	GPIO_PORTE_DEN_R |= PORTE_BUTTONS;     // digital enable PD6-0
	GPIO_PORTE_IS_R = ~PORTE_BUTTONS;			 // edge interrupts
	GPIO_PORTE_IBE_R |= PORTE_BUTTONS;	   // both edges
	GPIO_PORTE_ICR_R |= PORTE_BUTTONS;     // clear flags
	GPIO_PORTE_IM_R |= PORTE_BUTTONS;      // arm interrupt
	NVIC_PRI1_R = (NVIC_PRI1_R & ~PORTE_PRI_BITS) | PORTE_INT_PRI<<5;
	NVIC_EN0_R |= NVIC_EN0_PORTE;
}

uint8_t get_current_mode(void)
{
	return curr_mode;
}

void toggle_current_mode(void){
	curr_mode ^= AUTO_PLAY;
}