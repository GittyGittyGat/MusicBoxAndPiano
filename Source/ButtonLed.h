// ButtonLed.h: starter file for CECS447 Project 1 Part 1
// Runs on TM4C123, 
// Omar Fayoumi
// September 4, 2024
// Port B bits 5-0 outputs to the 6-bit DAC
// Port D bits 3-0 inputs from piano keys: CDEF:doe ray mi fa,negative logic connections.
// Port F is onboard LaunchPad switches and LED
// SysTick ISR: PF3 is used to implement heartbeat
#ifndef __BUTTONLED_H__
#define __BUTTONLED_H__

#include <stdint.h>

// Constants
#define PIANO     0  // piano mode: press a key to play a tone
#define AUTO_PLAY 1  // auto play mode: automatic playing a song

// moved from ButtonLed.c
#define SW1 0x10  // bit position for onboard switch 1(left switch)
#define SW2 0x01  // bit position for onboard switch 2(right switch)

//---------------------Switch_Init---------------------
// initialize switch interface
// Input: none
// Output: none 
void ButtonLed_Init(void);

//---------------------PianoKeys_Init---------------------
// initialize onboard Piano keys interface: PORT D 0 - 3 are used to generate 
// tones: CDEF:doe ray mi fa
// No need to unlock. Only PD7 needs to be unlocked.
// Input: none
// Output: none 
void PianoKeys_Init(void);

uint8_t get_current_mode(void);

// used to toggle modes
void toggle_current_mode(void);
#endif