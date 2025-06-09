// Sound.c
// This is the starter file for CECS 447 Project 1 Part 2
// Omar Fayoumi
// September 4, 2024
// Port B bits 5-0 outputs to the 6-bit DAC
// Port D bits 3-0 inputs from piano keys: CDEF:doe ray mi fa,negative logic connections.
// Port F is onboard LaunchPad switches and LED
// SysTick ISR: PF3 is used to implement heartbeat

#include "tm4c123gh6pm.h"
#include "sound.h"
#include "buttonLed.h"
#include <stdint.h>

#define DAC_MASK 0x3FUL

// define bit addresses for Port B bits 0,1,2,3,4,5 => DAC inputs 
#define DAC 	(*((volatile unsigned long *)0x400050FC))   
#define BUT1 	(*((volatile unsigned long *)0x40007004))
#define BUT2	(*((volatile unsigned long *)0x40007008))
#define BUT3	(*((volatile unsigned long *)0x40007010))
#define BUT4	(*((volatile unsigned long *)0x40007020))
#define BUT5	(*((volatile unsigned long *)0x40024004))
#define BUT6	(*((volatile unsigned long *)0x40024008))
#define BUT7	(*((volatile unsigned long *)0x40024010))
#define BUT1_MASK	0x01U
#define BUT2_MASK 0x02U
#define BUT3_MASK 0x04U
#define BUT4_MASK	0x08U
#define BUT5_MASK	0x01U
#define BUT6_MASK 0x02U
#define BUT7_MASK 0x04U

	
// 6-bit: value range 0 to 2^6-1=63, 64 samples
const uint8_t SineWave[64] = {32,35,38,41,44,47,49,52,54,56,58,59,61,62,62,63,63,63,62,62,
															61,59,58,56,54,52,49,47,44,41,38,35,32,29,26,23,20,17,15,12,
															10, 8, 6, 5, 3, 2, 2, 1, 1, 1, 2, 2, 3, 5, 6, 8,10,12,15,17,
															20,23,26,29};

// initial values for piano major tones.
// Assume SysTick clock frequency is 16MHz.
const uint32_t tonetab[] =
// C, D, E, F, G, A, B
// 1, 2, 3, 4, 5, 6, 7
// lower C octave:130.813, 146.832,164.814,174.614,195.998, 220,246.942
// calculate reload value for the whole period:Reload value = Fclk/Ft = 16MHz/Ft
{122137,108844,96970,91429,81633,72727,64777,
 30534*2/*lower c*/,27211*2,24242*2,22923*2,20408*2,18182*2,16194*2, // C5 Major notes
 15289*2 /*middle c*/,13621*2,12135*2,11454*2,10204*2,9091*2,8099*2,   // C5 Major notes
 7645*2 /*upper c*/,6810*2,6067*2,5727*2,5102*2,4545*2,4050*2};        // C6 Major notes



// Constants
// index definition for tones used in happy birthday.
#define C4 0
#define D4 1
#define E4 2
#define F4 3
#define G4 4
#define A4 5
#define B4 6
#define C5 0+7	//lower c
#define D5 1+7
#define E5 2+7
#define F5 3+7
#define G5 4+7
#define A5 5+7
#define B5 6+7 

#define MAX_NOTES 255 // maximum number of notes for a song to be played in the program
#define NUM_SONGS 4   // number of songs in the play list.
#define SILENCE MAX_NOTES // use the last valid index to indicate a silence note. The song can only have up to 254 notes. 
#define NUM_OCT  3   // number of octave defined in tontab[]
#define NUM_NOTES_PER_OCT 7  // number of notes defined for each octave in tonetab
#define NVIC_EN0_PORTF 0x40000000  // enable PORTF edge interrupt
#define NVIC_EN0_PORTD 0x00000008  // enable PORTD edge interrupt
#define NUM_SAMPLES 64  // number of sample in one sin wave period

// note table for Happy Birthday
// doe ray mi fa so la ti 
// C   D   E  F  G  A  B
NTyp playlist2[MAX_NOTES] = {
	G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2,
	G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2,
	B4, 4, SILENCE, 2, B4, 4, SILENCE, 2, B4, 4, SILENCE, 2, B4, 4, SILENCE, 2, A4, 4, SILENCE, 2, B4, 4, SILENCE, 2,
	C5, 8, SILENCE, 2, C5, 4, SILENCE, 2, C5, 4, SILENCE, 2, B4, 4, SILENCE, 2, A4, 4, SILENCE, 2, 
	G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2,
	G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2, G4, 4, SILENCE, 2,
	B4, 4, SILENCE, 2, B4, 4, SILENCE, 2, B4, 4, SILENCE, 2, B4, 4, SILENCE, 2, A4, 4, SILENCE, 2, B4, 4, SILENCE, 2,
	C5, 4, SILENCE, 2, G4, 4, SILENCE, 2, C5, 12, SILENCE, 0
};

NTyp playlist[][MAX_NOTES] = 
{

// score table for Mary Had A Little Lamb
{E4, 4, D4, 4, C4, 4, D4, 4, E4, 4, E4, 4, E4, 8, 
 D4, 4, D4, 4, D4, 8, E4, 4, G4, 4, G4, 8,
 E4, 4, D4, 4, C4, 4, D4, 4, E4, 4, E4, 4, E4, 8, 
 D4, 4, D4, 4, E4, 4, D4, 4, C4, 8, 0, 0 },

// score table for Twinkle Twinkle Little Stars
{C4,4,C4,4,G4,4,G4,4,A4,4,A4,4,G4,8,F4,4,F4,4,E4,4,E4,4,D4,4,D4,4,C4,8, 
 G4,4,G4,4,F4,4,F4,4,E4,4,E4,4,D4,8,G4,4,G4,4,F4,4,F4,4,E4,4,E4,4,D4,8, 
 C4,4,C4,4,G4,4,G4,4,A4,4,A4,4,G4,8,F4,4,F4,4,E4,4,E4,4,D4,4,D4,4,C4,8,0,0},
	
{
	  // Happy Birthday
// so   so   la   so   doe' ti
   G4,2,G4,2,A4,4,G4,4,C5,4,B4,4,
// pause so   so   la   so   ray' doe'
   SILENCE,4,  G4,2,G4,2,A4,4,G4,4,D5,4,C5,4,
// pause so   so   so'  mi'  doe' ti   la
   SILENCE, 4, G4,2,G4,2,G5,4,E5,4,C5,4,B4,4,A4,8, 
// pause fa'  fa'   mi'  doe' ray' doe'  stop
	 SILENCE,4,  F5,2,F5,2, E5,4,C5,4,D5,4,C5,8, SILENCE,0
},

{
	F4, 4, SILENCE, 2, F4, 4, SILENCE, 2, F4, 4, SILENCE, 2, F4, 4, SILENCE, 2, F4, 4, SILENCE, 2, F4, 4, SILENCE, 2,
	E4, 4, SILENCE, 2, E4, 4, SILENCE, 2, E4, 4, SILENCE, 2, E4, 4, SILENCE, 2, E4, 4, SILENCE, 2, E4, 4, SILENCE, 2,
	D4, 4, SILENCE, 2, D4, 4, SILENCE, 2, D4, 4, SILENCE, 2, D4, 4, SILENCE, 2, D4, 4, SILENCE, 2, D4, 4, SILENCE, 2,
	C4, 8, SILENCE, 2, C4, 4, SILENCE, 2, C4, 4, SILENCE, 2, D4, 4, SILENCE, 2, E4, 4, SILENCE, 2,
	F4, 4, SILENCE, 2, F4, 4, SILENCE, 2, F4, 4, SILENCE, 2, F4, 4, SILENCE, 2, F4, 4, SILENCE, 2, F4, 4, SILENCE, 2,
	E4, 4, SILENCE, 2, E4, 4, SILENCE, 2, E4, 4, SILENCE, 2, E4, 4, SILENCE, 2, E4, 4, SILENCE, 2, E4, 4, SILENCE, 2,
	D4, 4, SILENCE, 2, D4, 4, SILENCE, 2, D4, 4, SILENCE, 2, D4, 4, SILENCE, 2, D4, 4, SILENCE, 2, D4, 4, SILENCE, 2,
	C4, 4, SILENCE, 2, F4, 4, SILENCE, 2, C4, 12, SILENCE, 0
}
};

void InitDAC(void){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;
	while((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOB) != SYSCTL_RCGC2_GPIOB);
	GPIO_PORTB_AMSEL_R &= ~DAC_MASK; 	 // clear amsel to disable analog
	GPIO_PORTB_PCTL_R &= ~0x00FFFFFF;	 // clear PCTL to select GPIO
	GPIO_PORTB_DIR_R |= DAC_MASK;	   	 // lower six bits set to output
	GPIO_PORTB_AFSEL_R &= ~DAC_MASK; 	 // select regular GPIO
	GPIO_PORTB_DEN_R |= DAC_MASK;		 	 // digital enable
}

void InitSysTick(void){
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
	NVIC_ST_CTRL_R = (NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_CLK_SRC);
	NVIC_ST_CURRENT_R = 0;
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; //prio 3
}

void Timer0Init(void){
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;
	while((SYSCTL_RCGCTIMER_R & SYSCTL_RCGCTIMER_R0) != SYSCTL_RCGCTIMER_R0);
	TIMER0_CTL_R &= 0x00; // turn off the timer and clear control register
	TIMER0_CFG_R = TIMER_CFG_16_BIT;
	TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TAMIE | TIMER_TAMR_TACDIR; // periodic mode, enable interrupts, count down
	TIMER0_TAMATCHR_R = 0x00UL; // generate interrupt when timer reaches 0x00
	NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF) | 0x40000000; //prio 3
	NVIC_EN0_R |= 0x1<<19;
}

void Start_Timer0(uint32_t PERIOD){
	TIMER0_TAILR_R = PERIOD - 1;
	TIMER0_CTL_R |= TIMER_CTL_TAEN;
}

void Stop_Timer0(void){
	TIMER0_CTL_R &= ~TIMER_CTL_TAEN;
}

// File scope golbal
volatile uint8_t curr_song=0;      // 0: Happy Birthday, 1: Mary Had A Little Lamb. 2: Twinkle Twinkle Little Stars
volatile uint8_t stop_play=1;      // 0: continue playing a song, 1: stop playing a song
volatile uint8_t octave = 0;         // 0: lower C, 1: middle C, 2: upper C
volatile uint8_t curr_note_index = 0;
volatile uint8_t sine_wave_index = 0;
volatile uint8_t sine_wave_index_TIMER = 0;
//volatile int16_t TOTAL_DAC = -32;
volatile int16_t TOTAL_DAC = 0;
volatile uint8_t SYS_FIRST_TIME = 1;
volatile uint8_t TIMER_FIRST_TIME = 1;

																		// **************DAC_Init*********************
// Initialize 6-bit DAC 
// Input: none
// Output: none
void DAC_Init(void){
	
	//PORTD
	InitDAC();
	
	//SYSTICK INIT:
	InitSysTick();
	
	//TIMER 0 LATER
	Timer0Init();
}

void Timer0A_Handler (void) {
        if(TIMER_FIRST_TIME){
            TOTAL_DAC += (SineWave[sine_wave_index_TIMER]>>1); //add sinewave /2
            TIMER_FIRST_TIME = 0;
            DAC = TOTAL_DAC;
            /*if(TOTAL_DAC < 0){
                DAC = 0;
            }
            else if(TOTAL_DAC > 63){
                DAC = 63;
            }
            else{
                DAC = TOTAL_DAC;
            }*/
        }
        else{
            TOTAL_DAC -= (SineWave[sine_wave_index_TIMER]>>1); //subtract sinewave /2
            sine_wave_index_TIMER = (sine_wave_index_TIMER + 1) % NUM_SAMPLES;
            TOTAL_DAC += (SineWave[sine_wave_index_TIMER]>>1); //add sinewave /2
            DAC = TOTAL_DAC;
            /*if(TOTAL_DAC <= 0){
                DAC = 0;
            }
            else if(TOTAL_DAC >= 63){
                DAC = 63;
            }
            else{
                DAC = TOTAL_DAC;
            }*/
        }
}

// **************Sound_Start*********************
// Set reload value and enable systick timer
// Input: time duration to be generated in number of machine cycles
// Output: none
void Sound_Start(uint32_t period){
	NVIC_ST_RELOAD_R = (period >> 6) - 1; //divding the period by 64
	//previous line is equivalent to: 
	// NVIC_ST_RELOAD_R = (period / NUM_SAMPLES) - 1;
	NVIC_ST_CURRENT_R = 0;
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
}

void Sound_stop(void)
{
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
}

// Interrupt service routine
// Executed based on number of sampels in each period
void SysTick_Handler(void){
    if(curr_song < 3){
        DAC = SineWave[sine_wave_index];
        sine_wave_index = (sine_wave_index + 1) % NUM_SAMPLES;
    }
    else{
        if(SYS_FIRST_TIME){
            TOTAL_DAC += (SineWave[sine_wave_index]>>1);
            SYS_FIRST_TIME = 0;
            DAC = TOTAL_DAC;
            /*if(TOTAL_DAC <= 0){
                DAC = 0;
            }
            else if(TOTAL_DAC >= 63){
                DAC = 63;
            }
            else{
                DAC = TOTAL_DAC;
            }*/
        }
        else{
            TOTAL_DAC -= (SineWave[sine_wave_index]>>1);
            sine_wave_index = (sine_wave_index + 1) % NUM_SAMPLES;
            TOTAL_DAC += (SineWave[sine_wave_index]>>1);
            DAC = TOTAL_DAC;
            /*if(TOTAL_DAC <= 0){
                DAC = 0;
            }
            else if(TOTAL_DAC >= 63){
                DAC = 63;
            }
            else{
                DAC = TOTAL_DAC;
            }*/
        }
    }
    return;
}

void GPIOPortF_Handler(void){
	// simple debouncing code: generate 20ms to 30ms delay
	for (uint32_t time=0;time<(72724/2);time++);
	
	Sound_stop();
	
	if(GPIO_PORTF_RIS_R & SW1){
		toggle_current_mode();
		curr_song = 0;
		GPIO_PORTF_ICR_R |= SW1;
	}
	
	if(GPIO_PORTF_RIS_R & SW2){
		if(get_current_mode()){ //AUTO PLAY
			stop_play = 1;
		}
		else{ // PIANO
			octave = (octave + 1) % NUM_OCT;
		}
		GPIO_PORTF_ICR_R |= SW2;
	}
	
	return;
		
}

// Dependency: Requires PianoKeys_Init to be called first, assume at any time only one key is pressed
// Inputs: None
// Outputs: None
// Description: Rising/Falling edge interrupt on PD6-PD0. Whenever any 
// button is pressed, or released the interrupt will trigger.
void GPIOPortD_Handler(void){  
  // simple debouncing code: generate 20ms to 30ms delay
	for (uint32_t time=0;time<(72724/2);time++);
	
	Sound_stop();
	if(get_current_mode() == AUTO_PLAY){
		GPIO_PORTD_ICR_R |= (BUT1_MASK | BUT2_MASK | BUT3_MASK | BUT4_MASK);
		return;
	}
	if(GPIO_PORTD_RIS_R & BUT1_MASK){
		if(!BUT1){
			Sound_Start(tonetab[C5 + (7*octave)]);
		}
		else{
			Sound_stop();
		}
		GPIO_PORTD_ICR_R |= BUT1_MASK;
	}
	if(GPIO_PORTD_RIS_R & BUT2_MASK){
		if(!BUT2){
			Sound_Start(tonetab[D5+(7*octave)]);
		}
		else{
			Sound_stop();
		}
		GPIO_PORTD_ICR_R |= BUT2_MASK;
	}
	if(GPIO_PORTD_RIS_R & BUT3_MASK){
		if(!BUT3){
			Sound_Start(tonetab[E5+(7*octave)]);
		}
		else{
			Sound_stop();
		}
		GPIO_PORTD_ICR_R |= BUT3_MASK;
	}
	if(GPIO_PORTD_RIS_R & BUT4_MASK){
		if(!BUT4){
			Sound_Start(tonetab[F5+(7*octave)]);
		}
		else{
			Sound_stop();
		}
		GPIO_PORTD_ICR_R |= BUT4_MASK;
	}
	
	return;
}

void GPIOPortE_Handler(void){
	for (uint32_t time=0;time<(72724/2);time++);
	Sound_stop();
	if(get_current_mode() == AUTO_PLAY){
		GPIO_PORTE_ICR_R |= (BUT5_MASK| BUT6_MASK | BUT7_MASK);
		return;
	}
	if(GPIO_PORTE_RIS_R & BUT5_MASK){
		if(!BUT5){
			Sound_Start(tonetab[G5+(7*octave)]);
		}
		else{
			Sound_stop();
		}
		GPIO_PORTE_ICR_R |= BUT5_MASK;
	}
	if(GPIO_PORTE_RIS_R & BUT6_MASK){
		if(!BUT6){
			Sound_Start(tonetab[A5+(7*octave)]);
		}
		else{
			Sound_stop();
		}
		GPIO_PORTE_ICR_R |= BUT6_MASK;
	}
	if(GPIO_PORTE_RIS_R & BUT7_MASK){
		if(!BUT7){
			Sound_Start(tonetab[B5+(7*octave)]);
		}
		else{
			Sound_stop();
		}
		GPIO_PORTE_ICR_R |= BUT7_MASK;
	} 
}

// Subroutine to wait 0.1 sec
// Inputs: None
// Outputs: None
// Notes: ...
void Delay(void){
	uint32_t volatile time;
  time = 727240*20/91;  // 0.1sec
  while(time){
		time--;
  }
}

void play_a_song()
{
	stop_play = 0;
	curr_song = 0;
	curr_note_index = 0;
	
	SYS_FIRST_TIME = 1;
	TIMER_FIRST_TIME = 1;
	uint8_t time_to_delay;
	
	TOTAL_DAC = 0;
	
	while(get_current_mode() == AUTO_PLAY){
		SYS_FIRST_TIME = 1;
		TIMER_FIRST_TIME = 1;
		TOTAL_DAC = 0;
		curr_note_index = 0;
		while((playlist[curr_song][curr_note_index].delay) && (get_current_mode() == AUTO_PLAY)){
			if(stop_play){
				curr_song = (curr_song + 1) % NUM_SONGS;
				curr_note_index = 0;
				SYS_FIRST_TIME = 1;
				TIMER_FIRST_TIME = 1;
				TOTAL_DAC = 0;
				//sine_wave_index = 0;
				stop_play = 0;
			}
			if(curr_song == 3){
				if(playlist[curr_song][curr_note_index].tone_index == SILENCE){
					Sound_stop();
				}
				else{
					Sound_Start(tonetab[playlist[curr_song][curr_note_index].tone_index + (7 * octave)]);
				}
				if(playlist2[curr_note_index].tone_index == SILENCE){
					Stop_Timer0();
				}
				else{
					Start_Timer0(tonetab[playlist2[curr_note_index].tone_index + (7 * octave)]);
				}
				for(time_to_delay = 0; time_to_delay < playlist[curr_song][curr_note_index].delay; time_to_delay++){
					Delay();
				}
				Sound_stop();
				Stop_Timer0();
				curr_note_index++;
			}
			else{
				if(playlist[curr_song][curr_note_index].tone_index == SILENCE){
					Sound_stop();
				}
				else{
					Sound_Start(tonetab[playlist[curr_song][curr_note_index].tone_index + (7 * octave)]);
				}
				for(time_to_delay = 0; time_to_delay < playlist[curr_song][curr_note_index].delay; time_to_delay++){
					Delay();
				}
				Sound_stop();
				curr_note_index++;
			}	
		}
		for(time_to_delay = 0; time_to_delay < 15; time_to_delay++){
			Delay();
		}
	}
	
	return;
}


