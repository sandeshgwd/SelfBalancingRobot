#include <p33FJ128MC802.h>
#define FCY 40000000UL                       // instruction cycle rate (ignore)
#include <libpic30.h>                        // __delay32
                                             // __delay_ms and __delay_us
                                             // note: use only small values

/*
MAX_COUNT VALUES
TCKPS = 0     ::      Prescaler of 1
250 us = 10000
1ms    = 40000

TCKPS = 1     ::      Prescaler of 8


TCKPS = 2     ::      Prescaler of 64


TCKPS = 3     ::      Prescaler of 256

*/
#define MAX_COUNT 10000               

#define LEFT 0b01
#define RIGHT 0b10
#define BRAKE 0b00



void set_dir(int d)
{
	switch(d)
	{
		case LEFT:
		LATBbits.LATB14 = 1;
		LATBbits.LATB13 = 0;
		LATBbits.LATB11 = 1;
		LATBbits.LATB12 = 0;
		break;
		
		case RIGHT:
		LATBbits.LATB14 = 0;
		LATBbits.LATB13 = 1;
		LATBbits.LATB11 = 0;
		LATBbits.LATB12 = 1;
		break;
		
		case BRAKE:
		LATBbits.LATB14 = 0;
		LATBbits.LATB13 = 0;
		LATBbits.LATB11 = 0;
		LATBbits.LATB12 = 0;
		break;
		
		default:
		LATBbits.LATB14 = 0;
		LATBbits.LATB13 = 0;
		LATBbits.LATB11 = 0;
		LATBbits.LATB12 = 0;
		break;
	}
}
void drive(unsigned int speed)	
{
	OC1RS = speed;
	OC2RS = speed;
}
	

int main()
{
	// 40 MHz freq
	PLLFBDbits.PLLDIV = 38;					// pll feedback divider = 40;
	CLKDIVbits.PLLPRE = 0;					// pll pre divider = 2
	CLKDIVbits.PLLPOST = 0;					// pll post divider = 2
	
	//Init pwm pins
	RPOR7bits.RP15R = 18;					// connect OC1 to RP15 (PWM)
	//RPOR6bits.RP12R = 19;					// connect OC2 to RP12 (PWM)
	RPOR5bits.RP10R = 19;					// connect OC2 to RP10 (PWM)
	
	//Init direction pins
	TRISBbits.TRISB14 = 0;
	TRISBbits.TRISB13 = 0;
	TRISBbits.TRISB12 = 0;
	TRISBbits.TRISB11 = 0;
	
	//Init timer 2
	T2CONbits.TCS = 0;
	T2CONbits.TCKPS = 0;
	T2CONbits.TON = 1;
	PR2 = MAX_COUNT;
	
	//Init Output compare logic
	// turn-off OC1 to make sure changes can be applied
	OC1CON = 0;
	// set first cycle d.c.
	OC1R = 0;
	// set ongoing d.c. to 0
	OC1RS = 0;
	// Set OC1 to PWM mode with TMR2 clock source
	OC1CONbits.OCTSEL = 0;
	OC1CONbits.OCM = 6;
	// turn-off OC2 to make sure changes can be applied
	OC2CON = 0;
	// set first cycle d.c.
	OC2R = 0;
	// set ongoing d.c. to 0
	OC2RS = 0;
	// Set OC2 to PWM mode with TMR2 clock source
	OC2CONbits.OCTSEL = 0;
	OC2CONbits.OCM = 6;

	set_dir(LEFT);
	drive(2000);
	__delay_ms(250);
	drive(4000);
	__delay_ms(250);
	drive(6000);
	__delay_ms(250);
	drive(9000);
	__delay_ms(250);
	__delay_ms(250);
	__delay_ms(250);
	__delay_ms(250);
	__delay_ms(250);
	drive(0);
	drive(4000);
	while(1)
	{
		set_dir(BRAKE);
		__delay_ms(1);
		set_dir(LEFT);
		__delay_ms(250);
		__delay_ms(250);
		__delay_ms(250);
		__delay_ms(250);
		
		set_dir(BRAKE);
		__delay_ms(1);
		set_dir(RIGHT);
		__delay_ms(250);
		__delay_ms(250);
		__delay_ms(250);
		__delay_ms(250);
	}
}