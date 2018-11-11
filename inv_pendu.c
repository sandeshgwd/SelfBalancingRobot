#include <p33FJ128MC802.h>
#define FCY 40000000UL                       // instruction cycle rate (ignore)
#include <libpic30.h>                        // __delay32
                                             // __delay_ms and __delay_us
                                             // note: use only small values
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#define USE_AND_OR                           // enable AND_OR mask setting
#include <string.h>
#define BAUD_19200 129                       // brg for low-speed, 40 MHz clock
                                             // round((40000000/16/19200)-1)

//MOTORS
/*
MAX_COUNT VALUES
TCKPS = 0     ::      Prescaler of 1
250 us = 10000
1ms    = 40000

TCKPS = 1     ::      Prescaler of 8

TCKPS = 2     ::      Prescaler of 64

TCKPS = 3     ::      Prescaler of 256
*/
#define MAX_COUNT 40000               

#define FORWARD 0b01
#define BACKWARD 0b10
#define BRAKE 0b00

//PID controller
#define Kp_backward 4260//5200
#define Kp_forward 4800//5500
#define Ki 1.16
#define Kd 0.042

//SENSOR                                             
#define ACCL 4
#define ACCL_VCFG 2
#define SAMPLES_ACCL	3

#define SAMPLES_GYRO	10
#define GYRO 5
#define GYRO_VCFG 0

#define STABLE_ANGLE 90

//##############   Accelerometer variables
// Accelration output through PWM (capturing the rising and falling edge)
#define RISING 1
#define FALLING 0

int samplesIndexAccl = 0;

long int X_out = 0, Y_out = 0, X_out_mapped, Y_out_mapped;
float X_g, Y_g;
float temp_angle = 0;

float D;

float accl_angle=0;
float accl_angle_buf[SAMPLES_ACCL];

//volatile int INT1_interruptsetting = 1;
volatile int INT1_interruptsetting = RISING;
volatile long int TMR4_captured = 0;
volatile long int max_timer4_ticks=0;
volatile int Y_available;

//volatile int INT2_interruptsetting = 1;
volatile int INT2_interruptsetting = RISING;
volatile long int TMR5_captured = 0;
volatile long int max_timer5_ticks=0;
volatile int X_available;

long int X_plus1g = 31100;
long int X_minus1g = 18850;

long int Y_plus1g = 31150;
long int Y_minus1g = 18750;

long int X_0g_count = 25000;
long int Y_0g_count = 25000;

//######################################	

float gyroVoltage = 5;         	//Gyro is running at 5V
int gyroReferenceADC = 0;   	//Gyro is zeroed at 2.3V
float gyroSensitivity = .015;  	//Our example gyro is 15mV/deg/sec
float rotationThreshold = 1;   	//Minimum deg/sec to keep track of - helps with gyro drifting
float gyroRate = 0;
float currentAngle = 90;        //Keep track of our current angle
float gyroErrorVoltage = 0;
float gyroAngleChange = 0;
//float prevGyroAngleChange = 0;

//float acclVoltage = 5;        //Gyro is running at 5V
int acclReferenceADC = 0;   	//Gyro is zeroed at 2.3V
//float acclSensitivity = .015; //Our example gyro is 15mV/deg/sec
//float rotationThreshold = 1;  //Minimum deg/sec to keep track of - helps with gyro drifting
//float gyroRate = 0;
//float gyroErrorVoltage = 0;
//float gyroAngleChange = 0;
//float prevGyroAngleChange = 0;


int samplesGyro[SAMPLES_GYRO], samplesAccl[SAMPLES_ACCL];
int samplesIndexGyro = 0;
float averaged_sample_gyro, averaged_sample_accl;
unsigned long int sumGyro=0, sumAccl = 0;

//pid controller
float angleDeviation=0;
float Elast = 0;
long int I = 0;
long int vel = 0;
//-----------------------------------------------------------------------------
// Globals             
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines                
//-----------------------------------------------------------------------------

void set_dir(int d)
{
	switch(d)
	{
		case FORWARD:
		LATBbits.LATB14 = 1;
		LATBbits.LATB13 = 0;
		LATBbits.LATB11 = 1;
		LATBbits.LATB12 = 0;
		break;
		
		case BACKWARD:
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

void init_hw()
{
	// 40 MHz freq
	PLLFBDbits.PLLDIV = 38;					// pll feedback divider = 40;
	CLKDIVbits.PLLPRE = 0;					// pll pre divider = 2
	CLKDIVbits.PLLPOST = 0;					// pll post divider = 2
	
	RPOR4bits.RP8R = 3;                  	// U1Tx to RP8
	
	//Init pwm pins
	RPOR7bits.RP15R = 18;					// connect OC1 to RP15 (PWM)
	//RPOR6bits.RP12R = 19;					// connect OC2 to RP12 (PWM)
	RPOR5bits.RP10R = 19;					// connect OC2 to RP10 (PWM)
	
	//Init direction pins
	TRISBbits.TRISB14 = 0;
	TRISBbits.TRISB13 = 0;
	TRISBbits.TRISB12 = 0;
	TRISBbits.TRISB11 = 0;
}
void pwm_init()
{
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
}	
//################### Analog to Digital Conversion ######################
void adc_init()
{
	// setup conversion clock to 1 MHz (arbitrary)
	// (ADCS = Tad / Tcy - 1 = 39)
	AD1CON3bits.ADCS = 39;
	
	AD1CON1bits.AD12B = 1;
	// enable auto-convert
	AD1CON1bits.SSRC = 7;
	
	// set auto-sample time to 31 x Tad
	AD1CON3bits.SAMC = 31;
	
	// set format for 10b result to unsigned integer
	AD1CON1bits.FORM = 0;
	
	// turn-on A/D
	AD1CON1bits.ADON = 1;
	// set positive input to AN4, negative to Vref-
	AD1CHS0bits.CH0NA = 0;
}

int read_adc(int chan, int vcfg)
{	
	AD1CON2bits.VCFG = vcfg;		//AN4 = 2; AVdd and Vref-; AN5 = 0; AVdd & AVss
	AD1CHS0bits.CH0SA = chan;		//AN4 (pin6) = 4 and AN5 (pin7) = 5
	// clear done bit
	AD1CON1bits.DONE = 0; 
	
	// enable auto-sampling
	AD1CON1bits.ASAM = 1; 
	
	// sample and wait for conversion
	while (!AD1CON1bits.DONE);  
	
	// end auto-sampling
	AD1CON1bits.ASAM = 0; 
	
	// read result and return
	return (int)(ADC1BUF0);
}

//###############################################################

//################# Serial functions ############
void serial_init(int baud_rate)
{
	// set baud rate
	U1BRG = baud_rate;
	// enable uarts, 8N1, low speed brg
	U1MODE = 0x8000;
	// enable tx and rx
	U1STA = 0x0400;
	U1STAbits.UTXISEL1 = 0;
	U1STAbits.UTXISEL0 = 0;
	//IEC0bits.U1TXIE = 1;
}
void serial_putc(char c) //STEP 8
{
    while(U1STAbits.UTXBF);
    // write character
    U1TXREG = c;
}
void serial_puts(char *a)
{
	int index=0;

	while(a[(int)index]!='\0')
	{
		serial_putc(a[(int)index]);
		index++;
	}
	index=0;
}
void serial_putuint(unsigned int x)
{
	char integer[5];
	int place=4;
	if(x==0)
	{
		serial_putc('0');
	}
	while(x!=0 && place>=0)
	{
		integer[place]=(x%10)+48;
		x/=10;
		place--;
	}
	place++;
	while(place<=4)
	{
		serial_putc(integer[place]);
		place++;
	}
}

void serial_putint(int x)
{
	char integer[5];
	int place=4;
	
	if(x==0)
	{
		serial_putc('0');
	}
	if(x<0)
	{
		serial_putc('-');
		x = -x;
	}
		
	while(x!=0 && place>=0)
	{
		integer[place]=(x%10)+48;
		x/=10;
		place--;
	}
	place++;
	while(place<=4)
	{
		serial_putc(integer[place]);
		place++;
	}
}
//############################################

void pid_init()
{
	T3CONbits.TCS =0;
	T3CONbits.TCKPS =0;						//
	T3CONbits.TON =1;						//
	PR3 = 6000;       //150us
	IEC0bits.T3IE = 1;		
	IFS0bits.T3IF = 0;
}	
// Timer 3 ISR for Gyro sensor reading
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void)
{
	IFS0bits.T3IF = 0;
	samplesGyro[samplesIndexGyro] = read_adc(GYRO,GYRO_VCFG);
	sumGyro+=samplesGyro[samplesIndexGyro];
	
	//samplesAccl[samplesIndexAccl] = read_adc(ACCL,ACCL_VCFG);
	//sumAccl+=samplesAccl[samplesIndexAccl];
	
	//At this point we hav <SAMPLES> latest samples.
	//sum has summation of them.
	averaged_sample_gyro=(float)sumGyro/(float)SAMPLES_GYRO;
	//averaged_sample_accl=(float)sumAccl/(float)SAMPLES_ACCL;
	
	//Now prepare the buffer to accept new sample
	samplesIndexGyro = (samplesIndexGyro+1)%SAMPLES_GYRO;
	sumGyro -= samplesGyro[samplesIndexGyro];
	
	//samplesIndexAccl = (samplesIndexAccl+1)%SAMPLES_ACCL;
	//sumAccl -= samplesAccl[samplesIndexAccl];
	
	gyroErrorVoltage = (averaged_sample_gyro - gyroReferenceADC) * gyroVoltage / 4096.0;
	//acclErrorVoltage = (averaged_sample_accl - acclReferenceADC) * acclVoltage / 4096.0;
	
	//This line divides the voltage we found by the gyro's sensitivity
	gyroRate = gyroErrorVoltage/gyroSensitivity;
	
	
	//Ignore the gyro if our angular velocity does not meet our threshold
	if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold)
	{
	//serial_puts("T\n\r");
	//This line divides the value by 100 since we are running in a 10ms loop (1000ms/10ms)
	//gyroRate /= 1000;
	gyroAngleChange = gyroRate*150/1000000;
	currentAngle = (currentAngle + gyroAngleChange);// + 0.02 * prevGyroAngleChange;
	//prevGyroAngleChange = gyroAngleChange;
	}
	//gyroRate = gyroRate*5/1000;
	//currentAngle += gyroRate;
	//Keep our angle between 0-359.99 degrees
	if (currentAngle < 0.0)
	currentAngle += 360;
	else if (currentAngle >= 360.0)
	currentAngle -= 360;
	
	angleDeviation = (float) currentAngle - (float) STABLE_ANGLE;
	I += angleDeviation;
	if (I > 4096) I = 4096;
	if (I < -4096) I = -4096;
	
	float D;
	
	D = (float) (angleDeviation - Elast);
  	Elast = angleDeviation;
	
	if (angleDeviation > 0)
	{
		set_dir(FORWARD);
		vel = (Kp_forward * angleDeviation) + (Ki * I) + (Kd * D);
	}
	else if (angleDeviation < 0)
	{
		set_dir(BACKWARD);
		vel = (Kp_backward * angleDeviation) + (Ki * I) + (Kd * D);
	}
	else
	{
		set_dir(BRAKE);
		vel = 0;
	}
	// limit output
	if (vel > (MAX_COUNT))
	{
		vel = (MAX_COUNT);
	}	
	if (vel < -(MAX_COUNT))
	{
		vel = -(MAX_COUNT);
	}
	drive(abs(vel));			
	
}


void accl_init()
{
	//make AN4 digital
	AD1PCFGLbits.PCFG4 = 1;
	
	//init timer 4 for input capture
	T4CONbits.TCKPS = 1;
	T4CONbits.T32 = 0;
	T4CONbits.TCS = 0;
	T4CONbits.TON = 1;
	
	//init timer 5 for input capture
	T5CONbits.TCKPS = 1;
	T5CONbits.TCS = 0;
	T5CONbits.TON = 1;
	
	IPC5bits.INT1IP = 7;
	IPC7bits.INT2IP = 7;
	
	/*
	timer 5 gives x
	timer 4 gives y
	*/
		
	INTCON2bits.INT1EP = 0;       //rising edge
	INTCON2bits.INT2EP = 0;       //rising edge
	
	IEC1bits.INT1IE = 1;		  //enable external interrupt 1
	IEC1bits.INT2IE = 1;          //enable external interrupt 2
	
	RPINR0bits.INT1R = 4;        //Tie RP4 to INT1 input : Y axis
	RPINR1bits.INT2R = 2;        //Tie RP4 to INT2 input : X axis
}


int main()
{
	init_hw();
	adc_init();
	pwm_init();
	
	serial_init(BAUD_19200);
	serial_putc('A');
	serial_puts("Start!! ");
	serial_puts("\n\r");
	serial_putint(1234);
	unsigned long int tempAccl=0, tempGyro = 0;
	//GYRO TESTING
	int i;
	
	
	//waste 10 readings of accelerometer for stability
	__delay_ms(250);
	__delay_ms(250);
	accl_init();
	for(i = 0; i< 10; i++)
	{
		while(!(X_available == 1 && Y_available == 1));
		X_available = 0;
		Y_available = 0;
	}
	for(samplesIndexAccl=0;samplesIndexAccl<SAMPLES_ACCL-1;samplesIndexAccl++)
	{
		while(!(X_available == 1 && Y_available == 1));
		X_available = 0;
		Y_available = 0;
		X_out = TMR5_captured;
		Y_out = TMR4_captured;
		
		X_out_mapped = X_out-X_minus1g-(X_plus1g-X_minus1g)/2;
		Y_out_mapped = Y_out-Y_minus1g-(Y_plus1g-Y_minus1g)/2;
		
		X_g = (float)X_out_mapped/(float)((X_plus1g-X_minus1g)/2);
		Y_g = (float)Y_out_mapped/(float)((Y_plus1g-Y_minus1g)/2);
					
		if(Y_g>1.0)
		{
			Y_g=1.0;
		}
		if(Y_g<-1.0)
		{
			Y_g=-1.0;
		}	
		accl_angle = acos(Y_g)*(X_g>0?1:-1); 
		accl_angle_buf[samplesIndexAccl] = 92.2676 + accl_angle*180/3.14 ;//93.1774 + accl_angle*180/3.14 ;
	}		
	
	for(i = 0 ;i < 1000;i++)
	{
		tempGyro += read_adc(GYRO,GYRO_VCFG);
		//tempAccl += read_adc(ACCL,ACCL_VCFG);
	}
	gyroReferenceADC = tempGyro/1000;
	//acclReferenceADC = tempAccl/1000;
	
	__delay_ms(250);
	__delay_ms(250);
	
	for(samplesIndexGyro=0;samplesIndexGyro<SAMPLES_GYRO-1;samplesIndexGyro++)
	{
		samplesGyro[samplesIndexGyro] = read_adc(GYRO,GYRO_VCFG);
		sumGyro += samplesGyro[samplesIndexGyro];
	}
	pid_init();
	while(1)
	{	
			//samplesIndexAccl = (samplesIndexAccl+1)%SAMPLES_ACCL;
		//	serial_puts("\t");
			//serial_putint(accl_angle_buf[samplesIndexAccl]);
			//serial_puts("\n\r");
		
	
		//serial_putint(gyroRate*100);
		//serial_putint(angleDeviation);
		//serial_putint(currentAngle);
		//serial_putint(Y_out);
		//serial_putint(averaged_sample_accl - acclReferenceADC);
		//serial_puts("\n\r");
		
	}
	
	return 0;
}


void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt (void)
{
	
	if(INT1_interruptsetting == RISING)
	{
		
		//rising edge has occured.
		//reset TMR2.
		max_timer4_ticks = TMR4;
		TMR4 = 0;
		
		
		
		
		//set interrupt for falling edge.
		INTCON2bits.INT1EP = 1;       //falling edge
		INT1_interruptsetting = FALLING;
	}
	else if(INT1_interruptsetting == FALLING)
	{
	
		//falling edge has occured.
		//record TMR2.
		TMR4_captured = TMR4;
		
		
		
		
		//set interrupt for rising edge.
		INTCON2bits.INT1EP = 0;       //rising edge
		INT1_interruptsetting = RISING;
		Y_available = 1;
	}
	
	IFS1bits.INT1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt (void)
{
	if(INT2_interruptsetting == RISING)
	{
		//rising edge has occured.
		//reset TMR3.
		max_timer5_ticks = TMR5;
		TMR5 = 0;
		
		
		
		
		//set interrupt for falling edge.
		INTCON2bits.INT2EP = 1;       //falling edge
		INT2_interruptsetting = FALLING;
	}
	else if(INT2_interruptsetting == FALLING)
	{
		//falling edge has occured.
		//record TMR3.
		TMR5_captured = TMR5;
		
		
		
		
		//set interrupt for rising edge.
		INTCON2bits.INT2EP = 0;       //rising edge
		INT2_interruptsetting = RISING;
		X_available = 1;
		
		if(TMR5_captured>=18531 && TMR5_captured<=18535)
		{
			//accelerometer reading may be trustworthy. and robot is upright.
		//	currentAngle = 90.0;
			
			/*
			int index_of_last_gyro_value;
			if(samplesIndexGyro==0)
			{
				index_of_last_gyro_value = SAMPLES_GYRO-1;
			}
			else
			{
				index_of_last_gyro_value = samplesIndexGyro-1;
			}
			int last_gyro_sample = samplesGyro[index_of_last_gyro_value];
			int j;
			for(j=0;j<SAMPLES_GYRO;j++)
			{
				samplesGyro[j] = last_gyro_sample;
			}	
			*/
			
			
		}	
		
		
	}
	
	IFS1bits.INT2IF = 0;
}	
