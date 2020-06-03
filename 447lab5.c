//----------------------------------------------------------------
// CECS 447 Final Project 
// Project name: Functional Bluetooth Control Car
// Author: Nguyen Doan
// Date: 5/4/2020
//----------------------------------------------------------------
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <string.h>
#include "PWM.h"
#include "ST7735.h"

#define PB0      (*((volatile uint32_t *)0x40005004))
#define PB1      (*((volatile uint32_t *)0x40005008))
#define PC4      (*((volatile uint32_t *)0x40006040))
#define PC5      (*((volatile uint32_t *)0x40006080))

#define redLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define blueLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define greenLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
	
#define C5 119
#define D5 106
#define E5 94
#define F5 89
#define G5 79
#define A5 71
#define B5 63
#define C6 59
#define C0 0


#define sineLength  64
#define onTime 400000


const char
     RED = 0x02,
     GREEN = 0x08,
     BLUE = 0x04,
     OFF = 0x00;
char color = RED;
char rxChar[10];
char txChar;
uint8_t rx_flag = 0;
uint8_t  timeron = 60;
uint16_t count = 0;
char countstr[10];
uint8_t i = 0;
unsigned char * waveType;
unsigned int frequency = C5;
uint16_t data_length = sineLength; 
uint16_t k = 0;
volatile static uint32_t adcResult = 0;
volatile static int temp = 0;

unsigned char sineWaveLookup[64] = {32,35,38,41,44,47,50,52,55,57,59,60,62,63,63,64,
64,64,63,63,62,60,59,57,55,52,50,47,44,41,38,35,
32,29,26,23,20,17,14,12,9,7,5,4,2,1,1,0,
0,0,1,1,2,4,5,7,9,12,14,17,20,23,26,29};

typedef struct {
	int freq; 
	int duration;
} Song;

Song song[] = {{C5,20}, {C5,20}, {G5,20}, {G5,20}, {A5,20}, {A5,20}, {G5,40}, {F5,20}, 
							 {F5,20}, {E5,20}, {E5,20}, {D5,20}, {D5,20}, {C5,40}, {G5,20}, {G5,20},
							 {F5,20}, {F5,20}, {E5,20}, {E5,20}, {D5,40}, {G5,20}, {G5,20}, {F5,20},
							 {F5,20}, {E5,20}, {E5,20}, {D5,40}, {C5,20}, {C5,20}, {G5,20}, {G5,20}, 
						   {A5,20}, {A5,20}, {G5,40}, {F5,20}, {F5,20}, {E5,20}, {E5,20}, {D5,20}, 
							 {D5,20}, {C5,40}, {C0,40}	 
							};
unsigned int note = 0;
unsigned int newNote = 1;
unsigned char zeroWave[64] = {0};


//------------Forward----------------------------------
// Make the car go forware
// Input: none
// Output: none
//---------------------------------------------------
void Forward(void){ 
		PWM0A_Duty(39000);
		PWM0B_Duty(39000);
		PB0 = 1;
		PB1 = 0;
		PC5 = (1<<5);
		PC4 = 0;
}


//------------Backward-------------------------------
// Make the car go backware
// Input: none
// Output: none
//---------------------------------------------------
void Backward(void){
		PWM0A_Duty(39000);
		PWM0B_Duty(39000);
		PB0 = 0;
		PB1 = (1<<1);
		PC5 = 0;
		PC4 = (1<<4);
}


//------------Left----------------------------------
// Make the car turn left
// Input: none
// Output: none
//---------------------------------------------------
void Left(void){
		PWM0A_Duty(38000);
		PWM0B_Duty(0);
		PB0 = 1;
		PB1 = 0;
}


//------------Right----------------------------------
// Make the car turn right
// Input: none
// Output: none
//---------------------------------------------------
void Right(void){
		PWM0A_Duty(0);
		PWM0B_Duty(38000);
		PC5 = (1<<5);
		PC4 = 0;
}


//------------Brake----------------------------------
// Make the car stop
// Input: none
// Output: none
//---------------------------------------------------
void Brake(void){
		PWM0A_Duty(0);
		PWM0B_Duty(0);
		PB0 = 0;
		PB1 = 0;
		PC5 = 0;
		PC4 = 0;
}


//------------PortB_Init-------------------------------------
// Subroutine to initialize port B pins for input and output
// PB0,1 are ouptut
// Inputs: None
// Outputs: None
//-----------------------------------------------------------
void PortB_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000002;     	// B clock
  delay = SYSCTL_RCGC2_R;           	// delay   
  GPIO_PORTB_CR_R |= 0x03;          	// allow changes to PB0,1       
  GPIO_PORTB_AMSEL_R &= ~0x03;      	// disable analog function
  GPIO_PORTB_PCTL_R &= ~0x000000FF; 	// GPIO clear bit PCTL  
  GPIO_PORTB_DIR_R |= 0x03;        		// PB0,1 output   
  GPIO_PORTB_AFSEL_R &= ~0x00000011;	// no alternate function
  GPIO_PORTB_PUR_R &= ~0x00000011;  	// disable all pullup resistors        
  GPIO_PORTB_DEN_R |= 0x03;         	// enable digital pins PB0,1    
}


//------------PortC_Init-------------------------------------
// Subroutine to initialize port C pins for input and output
// PC4,5 are output
// Inputs: None
// Outputs: None
//-----------------------------------------------------------
void PortC_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000004;     	// C clock
  delay = SYSCTL_RCGC2_R;           	// delay   
  GPIO_PORTC_CR_R |= 0x30;          	// allow changes to PC4,5       
  GPIO_PORTC_AMSEL_R &= ~0x30;      	// disable analog function
  GPIO_PORTC_PCTL_R &= ~0x00FF0000; 	// GPIO clear bit PCTL  
  GPIO_PORTC_DIR_R |= 0x30;        		// PC4,5 output   
  GPIO_PORTC_AFSEL_R &= ~0x00110000;	// no alternate function
  GPIO_PORTC_PUR_R &= ~0x00110000;  	// disable all pullup resistors        
  GPIO_PORTC_DEN_R |= 0x30;         	// enable digital pins PC4,5    
}


//------------writeCharToUart0----------------------------------
// Write char to UART0
// Input: none
// Output: none
//--------------------------------------------------------------
void writeCharToUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF); //wait till Transmitter is not full
	UART0_DR_R = c; //write to UART0
}


//------------writeStringToUart0----------------------------------
// Write string to UART0
// Input: none
// Output: none
//----------------------------------------------------------------
void writeStringToUart0(char* str)   //write a string to Uart0
{
	int i;
    for (i = 0; i < strlen(str); i++)
    	writeCharToUart0(str[i]);
}


//------------writeCharToUart3----------------------------------
// Write char to UART3
// Input: none
// Output: none
//--------------------------------------------------------------
void writeCharToUart3(char c)
{
	while (UART3_FR_R & UART_FR_TXFF); //wait till Transmitter is not full
	UART3_DR_R = c; //write to UART3
}


//------------writeStringToUart3----------------------------------
// Write String to UART 3
// Input: none
// Output: none
//----------------------------------------------------------------
void writeStringToUart3(char* str)
{
	int i;
    for (i = 0; i < strlen(str); i++)
    	writeCharToUart3(str[i]);
}


//------------UART2_Handler----------------------------------
// UART2 Handler
// Input: none
// Output: none
//-----------------------------------------------------------
void UART2_Handler ()//this interrupt routine is for receiving data from bluetooth
{		
		rx_flag = 1;
    rxChar[i] = UART2_DR_R;
		writeCharToUart3(rxChar[i]);
  	i++;
		if(rxChar[i-1]==13) // check if rxChar is CR
		{
		rxChar[i-1]='\0';
		if(strcmp(rxChar,"red")==0)
					GPIO_PORTF_DATA_R = RED;
		if(strcmp(rxChar,"blue")==0)
					GPIO_PORTF_DATA_R = BLUE;
		if(strcmp(rxChar,"green")==0)
					GPIO_PORTF_DATA_R = GREEN;
		i=0;
		}
    UART2_ICR_R=UART_ICR_RXIC;//clear interrupt
}


//------------UART3_Handler----------------------------------
// UART3 Handler
// Input: none
// Output: none
//-----------------------------------------------------------
void UART3_Handler() //this interrupt routine is to transmit data over bluetooth
{
	txChar = UART3_DR_R;
    UART3_ICR_R=UART_ICR_TXIC; //clear interrupt
}

//------------setUp----------------------------------
// Initialize UART2,3 to use HC-05
// Input: none
// Output: none
//---------------------------------------------------
void setUp()
{

	    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);//40Mhz clock


			SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOC|SYSCTL_RCGC2_GPIOA|SYSCTL_RCGC2_GPIOB|SYSCTL_RCGC2_GPIOE|SYSCTL_RCGC2_GPIOD;
			GPIO_PORTF_DIR_R |= 0x0E;
			GPIO_PORTF_DEN_R |= 0x0E;

	    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; //Clock for UART0
	    GPIO_PORTA_DEN_R |= 3;
	    GPIO_PORTA_AFSEL_R |= 3;
	    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

	    UART0_CTL_R = 0;
	    UART0_CC_R = UART_CC_CS_SYSCLK;
	    UART0_IBRD_R = 21;
	    UART0_FBRD_R = 45;    // set baud rate as 1152000
	    UART0_LCRH_R = UART_LCRH_WLEN_8;
	    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;

    	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R3;      //Clock for UART3
      GPIO_PORTC_DEN_R |= 0xC0;
    	GPIO_PORTC_AFSEL_R |= 0xC0;
      GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC7_U3TX; //PC7 is transmit i.e RXD in bluetooth module

       // Configure UART3 to 9600 baud, 8N1 format
      UART3_CTL_R = 0;
    	UART3_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
      UART3_IBRD_R = 260;                               // r = 40 MHz / (Nx9600Hz), set floor(r)=260, where N=16
      UART3_FBRD_R = 27;                               // round(fract(r)*64)=27
      UART3_LCRH_R = UART_LCRH_WLEN_8; // configure for 8N1 w/o FIFO
      UART3_CTL_R = UART_CTL_TXE | UART_CTL_UARTEN; // enable TX and module
    	UART3_IM_R = UART_IM_TXIM;                       // turn-on TX interrupt
    	NVIC_EN1_R = 1<<27;//enable interrupt


      SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R2;            // turn-on UART2, leave other uarts in same status
    	GPIO_PORTD_DEN_R |= 0x40;
    	GPIO_PORTD_AFSEL_R |= 0x40;
    	GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD6_U2RX;//PD6 is recieve. i.e TXD in bluetooth module

        // Configure UART2 to 9600 baud, 8N1 format
    	UART2_CTL_R = 0;
      UART2_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    	UART2_IBRD_R = 260;                               // r = 40 MHz / (Nx9600Hz), set floor(r)=260, where N=16
    	UART2_FBRD_R = 27;                               // round(fract(r)*64)=27
    	UART2_LCRH_R = UART_LCRH_WLEN_8; // configure for 8N1 w/o FIFO
    	UART2_CTL_R = UART_CTL_RXE | UART_CTL_UARTEN; // enable RX, and module
    	UART2_IM_R = UART_IM_RXIM;                       // turn-on RX interrupt
    	NVIC_EN1_R = 1<<1;//enable interrupt

//    	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;
//      TIMER0_CTL_R &= ~TIMER_CTL_TAEN;//reset timer1
//    	TIMER0_CFG_R =TIMER_CFG_32_BIT_TIMER;
//    	TIMER0_TAMR_R =TIMER_TAMR_TAMR_PERIOD;
//      TIMER0_TAILR_R=4000000;//40000000/4000000=10Hz 100ms configure
//    	TIMER0_IMR_R= TIMER_IMR_TATOIM; // enable timer0 interrupt for every 100ms
//    	TIMER0_CTL_R  |= 0x00000001;
//    	NVIC_EN0_R|=1<<19; //enable interrupt

}


//---------------------------PortF_Init------------------------------
// Subroutine to initialize port F pins for input and output
// PF4 and PF0 are input SW1 and SW2 respectively
// PF3,PF2,PF1 are outputs to the LED
// Inputs: None
// Outputs: None
// Notes: These five pins are connected to hardware on the LaunchPad
//-------------------------------------------------------------------
void PortF_Init(void){ volatile unsigned long delay;
			SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
			delay = SYSCTL_RCGC2_R;           // delay   
			GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
			GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
			GPIO_PORTF_AMSEL_R &= ~0x1F;      // 3) disable analog function
			GPIO_PORTF_PCTL_R &= ~0x000FFFFF; // 4) GPIO clear bit PCTL  
			GPIO_PORTF_DIR_R |= 0x0E;         // 5) PF4,PF0 input, PF3,PF2,PF1 output   
			GPIO_PORTF_AFSEL_R &= ~0x1F;      // 6) no alternate function
			GPIO_PORTF_PUR_R |= 0x11;         // enable pullup resistors on PF4,PF0       
			GPIO_PORTF_DEN_R |= 0x1F;         // 7) enable digital pins PF4-PF0   
//			GPIO_PORTF_IS_R &= ~0x11;					// PF4,0 are edge sensitive
//			GPIO_PORTF_IBE_R &= ~0x11;				// PF4,0 are not both edge sensitive
//			GPIO_PORTF_IEV_R &= ~0x11;				// PF4,0 are negedge triggered
//			GPIO_PORTF_ICR_R = 0x11;					// clear flag on PF4,0
//			GPIO_PORTF_IM_R |= 0x11;					// interrupt on PF4,0
			
			
//			NVIC_PRI17_R = ( NVIC_PRI17_R & 0xFF00FFFF ) | 0x00A00000;	// Set interrupt priority to 5
//			NVIC_EN0_R = 1 << 30;	// enable IRQ30	
}


//------------DAC_init------------------------------
// Initialize SSI3 to control the DAC
// Input: none
// Output: none
//---------------------------------------------------
void DAC_init(void)
{
    SYSCTL_RCGCGPIO_R |= 0x08;   // enable clock to GPIOD 
    SYSCTL_RCGCSSI_R |= 0x08;    // enable clock to SSI3

		// PORTB 3, 1, 0 for SSI2: PD0 = Clk, PD3 = MOSI, PD1 = Fss
    GPIO_PORTD_AFSEL_R |= 0x0B;       // PORTD 3, 1, 0 for SSI3
    GPIO_PORTD_PCTL_R &= ~0x0000F0FF; // PORTD 3, 1, 0 for SSI3
    GPIO_PORTD_PCTL_R |= 0x00001011;
    GPIO_PORTD_DEN_R |= 0x0B;         // PORTD 3, 1, 0 as digital pins

    // initialize SSI2 
    SSI3_CR1_R = 0;      // make it master
    SSI3_CC_R = 0;       // use system clock
    SSI3_CPSR_R = 10;    // clock prescaler divide by 16 gets 1 MHz clock
    SSI3_CR0_R = 0xF;    // clock rate div by 1, phase/polarity 0 0, mode freescale, data size 16; see ds page 969
    SSI3_CR1_R = 2;      // enable SSI3
}


//------------DAC_write------------------------------
// Write a value to DAC through SSI3
// Input: value
// Output: none
//---------------------------------------------------
void DAC_write(uint8_t value){
    const uint16_t UNITYGAIN = 1<<13;
    const uint16_t NOTSHUTDOWN = 1<<12;
    uint16_t command = 0;
    command = (value << 4) | UNITYGAIN | NOTSHUTDOWN;
	
	
    SSI3_DR_R = command;    // send command
	
    while (SSI3_SR_R & 0x10) ;   // wait for transmit done
}


//------------timer0AInit------------------------
// Timer0A Inialization
// Input: none
// Output: none
//---------------------------------------------------
void timer0AInit(void){
			SYSCTL_RCGCTIMER_R |= 1;		// enable Timer Block 0
			TIMER0_CTL_R = 0;						// disable Timer before init
			TIMER0_CFG_R = 0x04;				// 16-bit mode
			TIMER0_TAMR_R = 0x02;				// periodic mode, down-counter
			TIMER0_TAPR_R = 10; //20;					// 4MHz = 80M/20 prescalar setting
			TIMER0_TAILR_R = frequency;	// interval load value register
			TIMER0_ICR_R = 1;						// clear Timer0A timeout flag
			TIMER0_IMR_R |= 1;					// enable Timer0A timeout interrupt
			TIMER0_CTL_R |= 1;					// enable Timer0A
			NVIC_EN0_R |= 1 << 19;			// enable IRQ19
}


//------------Timer0A_Handler------------------------
// Timer0A Interrupt Handler
// Input: none
// Output: none
//---------------------------------------------------
void Timer0A_Handler(void){
	volatile int32_t readback;	
	if( TIMER0_MIS_R & 1 ){
		DAC_write(waveType[k]);
		if(k == data_length){
			k = 0;
		}else{
			k++;
		}
		TIMER0_ICR_R = 1;							// clear Timer0A timeout flag
		readback = TIMER0_ICR_R;
	}
	else{
		TIMER0_ICR_R = 1;							// clear Timer0A timeout flag
		readback = TIMER0_ICR_R;
	}
}


//------------timer1BInit----------------------------
// Timer1B Initalization
// Input: none
// Output: none
//---------------------------------------------------
void timer1BInit(void){
     SYSCTL_RCGCTIMER_R |= 2;     // enable clock to timer block 1
     TIMER1_CTL_R = 0;            // disable timer1 while initializing
     TIMER1_CFG_R = 0x04;         // 16-bit operation
     TIMER1_TBMR_R = 0x01;        // one-shot, countdown
     TIMER1_TBPR_R = 100;//200;         // 4MHz = 80M/200 prescalar setting
     TIMER1_TBILR_R = onTime;     // interval load value register
     TIMER1_ICR_R = 0x100;        // clear Timer1B timeout flag
     TIMER1_IMR_R |= 0x100;       // enable Timer1B timeout interrupt
     TIMER1_CTL_R |= 0x100;       // enable Timer1B
     NVIC_EN0_R |= 1 << 22;       // enable IRQ22
}

//------------Timer1B_Handler------------------------
// Timer1B Interrupt Handler
// Input: none
// Output: none
//---------------------------------------------------
void Timer1B_Handler(void){
     volatile int32_t readback;
 
     if( TIMER1_MIS_R & ( 1 << 8) ){    // ds pdf 752
				 count++;
				 timer1BInit();
         TIMER1_ICR_R = 1 << 8;        	// clear Timer1B timeout flag
         readback = TIMER1_ICR_R;
     }
     else{
         TIMER1_ICR_R = 1 << 8;        	// clear Timer1B timeout flag
         readback = TIMER1_ICR_R;
     }
 }

 
//------------ADC1_InitSWTriggerSeq3_Ch1------------
// ADC1 Intialization
// Input: none
// Output: none
//---------------------------------------------------
 void ADC1_InitSWTriggerSeq3_Ch1(void){ volatile uint32_t delay;
                                  // 1) activate clock for Port E
  SYSCTL_RCGCGPIO_R |= (1<<4);		
  delay = SYSCTL_RCGCGPIO_R;      //   allow time for clock to stabilize
  delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTE_DIR_R &= ~(1<<1);      // 2) make PE1 input
  GPIO_PORTE_AFSEL_R |= (1<<1);     // 3) enable alternate function on PE1
  GPIO_PORTE_DEN_R &= ~(1<<1);      // 4) disable digital I/O on PE1
                                  // 4a) configure PE1 as ?? (skip this line because PCTL is for digital only)
  //GPIO_PORTE_PCTL_R = GPIO_PORTF_PCTL_R&0xFFFFFF0F;
  GPIO_PORTE_AMSEL_R |= (1<<1);     // 5) enable analog functionality on PE1
  SYSCTL_RCGC0_R |= 0x00020000;   // 6) activate ADC1 (legacy code)
  SYSCTL_RCGCADC_R = (1<<1);		  // 6) activate ADC1 (actually doesn't work)
  SYSCTL_RCGC0_R &= ~0x00000300;  // 7) configure for 125K (legacy code)
  //ADC1_PC_R &= ~0xF;              // 7) clear max sample rate field
  //ADC1_PC_R |= 0x1;               //    configure for 125K samples/sec
  ADC1_SSPRI_R = 0x3210;          // 8) Sequencer 3 is highest priority
  ADC1_ACTSS_R &= ~(1<<3);        // 9) disable sample sequencer 3
  ADC1_EMUX_R &= ~(0xF<<12);         // 10) seq3 is continuously sample
  //ADC1_SSMUX3_R &= ~0x000F;       // 11) clear SS3 field
  ADC1_SSMUX3_R = 2;             //    set channel
  ADC1_SSCTL3_R = 0x6;         // 12) no TS0 D0, yes IE0 END0
  ADC1_IM_R &= ~(1<<3);           // 13) disable SS3 interrupts
	ADC1_ACTSS_R |= (1<<3);         // 14) enable sample sequencer 3
	ADC1_ISC_R = (1<<3);
//	NVIC_EN1_R |= 1 << 19;
}


//------------ADC0_InSeq3-----------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: 12-bit result of ADC conversion
//----------------------------------------
uint32_t ADC1_InSeq3(void){  uint32_t result;
  ADC1_PSSI_R = 0x0008;            // 1) initiate SS3
  while((ADC1_RIS_R&0x08)==0){};   // 2) wait for conversion done
    // if you have an A0-A3 revision number, you need to add an 8 usec wait here
  result = ADC1_SSFIFO3_R&0xFFF;   // 3) read result
  ADC1_ISC_R = 0x0008;             // 4) acknowledge completion
  return result;
}


//------------ADC1Seq3-Handler------------
// Interrrupt hander ADC1 conversion
// Input: none
// Output: none
//----------------------------------------
void ADC1Seq3_Handler(void)
{
	ADC1_PSSI_R = 0x0008;            // 1) initiate SS3
	while((ADC1_RIS_R&0x08)==0){};   // 2) wait for conversion done
	adcResult = ADC1_SSFIFO3_R&0xFFF; 
	ADC1_ISC_R = (1<<3);
}
 
void EnableInterrupts(void);

//------------main()----------------------
// Main loop
// Input: none
// Output: none
//----------------------------------------
int main(void)
{
		PortF_Init();
    setUp();
		ADC1_InitSWTriggerSeq3_Ch1();
		PortB_Init();
		PortC_Init();
		PortF_Init();
		PWM0A_Init(40000, 39000);        
		PWM0B_Init(40000, 39000);        
		DAC_init();
		EnableInterrupts(); 
		ST7735_InitR(INITR_REDTAB);
		ST7735_SetRotation(3);
		ST7735_FillScreen(0);  
		adcResult = ADC1_InSeq3();
		temp = adcResult*16/75-20;
		ST7735_OutString("Good Morning\r");
		ST7735_OutString("Temperature:" );
		ST7735_OutUDec(temp);
		ST7735_OutString("F \r" );
		Brake();
		waveType = zeroWave;
    while(1){
		  if (rx_flag == 1){
				  switch (rxChar[i-1]){
						case 'w': 
							Forward();
							waveType = zeroWave; 
							break;
						case 's': 
							waveType = zeroWave; 
							Backward();
							break;
						case 'd': 
							waveType = zeroWave; 
							Right();
							break; 
						case 'a': 
							waveType = zeroWave; 
							Left();
							break;
						case 'b':
							Brake();
							waveType = sineWaveLookup;
							break;
						case 't': 
							adcResult = ADC1_InSeq3();
							temp = adcResult*16/75-20;
							ST7735_SetCursor(0,1);	
							ST7735_OutString("Temperature:" );
							ST7735_OutUDec(temp);
							ST7735_OutString("F \r" );
							break;
						default: Brake();
					}
					rx_flag = 0;
			}
			if (newNote == 1){
				frequency = song[note].freq;
				data_length = sineLength;
				timer0AInit();	
				//waveType = sineWaveLookup;
				count = 0;
				timer1BInit();
			}
			newNote = 0;
			if (count == song[note].duration){
				//waveType = zeroWave;
				note += 1;
				newNote = 1; 
			}
			if (note == 42){
				note = 0; 
			}
		}
}

