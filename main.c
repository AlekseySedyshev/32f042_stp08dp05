#include "stm32f0xx.h"        // Device header

// PA3 - OE, PA4 - LATCH, PA5 - CLK, PA7 - MOSI

#define LATCH_OFF()					GPIOA->BRR 	|=1<<4
#define LATCH_ON()					GPIOA->BSRR |=1<<4

#define QNT									2	//Quantity of LED Segments

#define SPI1_DR_8bit 			*((__IO uint8_t *)&SPI1->DR)		// Limit for spi bus 8 bit

uint8_t i,tic=0,flag=0;
uint16_t counter;

unsigned char TX_BUF[QNT];

uint16_t TimingDelay,led_count,mss;

void TimingDelayDec(void) 																						{
 if (TimingDelay			!=0x00) TimingDelay--;
 if (!led_count) {led_count=500; GPIOB->ODR ^=1;}
 if (!mss) {mss=200;tic=1;}
 led_count--;mss--;
 }

void TIM17_IRQHandler(void)																						{
		if (TIM17->SR & TIM_SR_UIF) {
					TimingDelayDec();
  				TIM17->SR &=(~TIM_SR_UIF);
		}
}	
void delay_ms (uint16_t DelTime) 																			{
    TimingDelay=DelTime;
  while(TimingDelay!= 0x00);
}



uint8_t SPI_Write(uint8_t value) 																			{//WriteSingle

while (!(SPI1->SR & SPI_SR_TXE)){};	 
SPI1_DR_8bit = value;
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; 
SPI1_DR_8bit;	

return 1;	
}


void 		SPI_WriteBurst( uint8_t *buff, uint8_t size )									{//WriteBurst

	uint8_t j_;
for( j_ = 0; j_ < size; j_ ++ )
 {
  while (!(SPI1->SR & SPI_SR_TXE)){};	
	SPI1_DR_8bit = buff[j_]; 
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){};
	SPI1_DR_8bit;	
 }
}


//------------------------------LED ------------------------------
void LED_Init(void)																										{//CC1101_Init
uint8_t j,tst;
tst=0xff;
for (i=0;i<9;i++)
{	
	for (j=0;j<QNT;j++)
		{
			SPI_Write(tst);

		}
tst>>=1;//<<i;		
LATCH_ON();
LATCH_OFF();
delay_ms(100);	
}

}



static const uint8_t seg_font[] = 																		{//Font of 7 seg. LED
	0x3f,	//0		
	0x06,	//1
	0x5b,	//2
	0x4f,	//3
	0x66, //4
	0x6d,	//5
	0x7d, //6
	0x07, //7
	0x7f, //8
	0x6f, //9
	0x77, //A		10
	0x7c, //b		11
	0x39, //C		12
	0x5E, //d		13
	0x79, //E		14
	0x71, //F		15
	0x63, //^0	16
	0x40, //-		17
	0x08, //_		18
	0x80 //.		19
};


void LED_Char(char symb)															        				{// Print Symbol
  SPI_Write(seg_font[symb]);
}

void LCD_PrintStr(char *s)																	    			{// Print String
	while (*s)
        {
          LED_Char(*s);
          s++;
        }
LATCH_ON();
delay_ms(1);				
LATCH_OFF();				
      }
void LCD_PrintDec(long value,uint8_t msb) 														{// Print Dec
	
	char i=1,d=0;
	unsigned char text[10];
	do 
  { 
    if (value >=10)  {
				d = value % 10; 																			
				text[i] = d ; 																			
				value /= 10; 																					
			}
		else 
			{	text[i] = value;
				value=0;
			}
 		i++;
  }
	while(value); 
	i--;			

if(!msb)										{
//----------insert space------------------	
	if (i<QNT) {
d=QNT-i;
	do
	{	SPI_Write(0);
		d--;
	}
	while(d);
}
	//-------------------------
	do
	{	LED_Char(text[i]);
		i--;
	}
	while(i);
}
else												{
for(msb=1;msb<=i;msb++)

{	LED_Char(text[msb]);}

if (i<QNT) {
d=QNT-i;
	do
	{	SPI_Write(0);
		d--;
	}
	while(d);
}

}
LATCH_ON();
//delay_ms(1);
LATCH_OFF();	
}
			
void LCD_PrintHex(long value,uint8_t msb) 														{// Print Hex
	
	char i=1,d=0;
	unsigned char text[10];
	do 
  { 
    if (value >=0x10)  {
				d = value % 0x10; 																				
				text[i] = d; 																			
				value /= 0x10; 																						
			}
		else 
			{	
				text[i] = value;
				value=0;
			}
 		i++;
  }
	while(value); 
	i--;			
	
	if(!msb)										{
//------------insert space--------------
		if (i<QNT) {
d=QNT-i;
	do
	{	SPI_Write(0);
		d--;
	}
	while(d);
}
//--------------------------------------
		do
	{	LED_Char(text[i]);
		i--;
	}
	while(i);
}
else												{
for(msb=1;msb<=i;msb++)

{	LED_Char(text[msb]);}

if (i<QNT) {
d=QNT-i;
	do
	{	SPI_Write(0);
		d--;
	}
	while(d);
}

}
LATCH_ON();
//delay_ms(1);
LATCH_OFF();
}

/*	
void Init_DMA (void)
{
 
  RCC->AHBENR |= RCC_AHBENR_DMA1EN; 
  SPI1->CR2 |= SPI_CR2_RXDMAEN; 
	SPI1->CR2 |= SPI_CR2_TXDMAEN;
  DMA1_Channel1->CPAR = (uint32_t) (&(SPI1_DR_8bit)); 
  DMA1_Channel1->CMAR = (uint32_t)(SPI_array); 
  DMA1_Channel1->CNDTR = 2; 
  DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 \
                      | DMA_CCR_TEIE | DMA_CCR_TCIE ;  
  DMA1_Channel1->CCR |= DMA_CCR_EN; 
  
   NVIC_EnableIRQ(DMA1_Channel2_3_IRQn); 
  NVIC_SetPriority(DMA1_Channel2_3_IRQn,0); 
}
void DMA1_Channel2_3_IRQHandler(void)
{

}
*/

	void initial (void)																									{
//---------------TIM17------------------
  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    																			//HSI 8 MHz - 1 msek
  TIM17->PSC = 8000-1;
  TIM17->ARR = 1;
  TIM17->CR1 |= TIM_CR1_ARPE | TIM_CR1_DIR | TIM_CR1_CEN; 											// 
	TIM17->DIER |=TIM_DIER_UIE;
	NVIC_EnableIRQ (TIM17_IRQn);
	NVIC_SetPriority(TIM17_IRQn,0x05);	

//-------------------GPIOB-Blinking Led		
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN; 																					//
	GPIOB->MODER |= GPIO_MODER_MODER0_0;																					//Pb0-Out 
//----------------OE pin PWM--------------------------
	/*GPIOA PIN 3 - Alternative mode */ 

	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN; 								//
	GPIOA->MODER |= GPIO_MODER_MODER3_1; 								// alternative
	GPIOA->AFR[0] |= 0x02 << GPIO_AFRL_AFRL3_Pos;  			//Pa3 - Alternative AFR2 (TIM2_CH4)

	RCC->APB1ENR |=RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 1;  																		// 8MHZ - in
	TIM2->ARR = 20; 	 
	TIM2->CCR4 = 0;  
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;
	TIM2->CCER |= TIM_CCER_CC4E |TIM_CCER_CC4P; 
 	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->EGR |= TIM_EGR_UG;		
	
//------------------------SPI-----------------------------------
	RCC->AHBENR 		|=RCC_AHBENR_GPIOAEN;
	GPIOA->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOA->MODER 		|=GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; 	//Pa4 - out,Pa5..7 - Alt_mode 
	GPIOA->AFR[0] 	|=(0<<GPIO_AFRL_AFRL7_Pos) |(0<<GPIO_AFRL_AFRL6_Pos) | (0<<GPIO_AFRL_AFRL5_Pos);  // SPI - Alternative
	GPIOA->MODER 		|=GPIO_MODER_MODER4_0;
	
	RCC->APB2ENR |=RCC_APB2ENR_SPI1EN;
	SPI1->CR1 |=SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | (2<<SPI_CR1_BR_Pos);  // if HSI8 - SpiSpeed (BR=2) - 1MHz
	SPI1->CR2 |=SPI_CR2_FRXTH;
	SPI1->CR1 |=SPI_CR1_SPE;

	
} 

int main(void)
{
initial();
	TIM2->CCR4 = 10; 	
LED_Init();
counter=0;
tic=0;

	

//-----------------------------initial data----------------------------------

while (1)  /* Main loop */
{
	
	if (tic && !flag) {
///----------------Print DEC-------------
	if (counter%20) {TIM2->CCR4 = counter%20;}
	else {TIM2->CCR4=20;}
	LCD_PrintDec(counter,1);
	counter++;
	if(counter > 99) {counter =0;flag=1;}
	tic=0;
	}
	
	if (tic && flag) {
///----------------Print Hex-------------
	TIM2->CCR4=10;				// 50% of bright 
	LCD_PrintHex(counter,1);
	counter++;
	if(counter > 255) {counter =0;flag=0;}
	tic=0;
	}	
	
	

} // end - main loop 
} // end - Main  
	
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ while (1)  {  } }
#endif
