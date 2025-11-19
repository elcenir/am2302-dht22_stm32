/*
 * this software is for running AM2302 with stm32f103c8 without delay by using timer
 * PB4 and PB6 and sensor data pin are connected together and pulled up with 1K resistor
 *
 *
 * Attention :
 * this file contains the IRQ_Handler function for timer 4 update interrupt
 * this software uses these: TIM4 , TIM3 , DMA_CH1 , DMA_CH4 , PB6 and PB4
 * you can add these files to your project with HAL .
 * you might have problem running this code on clone blue pills
 * because their timer GPIO's can't be in OD mode
 *
 *
 * launch :
 * 1.include am.h in main.c
 * 2.call am_init() in main function
 * 3.temperature and pressure are going to be stored in global variables named RH & TMP
 *
 *
 * Licenses : CC BY-NC
 * elcen.ir
 *
 */


#include "am.h"
#include "stdio.h"

float am_RH,am_TMP;

static am_typedef am_data;
static uint8_t am_loop;
static uint8_t cs;
static uint32_t am_damage;
static uint32_t am_total;
static uint16_t CCR1[43],CCR2[43],DIF[43];


void am_init(void){

	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	GPIOB->CRL &= ~( GPIO_CRL_CNF6 | GPIO_CRL_MODE6);

	GPIOB->CRL |= GPIO_CRL_CNF6_0;


	//TIM4 for receiving bits
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	TIM4->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_CKD | TIM_CR1_DIR);
	TIM4->CR1 |= TIM_CR1_OPM;

	TIM4->ARR = (uint16_t)0xFFFF;
	TIM4->PSC = (uint16_t)71;

	//TIM4 slave of TIM3
	TIM4->SMCR &= ~TIM_SMCR_TS;
	TIM4->SMCR |= TIM_SMCR_TS_1;

	//TIM4 ch1 & ch2 for receiving pulses
	TIM4->SMCR &= ~TIM_SMCR_SMS;
	TIM4->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2);
	TIM4->CCMR1 &= ~( TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);

	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1;

	TIM4->CCER |= TIM_CCER_CC1P;
	TIM4->CCER &= ~TIM_CCER_CC2P;

	TIM4->CCER |= ( TIM_CCER_CC1E | TIM_CCER_CC2E );

	TIM4->EGR = TIM_EGR_UG;

	//enabling tim4 update interrupt for the end of sampling
	TIM4->SR = ~( TIM_SR_UIF );
	TIM4->DIER |= ( TIM_DIER_UIE );

	TIM4->DIER |= (TIM_DIER_CC1DE | TIM_DIER_CC2DE);

	uint32_t PG = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(TIM4_IRQn,NVIC_EncodePriority(PG, 2, 0));

	NVIC_EnableIRQ(TIM4_IRQn);


	//TIM3 ch1 output in open-drain PB4 to pull data bus low at start of sampling every 2s

	AFIO->MAPR &= ~AFIO_MAPR_TIM3_REMAP;
	AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_1;

	GPIOB->CRL &= ~GPIO_CRL_MODE4;
	GPIOB->CRL |= (GPIO_CRL_CNF4| GPIO_CRL_MODE4_1);

	//TIM3 for pulling data pin low and start operation every 2s
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	TIM3->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_CKD | TIM_CR1_DIR);

	TIM3->ARR = (uint16_t)29999;
	TIM3->PSC = (uint16_t)7199;

	TIM3->CCR1 = (uint16_t)50;

	//TIM3 Master
	TIM3->CR2 &= ~TIM_CR2_MMS;
	TIM3->CR2 |= TIM_CR2_MMS_1;

	//TIM3 ch1 PWM
	TIM3->CCMR1 |= TIM_CCMR1_CC1S;
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE;
	TIM3->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM3->CCMR1 |= TIM_CCMR1_OC1M;
	TIM3->CCER &= ~TIM_CCER_CC1P;
	TIM3->CCER |= TIM_CCER_CC1E ;


	TIM3->DIER &= ~TIM_DIER_UIE;
	TIM3->EGR = TIM_EGR_UG;
	TIM3->SR = ~(TIM_SR_UIF);

	//DMA1_ch1_tim4_ccr1
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	DMA1_Channel1->CPAR = (uint32_t)&(TIM4->CCR1);
	DMA1_Channel1->CMAR = (uint32_t)CCR1;
	DMA1_Channel1->CCR = 0x0;
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_MINC;
	DMA1_Channel1->CNDTR = 44;

	//DMA1_ch4_tim4_ccr2

	DMA1_Channel4->CPAR = (uint32_t)&(TIM4->CCR2);
	DMA1_Channel4->CMAR = (uint32_t)CCR2;
	DMA1_Channel4->CCR = 0x0;
	DMA1_Channel4->CCR |= DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_MINC;
	DMA1_Channel4->CNDTR = 44;


	//start DMA and TIM3
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	DMA1_Channel4->CCR |= DMA_CCR_EN;
	TIM3->CR1 |= TIM_CR1_CEN;


}



/*
 * at the end of each sampling this function is called
 */

void am_u_callback(void){

	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;

	for(uint8_t i=0;i<42;i++){
		DIF[i]=CCR1[i+1]-CCR2[i];

		if(DIF[i]>50)am_data.data |= (uint64_t)0x1 << (41-i);
	}

	cs = (uint8_t)(am_data.byte[4]+ am_data.byte[3]+ am_data.byte[2]+ am_data.byte[1]);

	if( (cs == am_data.byte[0]) && cs ){

		am_TMP = (float)am_data.result.tmp / 10.0f;
		am_RH = (float)am_data.result.rh/10.0f;

	}else{am_TMP=0;am_RH=0;am_damage++;}

	am_total++;

	am_data.data=0;
	am_loop=0;

	DMA1_Channel1->CNDTR = 44;
	DMA1_Channel4->CNDTR = 44;

	DMA1_Channel1->CCR |= DMA_CCR_EN;
	DMA1_Channel4->CCR |= DMA_CCR_EN;

}


/*
 * this is IRQ_Handler function for TIM4 update interrupt
 * these function address's are written in interrupt vector table in start up file.
 * in HAL library these functions are written in it.c file
 *
 * IRQHandler functions are first functions that are called after an interrupt is happened
 * they do these:
 *
 * 1.finding interrupt request
 * 2.clearing interrupt request flag
 * 3.calling callback function
 *
 */

void TIM4_IRQHandler(void){

	if( (TIM4->SR & TIM_SR_UIF)   && (TIM4->DIER & TIM_DIER_UIE) ){
		TIM4->SR = ~TIM_SR_UIF;
		am_u_callback();
	}

}






