/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define LED_R							(PH0)
#define LED_Y							(PH1)
#define LED_G							(PH2)

#define ADC_DIGITAL_SCALE(void) 					(0xFFFU >> ((0) >> (3U - 1U)))		//0: 12 BIT 
#define ADC_CALC_DATA_TO_VOLTAGE(DATA,VREF) ((DATA) * (VREF) / ADC_DIGITAL_SCALE())

#define ADC_PDMA_CH 					(2)
#define ADC_PDMA_OPENED_CH   			(1 << ADC_PDMA_CH)
#define ADC_DMA_SAMPLE_COUNT 		(4)

#define USE_ADC_CH0_3
//#define USE_ADC_CH7_10

uint16_t aADCxConvertedData[ADC_DMA_SAMPLE_COUNT] = {0};
uint16_t pdmaConvertedData[ADC_DMA_SAMPLE_COUNT] = {0};

uint32_t Vgap = 0;
uint32_t Vtemp = 0;
uint32_t Vbat = 0;

float SensorData = 0;
float fVDD = 0;
float Check = 0;

enum
{
	ADC0_CH0 = 0 ,
	ADC0_CH1 ,
	ADC0_CH2 , 
	ADC0_CH3 , 
	ADC0_CH4 ,
	ADC0_CH5 , 
	ADC0_CH6 , 
	ADC0_CH7 ,
	ADC0_CH8 , 
	ADC0_CH9 , 
	ADC0_CH10 , 
	ADC0_CH11 ,
	ADC0_CH12 , 
	ADC0_CH13 , 
	ADC0_CH14 ,
	ADC0_CH15 , 
	
	ADC0_CH16_BAND_GAP_VOLT , 
	ADC0_CH17_TEMP_SENSOR ,
	ADC0_CH18_VBAT , 
	
	ADC_CH_DEFAULT 	
}ADC_CH_TypeDef;

typedef enum{

	flag_ADC_Band_GAP = 0 ,
	flag_ADC_Data_Ready ,	
	flag_ADC_Sensor_Ready ,
	
	flag_PDMA_Trans_Data_Ready ,
	
	flag_DEFAULT	
}Flag_Index;

uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

void ReloadPDMA(void)
{
    /* transfer width is half word(16 bit) and transfer count is ADCDatalenght+1 */
    PDMA_SetTransferCnt(PDMA, ADC_PDMA_CH, PDMA_WIDTH_16, ADC_DMA_SAMPLE_COUNT);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA, ADC_PDMA_CH, PDMA_EADC0_RX, FALSE, (uint32_t) 0);
}

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);
	
    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
		#if 0
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
		#else
        if (PDMA_GET_ABORT_STS(PDMA) & ADC_PDMA_OPENED_CH)
        {
			printf("ABTSTS\r\n");
        }
        PDMA_CLR_ABORT_FLAG(PDMA, ADC_PDMA_OPENED_CH);

		#endif
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if(PDMA_GET_TD_STS(PDMA) & ADC_PDMA_OPENED_CH)
        {
			//insert process
			set_flag(flag_PDMA_Trans_Data_Ready , ENABLE);
			LED_G ^= 1;
			
//			printf("TDIF\r\n");
        }        

		/* Clear PDMA transfer done interrupt flag */
		PDMA_CLR_TD_FLAG(PDMA, ADC_PDMA_OPENED_CH);
    }
    else if (status & (PDMA_INTSTS_REQTOF1_Msk))     /* Check the DMA time-out interrupt flag */
    {
        PDMA_CLR_TMOUT_FLAG(PDMA,ADC_PDMA_CH);
		printf("REQTOF\r\n");

    }
    else
    {

    }
	
}


void PDMA_Init(void)
{
	SYS_ResetModule(PDMA_RST);

    /* Configure PDMA peripheral mode form ADC to memory */
    /* Open PDMA Channel 1 based on ADC_PDMA_CH setting*/
    PDMA_Open(PDMA, BIT0 << ADC_PDMA_CH);

    /* transfer width is half word(16 bit) and transfer count is ADCDatalenght+1 */
    PDMA_SetTransferCnt(PDMA, ADC_PDMA_CH, PDMA_WIDTH_16, ADC_DMA_SAMPLE_COUNT);

    /* Set source address as ADC data register (no increment) and destination address as g_i32ConversionData array (increment) */
    PDMA_SetTransferAddr(PDMA, ADC_PDMA_CH, (uint32_t)& (EADC->CURDAT), PDMA_SAR_FIX, (uint32_t)pdmaConvertedData, PDMA_DAR_INC);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA, ADC_PDMA_CH, PDMA_EADC0_RX, FALSE, 0);

    /* Set PDMA as single request type for ADC */
    PDMA_SetBurstType(PDMA, ADC_PDMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_128);

    PDMA_EnableInt(PDMA, ADC_PDMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);
	
	PDMA_Trigger(PDMA , ADC_PDMA_CH);

    /* ADC enable PDMA transfer */
    EADC_ENABLE_PDMA(EADC);
}

void EADC00_IRQHandler(void)
{
//    set_flag(flag_ADC_Band_GAP , ENABLE);
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

void EADC01_IRQHandler(void)
{
    set_flag(flag_ADC_Data_Ready , ENABLE);
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);
}

void EADC02_IRQHandler(void)
{
//	set_flag(flag_ADC_Sensor_Ready , ENABLE);
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);
}

void ADC_Read_Int_Channel(void)
{
	set_flag(flag_ADC_Sensor_Ready , DISABLE);
	
   /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    /* Set sample module 16 external sampling time to 0xF */
    EADC_SetExtendSampleTime(EADC, ADC0_CH16_BAND_GAP_VOLT, 0x3F);
    EADC_SetExtendSampleTime(EADC, ADC0_CH17_TEMP_SENSOR, 0x3F);

    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);
    EADC_ENABLE_INT(EADC, (BIT0 << 2));
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 2, (BIT0 << ADC0_CH16_BAND_GAP_VOLT));
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 2, (BIT0 << ADC0_CH17_TEMP_SENSOR));
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 2, (BIT0 << ADC0_CH18_VBAT));
	
    NVIC_EnableIRQ(EADC02_IRQn);

    EADC_START_CONV(EADC, (BIT0 << ADC0_CH16_BAND_GAP_VOLT) | (BIT0 << ADC0_CH17_TEMP_SENSOR)| (BIT0 << ADC0_CH18_VBAT));

//	while(is_flag_set(flag_ADC_Sensor_Ready) == DISABLE);
    Vgap = EADC_GET_CONV_DATA(EADC, ADC0_CH16_BAND_GAP_VOLT);
    Vtemp = EADC_GET_CONV_DATA(EADC, ADC0_CH17_TEMP_SENSOR);
    Vbat = EADC_GET_CONV_DATA(EADC, ADC0_CH18_VBAT);
	
    EADC_DISABLE_INT(EADC, (BIT0 << 2));
 	NVIC_DisableIRQ(EADC02_IRQn);
	
}


void ADC_Convert_Ext_Channel(void)
{
	uint8_t i = 0;

	uint8_t SampleCount = 0;
	uint8_t ModuleCount = 0;

	#if defined (USE_ADC_CH0_3)
	uint8_t ModuleNum = ADC0_CH0;
	#elif defined (USE_ADC_CH7_10)
	uint8_t ModuleNum = ADC0_CH7;
	#endif
	
	set_flag(flag_PDMA_Trans_Data_Ready , DISABLE);	
	set_flag(flag_ADC_Data_Ready , DISABLE);
	
    /* Set input mode as single-end, and Single mode*/
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

	EADC_ConfigSampleModule(EADC, ModuleNum, EADC_SOFTWARE_TRIGGER, ModuleNum);
	EADC_ConfigSampleModule(EADC, ModuleNum+1, EADC_ADINT0_TRIGGER, ModuleNum+1);
	EADC_ConfigSampleModule(EADC, ModuleNum+2, EADC_ADINT0_TRIGGER, ModuleNum+2);
	EADC_ConfigSampleModule(EADC, ModuleNum+3, EADC_ADINT0_TRIGGER, ModuleNum+3);

    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);
    EADC_ENABLE_INT(EADC, BIT0);
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, (BIT0 << ModuleNum));

    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);
    EADC_ENABLE_INT(EADC, BIT1);
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 1, (BIT0 << (ModuleNum+3)));

//    NVIC_EnableIRQ(EADC00_IRQn);
    NVIC_EnableIRQ(EADC01_IRQn);
	
	PDMA_Init();

    EADC_START_CONV(EADC, (BIT0 << ModuleNum));
//	while(is_flag_set(flag_ADC_Data_Ready) == DISABLE);

	for (ModuleCount = ModuleNum , SampleCount = 0; ModuleCount < (ModuleNum + ADC_DMA_SAMPLE_COUNT) ; ModuleCount++, SampleCount++)
	{
		aADCxConvertedData[SampleCount] = EADC_GET_CONV_DATA(EADC, ModuleCount);
	}


//    while(is_flag_set(flag_PDMA_Trans_Data_Ready) == DISABLE);
	for (i = 0 ; i < ADC_DMA_SAMPLE_COUNT; i++)
	{
		printf("0x%3X,0x%3X,|" , aADCxConvertedData[i],pdmaConvertedData[i] );
	}

	printf(" , 0x%3X,0x%3X,0x%3X" , Vgap , Vtemp, Vbat);

	printf("\r\n");
	
}


void TMR1_IRQHandler(void)
{
	static uint16_t CNT = 0;	
//	static uint32_t log = 0;	
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
	
		if (CNT++ > 1000)
		{		
			CNT = 0;
//			printf("%s : %2d\r\n" , __FUNCTION__ , log++);
//			ADC_Convert_Ext_Channel(ADC0_CH7);
			LED_R ^= 1;
		}
    }
}

void TIMER1_HW_Init(void)
{
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void TIMER0_HW_Init(void)
{
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
}

void TIMER0_Polling(uint32_t u32Usec)
{
	TIMER_Delay(TIMER0, u32Usec);
}

void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is 96MHz, set divider to 8, EADC clock is 96/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    CLK_EnableModuleClock(PDMA_MODULE);

	TIMER0_HW_Init();
	TIMER1_HW_Init();
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

	#if defined (USE_ADC_CH0_3)
    PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk| SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_EADC0_CH0 | SYS_GPB_MFPL_PB1MFP_EADC0_CH1 | SYS_GPB_MFPL_PB2MFP_EADC0_CH2| SYS_GPB_MFPL_PB3MFP_EADC0_CH3);

    /* Disable the GPB0 - GPB3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0|BIT1|BIT2|BIT3);
	
	#elif defined (USE_ADC_CH7_10)
    PB->MODE &= ~(GPIO_MODE_MODE7_Msk | GPIO_MODE_MODE8_Msk | GPIO_MODE_MODE9_Msk | GPIO_MODE_MODE10_Msk);

    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB7MFP_Msk );
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB7MFP_EADC0_CH7);

    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB8MFP_Msk | SYS_GPB_MFPH_PB9MFP_Msk | SYS_GPB_MFPH_PB10MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB8MFP_EADC0_CH8 | SYS_GPB_MFPH_PB9MFP_EADC0_CH9 | SYS_GPB_MFPH_PB10MFP_EADC0_CH10);

    /* Disable the GPB0 - GPB3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT10|BIT9|BIT8|BIT7);
	#endif

    /* Enable temperature sensor */
    SYS->IVSCTL |= SYS_IVSCTL_VTEMPEN_Msk;

    /* Set reference voltage to external pin (3.3V) */
    SYS_SetVRef(SYS_VREFCTL_VREF_PIN);
	
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
	
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());


	LED_Init();
	TIMER1_Init();
	
    /* Got no where to go, just loop forever */
    while(1)
    {
//		TIMER0_Polling(1000);
		LED_Y ^= 1;

		ADC_Read_Int_Channel();
		ADC_Convert_Ext_Channel();
	
    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
