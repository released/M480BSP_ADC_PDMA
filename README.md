# M480BSP_ADC_PDMA
 M480BSP_ADC_PDMA


update @ 2021/08/13

1. add define USE_ADC_CH0_3 , USE_ADC_CH7_10 , for different ADC channel
	
	USE_ADC_CH0_3 : PB0 ~ PB3
	
	USE_ADC_CH7_10 : PB7 ~ PB10


update @ 2020/06/10

1. Get ADC (PB7 ,PB8 ,PB9 ,PB10 ) with IRQ , and transfer with PDMA , 

2. parameter : ModuleNum , cound consider as priority , with the lower number , the higher priority

3. Scenario : 

- get ADC internal channel data with ADIF2 (ADINT2)

- get first ADC channel (PB.7) with software trigger and start ADINT0 IRQ

- get the other 3 ADC channels (PB8 ,PB9 ,PB10) , trigger by ADINT0 IRQ

- after 4 ADC channels convert , transfer by PDMA (array with 4 bytes , mapping to 4 ADC channel)

- output log with regular convert result : aADCxConvertedData , and PDMA transfer result : pdmaConvertedData

4. output log message as below ADC (PB7 ,PB8 ,PB9 ,PB10 ) and band-gap , temperature sensor , vbattery

![image](https://github.com/released/M480BSP_ADC_PDMA/blob/master/log.jpg)
