#include "adc.h"
#include "stm32f4xx.h"

#define ADC1_EN 	(1U<<8)
#define PORTA_EN    (1U<<0)
#define ADC_CH1     (1U<<0)
#define ADC_ON      (1U<<0)

void adc_conf (void)
{

	RCC->APB2ENR |= ADC1_EN;

	//PA1 usato per analog ADC1
	RCC->AHB1ENR |= PORTA_EN;

	//PA0 analog mode
	/*GPIOA->MODER |= (1U<<0);
	  GPIOA->MODER |= (1U<<1);*/

	//PA1 analog mode
	GPIOA->MODER |= (1U<<2);
	GPIOA->MODER |= (1U<<3);

	//nel registro ADC_SQR ci sono diverse posizioni. In ognuna si deve mettere in posizione ordinata il canale dell'adc che si andrà a leggere
	//partendo dal primo che è SQ1 andando avanti. Il numero del canale deve essere messo in binario.

	ADC1->SQR3 = ADC_CH1;      //nei primi 5 bit che indicano la prima sequenza SQ1 si mette il numero del canale, in questo caso 1 in binario.

	//il bit L contenuto in SQR1 (dal bit 20 al 24) indica la lunghezza della sequenza, ovvero il numero di conversioni da effettuare.
	//in questo caso è solo una, perchè si utilizzerà un solo canale. Per la lunghezza di 1 la sequenza è 0000.

	ADC1->SQR1 = 0X00; // in realtà si mettono tutti a zero perchè le altre sequenze non contano.




	//per abilitare l'adc si setta il bit contenuto nel registro ADC_CR2
	ADC1->CR2 |= ADC_ON;
}


void start_conv(void)
{
	//per abilitare la conversione continua
	ADC1->CR2 |= (1U<<1);
	//start conversion
	ADC1->CR2 |= (1U<<30);
}


uint32_t read_conv(void)
{
	//attendere fine conversione
	//la fine della conversione è indicata da un bit presente nel registro ADC_SR status register (bit EOC end of conversion)
	while (!(ADC1->SR & (1U<<1))) {};  //in attesa della fine della conversione


	//infine restituisce il valore letto
	return (ADC1->DR);
}
