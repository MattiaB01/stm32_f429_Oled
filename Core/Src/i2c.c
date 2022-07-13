/*
 * i2c.c
 *
 *  Created on: 26 feb 2022
 *      Author: Mattia
 */
#include <stdio.h>
#include "stm32f4xx.h"
#include "i2c.h"

/* dal datasheet
 * pb8  -> ic21 scl
 * pb9  -> ic21 sda
 */


void i2c_start(void);
void i2c_stop(void);
void i2c_writeAddress(uint8_t address);
void i2c_writeRegister(uint8_t maddr);
void i2c_writeData(uint8_t data);


void i2c_init__2(void){
RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN; //enable gpiob clock
RCC->APB1ENR|=RCC_APB1ENR_I2C1EN; //enable i2c1 clock
GPIOB->MODER|=0xA0000; //set pb8and9 to alternative function
GPIOB->AFR[1]|=0x44;
GPIOB->OTYPER|=GPIO_OTYPER_OT8|GPIO_OTYPER_OT9; //set pb8 and pb9 as open drain
I2C1->CR1=I2C_CR1_SWRST;
I2C1->CR1&=~I2C_CR1_SWRST;
I2C1->CR2|=50;
I2C1->CCR|=0x2|(1<<15)|(1<<14);
I2C1->TRISE=20; //output max rise
I2C1->CR1|=I2C_CR1_PE;
}

void i2c_init(void) {

	/*Enable clk access to i2c1
	 * enable clk portB
	 * pb8,pb9 alternate fuction
	 * pb8,pb9 output mode open drain
	 * pb8,pb9 enable pullup
	 */
	RCC->AHB1ENR |= (1U << 1);
	RCC->APB1ENR |= (1U << 21);

	GPIOB->MODER |= (1U << 19);
	GPIOB->MODER &= ~(1U << 18);

	GPIOB->AFR[1] |= 0x44;

	GPIOB->MODER |= (1U << 17);
	GPIOB->MODER &= ~(1U << 16);

	GPIOB->OSPEEDR |= (3<<16) | (3<<18);  // Bits (17:16)= 1:1 --> High Speed for PIN PB8; Bits (19:18)= 1:1 --> High Speed for PIN PB9

	GPIOB->OTYPER |= (1U << 8);
	GPIOB->OTYPER |= (1U << 9);

	GPIOB->PUPDR |= (1U << 18);
	GPIOB->PUPDR &= ~(1U << 19);

	GPIOB->PUPDR |= (1U << 16);
	GPIOB->PUPDR &= ~(1U << 17);

	//RESET ENABLE
	I2C1->CR1 |= (1U << 15);
	//RESET DISABLE
	I2C1->CR1 &= ~(1U << 15);

	//I2C1->CR2 |= (1U << 4); //16MHZ
	I2C1->CR2 |= (45<<0); //45Mhz


//	I2C1->CCR = 80; //100KHZ
	I2C1->CCR = 225; //100KHZ

	//I2C1->TRISE = 17;
	I2C1->TRISE = 46;

	I2C1->CR1 |= (1U << 0);  //PE

}

int i2c_bus_scan(void) {
	int a = 0; //address variable

	int c = 0;

	for (uint8_t i = 0; i < 128; i++) //go through all 127 address
			{

		I2C1->CR1 |= I2C_CR1_START; //generate start

		while (!(I2C1->SR1 & I2C_SR1_SB)); // wait to start to be generated

		I2C1->DR = (i << 1 | 0); // transmit the address

		while (!(I2C1->SR1) | !(I2C1->SR2)) {}; //clear status register

		I2C1->CR1 |= I2C_CR1_STOP; //generate stop condition

		//delay(100); //minium wait time is 40 uS, but for sure, leave it 100 uS

		for (int b = 0; b < 1000; b++) {};

		a = (I2C1->SR1 & I2C_SR1_ADDR); //read the status register address set

		if (a == 2) //if the address is valid
				{
			//print the address
//			sprintf(data,
//					"Found I2C device at adress 0x%X (hexadecimal), or %d (decimal)\r\n",
//					i, i);
//			UART_Write_String(data);

			/*collegamento precedente pa9->rx
			 * pa10->tx , pa1->adc
			 */

			c = i;

		}

	}
	return c;
}

void i2c_byteRead(char saddr, char maddr, char *data) {
	//saddr= slave addr, maddr=memory addr, data = the data to send

	//variabile temporanea
	volatile int temp;

	//wait until i2c is not busy
	while (I2C1->SR2 & (1U << 1)) {
	}

	//generate start
	I2C1->CR1 |= (1U << 8);

	//wait until the start condition is generated (attesa fino a quando non è settato)
	while (!(I2C1->SR1 & (1U << 0))) {
	};

	//l'indirizzo slave è di 7 bit mentre la variabile che lo contiene è di 8 quindi sposto tutto di 1 bit
	I2C1->DR = saddr << 1;

	//attesa fino a quando il data register è settato
	while (!(I2C1->SR1 & (1U << 1))) {
	}; //ciclo di attesa mentre non è settato

	//clear addr flag
	temp = I2C1->SR2;

	//send maddr
	I2C1->DR = maddr;

	//wait until trasmitter is empty
	while (!((I2C1->SR1) & (1U << 7))) {
	}; //TXE

	//restart condition
	I2C1->CR1 |= (1U << 8);

	//wait until the start condition is generated (attesa fino a quando non è settato)
	while (!(I2C1->SR1 & (1U << 0))) {
	};

	//transmit slave addr + bit read
	I2C1->DR = saddr << 1 | 1;

	//disable ack
	I2C1->CR1 &= ~(1U << 10);

	//clear addr flag
	temp = I2C1->SR2;

	//generate stop condition
	I2C1->CR1 |= (1U << 9);

	//attesa RXE è settato
	while (!((I2C1->SR1) & (1U << 6))) {
	};

	//read data
	*data++ = I2C1->DR;

}

void i2c_burstRead(char saddr, char maddr, int n, char *data) {
	//variabile temporanea
	volatile int temp;

	//wait until i2c is not busy
	while (I2C1->SR2 & (1U << 1)) {
	}

	//generate start
	I2C1->CR1 |= (1U << 8);

	//wait until the start condition is generated (attesa fino a quando non è settato)
	while (!(I2C1->SR1 & (1U << 0))) {
	};

	//l'indirizzo slave è di 7 bit mentre la variabile che lo contiene è di 8 quindi sposto tutto di 1 bit
	I2C1->DR = saddr << 1;

	//attesa fino a quando il data register è settato
	while (!(I2C1->SR1 & (1U << 1))) {
	}; //ciclo di attesa mentre non è settato

	//clear addr flag
	temp = I2C1->SR2;

	//wait until trasmitter is empty
	while (!((I2C1->SR1) & (1U << 7))) {
	}; //TXE

	//send maddr
	I2C1->DR = maddr;

	//wait until trasmitter is empty
	while (!(I2C1->SR1) & (1U << 7)) {
	}; //TXE

	//restart condition
	I2C1->CR1 |= (1U << 8);

	//wait until the start condition is generated (attesa fino a quando non è settato)
	while (!(I2C1->SR1 & (1U << 0))) {
	};

	//transmit slave addr + bit read
	I2C1->DR = saddr << 1 | 1;

	//attesa fino a quando il data register è settato
	while (!(I2C1->SR1 & (1U << 1))) {
	}; //ciclo di attesa mentre non è settato

	//clear addr flag
	temp = I2C1->SR2;

	//enable Ack
	I2C1->CR1 |= (1U << 10); //ack

	while (n > 0U) {
		if (n == 1U) {
			//disable ack
			I2C1->CR1 &= ~(1U << 10);

			//stop condition
			I2C1->CR1 |= (1U << 9);

			//attesa RXE è settato
			while (!((I2C1->SR1) & (1U << 6))) {};

			*data++ = I2C1->DR;

			break;
		}

		else {

			while (!((I2C1->SR1) & (1U << 6))) {};

			*data++ = I2C1->DR;

			n--;
			}

	}
}

void i2c_WriteMulti__(char saddr, char maddr, uint8_t *data,uint8_t length){

	volatile int temp;

	//wait until i2c is not busy
		while (I2C1->SR2 & (1U << 1)) {}

		//generate start
			I2C1->CR1 |= (1U << 8);

			//I2C1->CR1 |= I2C_CR1_START;

			//wait until the start condition is generated (attesa fino a quando non è settato)
				while (!(I2C1->SR1 & (1U << 0))) {};

				//transmit slave address
				I2C1->DR = saddr<<1;

				//attesa fino a quando il data register è settato
				while (!(I2C1->SR1 & (1U << 1))) {}; //ciclo di attesa mentre non è settato

					//clear addr flag
						temp = I2C1->SR2;

//						Bit 7 TxE: Data register empty (transmitters)
//						0: Data register not empty
//						1: Data register empty
//						– Set when DR is empty in transmission. TxE is not set during address phase.

						//wait until data register empty (TXE)
						while (!(I2C1->SR1&(1U<<7))){};

						//send memory address
						I2C1->DR=maddr;

						for (int i=0;i<length;i++)
						{
							//wait until data register empty (TXE)
							while (!(I2C1->SR1)&(1U<<7)){};


							//Transmit data
							I2C1->DR = *data++;
						}

//						Bit 2 BTF: Byte transfer finished
//						0: Data byte transfer not done
//						1: Data byte transfer succeeded

						//wait until transfer finished
						while(!(I2C1->SR1&(1U<<2))) {};

						//generate stop
						I2C1->CR1 |= (1U<<9);

//						I2C1->CR1 |= I2C_CR1_STOP;

}

void i2c_writeByte__(char saddr, char maddr, char data) {

	volatile int temp;

	//wait until i2c is not busy
	while (I2C1->SR2 & (1U << 1)) {};

	//generate start
	I2C1->CR1 |= (1U << 8);

	//I2C1->CR1 |= I2C_CR1_START;

	//wait until the start condition is generated (attesa fino a quando non è settato)
	while (!(I2C1->SR1 & (1U << 0))) {};

	//transmit slave address
	I2C1->DR = saddr << 1;

	//attesa fino a quando il data register è settato
	while (!(I2C1->SR1 & (1U << 1))) {}; //ciclo di attesa mentre non è settato

	//clear addr flag
	temp = I2C1->SR2;

//						Bit 7 TxE: Data register empty (transmitters)
//						0: Data register not empty
//						1: Data register empty
//						– Set when DR is empty in transmission. TxE is not set during address phase.

	//wait until data register empty (TXE)
	while (!(I2C1->SR1 & (1U << 7))) {};

	//send memory address
	I2C1->DR = maddr;

	//wait until data register empty (TXE)
	while (!(I2C1->SR1 & (1U << 7))) {};

	I2C1->DR =data;

	//wait until transfer finished
	while(!(I2C1->SR1&(1U<<2))) {};

	I2C1->CR1 |= I2C_CR1_STOP;

}

void i2c_writeByte(uint8_t saddr, uint8_t maddr, uint8_t data) {

	i2c_start();
	//send address

	i2c_writeAddress(saddr);

	i2c_writeRegister(maddr);

	i2c_writeData(data);

	i2c_stop();
	//
//	volatile int Temp;
//	while(I2C1->SR2&I2C_SR2_BUSY){;}          /*wait until bus not busy*/
//	I2C1->CR1|=I2C_CR1_START;                 /*generate start*/
//	while(!(I2C1->SR1&I2C_SR1_SB)){;}         /*wait until start bit is set*/
//	I2C1->DR = saddr<< 1;                 	 /* Send slave address*/
//	while(!(I2C1->SR1&I2C_SR1_ADDR)){;}      /*wait until address flag is set*/
//	Temp = I2C1->SR2; 											 /*clear SR2 by reading it */
//	while(!(I2C1->SR1&I2C_SR1_TXE)){;}       /*Wait until Data register empty*/
//	I2C1->DR = maddr;                        /* send memory address*/
//	while(!(I2C1->SR1&I2C_SR1_TXE)){;}       /*wait until data register empty*/
//	I2C1->DR = data;
//	while (!(I2C1->SR1 & I2C_SR1_BTF));      /*wait until transfer finished*/
//	I2C1->CR1 |=I2C_CR1_STOP;								 /*Generate Stop*/

}

void i2c_writeMulti (uint8_t saddr, uint8_t maddr, uint8_t *data, uint8_t size)
{
/**** STEPS FOLLOWED  ************
1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
2. Keep Sending DATA to the DR Register after performing the check if the TXE bit is set
3. Once the DATA transfer is complete, Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
*/

		//start
		i2c_start();

		//send address
		i2c_writeAddress(saddr);

		//write reg
		i2c_writeRegister(maddr);

		while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
		while (size)
		{
			while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
			I2C1->DR = (uint32_t )*data++;  // send data
			size--;
		}

		while (!(I2C1->SR1 & (1<<2)));  // wait for BTF to set

		//stop
		i2c_stop();

}

void i2c_start(void)
{
	I2C1->CR1 |= (1<<10);  // Enable the ACK
	I2C1->CR1 |= (1<<8);  // Generate START
	while (!(I2C1->SR1 & (1<<0)));  // Wait for SB bit to set
}


void i2c_stop(void)
{

	//stop
	I2C1->CR1 |= (1<<9);  // Stop I2C

}

void i2c_writeAddress(uint8_t saddr)
{
	//send address
	I2C1->DR = saddr;  //  send the address
	while (!(I2C1->SR1 & (1 << 1)));  // wait for ADDR bit to set
	uint8_t temp = I2C1->SR1 | I2C1->SR2; // read SR1 and SR2 to clear the ADDR bit

}

void i2c_writeRegister(uint8_t maddr)
{
	//write reg
	while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
	I2C1->DR = maddr;
	while (!(I2C1->SR1 & (1<<2)));  // wait for BTF bit to set

}

void i2c_writeData(uint8_t data)
{	//write data
	while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
	I2C1->DR = data;
	while (!(I2C1->SR1 & (1<<2)));  // wait for BTF bit to set
}
