/*
 * adc.h
 *
 *  Created on: 1 nov 2021
 *      Author: Mattia
 */

#ifndef ADC_H_
#define ADC_H_

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdint.h>

void start_conv(void);
uint32_t read_conv(void);
void adc_conf (void);

#endif /* ADC_H_ */
