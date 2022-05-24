/*
* qamdec.c
*
* Created: 05.05.2020 16:38:25
*  Author: Chaos
*/ 

#include "avr_compiler.h"
#include "pmic_driver.h"
#include "TC_driver.h"
#include "clksys_driver.h"
#include "sleepConfig.h"
#include "port_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
#include "stack_macros.h"

#include "mem_check.h"
#include "errorHandler.h"

#include "qaminit.h"
#include "qamdec.h"

QueueHandle_t decoderQueue;


uint16_t adc_rawdata_buffer[320] = {};  //32 * 16bit var * 10 ms
uint32_t raw_data_buffer_index = 0; 	

TickType_t xLastWakeTime;

void send_to_data_buffer(uint16_t Data);
uint32_t get_slope(uint16_t index_0to319 );

void vQuamDec(void* pvParameters)
{
	( void ) pvParameters;
	
	decoderQueue = xQueueCreate( 4, NR_OF_SAMPLES * sizeof(int16_t) );  //datenarray
	
	while(evDMAState == NULL) {
		vTaskDelay(3/portTICK_RATE_MS);
	}
	
	uint16_t bufferelement[NR_OF_SAMPLES];
	
	xEventGroupWaitBits(evDMAState, DMADECREADY, false, true, portMAX_DELAY);
	
	for(;;) {
		while(uxQueueMessagesWaiting(decoderQueue) > 0) {
			if(xQueueReceive(decoderQueue, &bufferelement[0], portMAX_DELAY) == pdTRUE) {  
				for (uint8_t i = 0; i < 32; i++)
				{	
					send_to_data_buffer( bufferelement[i] );
				}
			}
		}		
		vTaskDelay( 2 / portTICK_RATE_MS );
	}
}

void send_to_data_buffer(uint16_t Data)
{
	adc_rawdata_buffer[raw_data_buffer_index % 320] = Data; 
	raw_data_buffer_index++;  
}

uint16_t get_data_from_buffer(uint32_t index_adress)
{
	return adc_rawdata_buffer[index_adress % 320];
}


void vQuamDecAnalysis(void* pvParameters)
{
	xLastWakeTime = xTaskGetTickCount();
	uint32_t slope_value = 0; 
	uint16_t max = 0;
	uint16_t min = 0; 
	while (1)
	{
		
		
		slope_value = get_slope(0);
		max = slope_value >> 16;
		min = slope_value; 

		vTaskDelayUntil( &xLastWakeTime, 5 / portTICK_RATE_MS);
	}
}

uint32_t get_slope(uint16_t index_0to319 )  //
{
	uint16_t y1 = 0;
	uint16_t y2 = 0;
	uint32_t slope_save = 0; 
	uint16_t calculate_max = 0; 
	uint16_t calculate_min = 0; 
	uint32_t return_test = 0; 
	for (uint8_t i = 0; i < 32; i++)
	{
		y1 = adc_rawdata_buffer[i];
		y2 = adc_rawdata_buffer[i+1];
		if (y2 > y1) //slope positive
		{
			slope_save = (slope_save << 1) | 0x0001; //1 an erster stelle, 1 für pos und 0 für neg slope
		}
		else //slope negative
		{
			slope_save = (slope_save << 1) & 0x0000; //0 an erster stelle
		}
	}
	for (uint8_t i = 0; i < 32; i++)
	{
		if ( ( ((slope_save >> i) & 1) == 0) && (slope_save >> (i + 1) ) && (slope_save >> (i + 2) )    )  //wendepunkt, 0 = negativ, 1 = pos
		{
			calculate_max = adc_rawdata_buffer[i + 1]; //nächster wert nach wendepunkt
			
		}
		if ( (slope_save >> i)  && ( (slope_save >> (i + 1)) == 0 ) && ((slope_save >> (i + 2)) == 0  )    )  //wendepunkt, 0 = negativ, 1 = pos
		{
			calculate_min = adc_rawdata_buffer[i + 1]; //nächster wert nach wendepunkt
		}
	}
	return_test = (  (calculate_max << 16)   |  ( calculate_min) );
	return return_test; 
	//return (  (calculate_max << 16)   |  ( calculate_min) );
}


//uint32_t get_slope(uint16_t index_0to319 )  //
//{
	//uint16_t y1 = 0;
	//uint16_t y2 = 0;
	//uint32_t slope_save = 0; 
	//for (uint8_t i = 0; i < 32; i++)
	//{
		//y1 = adc_rawdata_buffer[i];
		//y2 = adc_rawdata_buffer[i+1];
		//if (y2 > y1) //slope positive
		//{
			//slope_save = (slope_save << 1) | 1;
		//}
		//else //slope negative
		//{
			//slope_save = (slope_save << 1) | 0;
		//}
	//}
	//return slope_save; 
//}


void get_sin_mean(uint16_t index)
{
	//12 bit adc --> 4096 max wert
	for (uint8_t i = 0; i < 32; i++)
	{
		
	}
}

void fillDecoderQueue(uint16_t buffer[NR_OF_SAMPLES])
{
	BaseType_t xTaskWokenByReceive = pdFALSE;

	xQueueSendFromISR( decoderQueue, &buffer[0], &xTaskWokenByReceive );
}

ISR(DMA_CH2_vect)
{
	DMA.CH2.CTRLB|=0x10;

	fillDecoderQueue( &adcBuffer0[0] );  //globaler adc, DMA Buffer
}

ISR(DMA_CH3_vect)
{
	DMA.CH3.CTRLB |= 0x10;

	fillDecoderQueue( &adcBuffer1[0] );
}
