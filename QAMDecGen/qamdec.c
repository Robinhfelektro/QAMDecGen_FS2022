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


//Funktionen initialisieren
void send_to_data_buffer(uint16_t Data);
//uint32_t get_slope(uint16_t index_0to319 );
uint16_t idle_calculate_offset(uint16_t max, uint16_t min);
bool idle_get_min_max(uint16_t startindex, uint16_t* max, uint16_t* min, uint16_t* max_index, uint16_t* min_index);

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
	
	
	
		/*Funktionen:	berechnen min max mit index Pointer min min max und inex übergeben siehe mail
						(überprüfung ob wert valide ist)
						mean berechnen mit min max 
						
						
		Programm		32 adc werte mit funktion analysieren --> Logik damit Ringbuffer nicht überholt werden kann
						mean, max berechnen mit funktion
						0 PUnkt mithilfe von Index herausfinden
							jeweils 90 oder 180° sind innteressante Punkte verschoben
						
						
			
		1. Initialisierung mit Synchronisationssygnal. (z.B. 10 mal testen)
		2. Datean aus Ringbuffer auswerten und speichern (symbol erkenne, error index wait)
				- offetkorrektur
				- fehrlüberüfung --> zurück zu modus init
					- Wait auf ringbuffer
				- Daten speichern von Symbol
				
			2.1 Warten auf Startsymbol
			2.2 Startsymbol gekommen
			
				
		3. Daten aus Speicher auswerten mithilfe Protokoll ()
						
		1. DC Offset mit Synchrosignal herausfinden mit einem 64 bit bereich. max wert -8 = 0phasendurchgang
			--> sanity check +32 oder - 32 index muss wieder maximalwert sein
						-Funktion Sanity check
						-Min Max Berechnen Funktion
						- 
		2. Daten Signale auswerten mithilfe der Berechnenten 0 Punkte aus dem 1 modus berechen. Datensymbole abspeichern
		
		
						- check ringbuffer überholen oder vielleicht Errorcheck? Index check
						
		3. Datensymbole in Protokoll umrechnen 
		
										
	
		*/
	#define STATE_IDLE 1
	#define STATE_START 2
	xLastWakeTime = xTaskGetTickCount();
	
	
	uint8_t State_Switch = 1; 
	uint16_t idle_max_value = 0; 
	uint16_t idle_min_value = 0; 
	uint16_t idle_max_index = 0; 
	uint16_t idle_min_index = 0; 
	
	uint16_t Index = 0; 
	uint16_t DC_Offset = 0; 
	bool ErrorTest = pdFALSE; 
	uint8_t ululu; 
	while (1)
	{
		
		switch(State_Switch)
		{
			case STATE_IDLE:
			
				Index = raw_data_buffer_index % 320;  //unnötig??
				ErrorTest = idle_get_min_max(70, &idle_max_value, &idle_min_value, &idle_max_index, &idle_min_index);
				if (ErrorTest == pdTRUE)
				{
					ululu = 1; 
				}
				DC_Offset = idle_calculate_offset(idle_max_value, idle_min_value);
			
			break; 
			
			case STATE_START:
			break;
			
			default:
			break; 
		}
		vTaskDelayUntil( &xLastWakeTime, 5 / portTICK_RATE_MS);
	}
}


uint16_t idle_calculate_offset(uint16_t max, uint16_t min)
{
	uint16_t dc_offset = 0; 
	
	dc_offset = ((max - min) / 2) + min; 
	
	return dc_offset; 
}

bool idle_get_min_max(uint16_t startindex, uint16_t* max, uint16_t* min, uint16_t* max_index, uint16_t* min_index)
{
	uint16_t y1 = 0; 
	uint16_t y2 = 0; 
	uint16_t y1_index = 0; 
	uint16_t y2_index = 0; 
	
	uint16_t max_safe = 0; 
	uint16_t min_safe = 4096; //für erste if abfrage
	uint16_t max_index_safe = 0; 
	uint16_t min_index_safe = 0; 
	uint16_t kontrolle_max = 0; 
	uint16_t kontrolle_min = 0; 
	
	
	bool Error_Flag = pdFALSE; 
	
	for (uint8_t i = 0; i < 64; i++)
	{
		y1_index = i+startindex;
		y2_index = i+startindex+1;
		y1 = adc_rawdata_buffer[y1_index];
		y2 = adc_rawdata_buffer[y2_index];
		if ( (y2 > y1) & (y2 > max_safe)  )
		{
			max_safe = y2; 
			max_index_safe = y2_index; 
		}
		else
		{
			if ( (y2 < y1) & (y2 < min_safe))
			{
				min_safe = y2; 
				min_index_safe = y2_index;
 			}
		}
	}


	kontrolle_max = adc_rawdata_buffer[max_index_safe+64];  //Werte für kontrolle kopieren
	kontrolle_min = adc_rawdata_buffer[min_index_safe+64];

	
	if ( (kontrolle_max > (max_safe-100)) & (kontrolle_max < (max_safe+100)) )  //kontrolle maximalwert +- 100 von 4096 ca. 2.5% fehler
	{
		max = &max_safe;
		max_index = &max_index_safe; 
	}
	else
	{
		return pdTRUE;
	}
	
	if ( (kontrolle_min > (min_safe-100)) & (kontrolle_min < (min_safe+100)) )  //kontrolle minimalwert +- 100 von 4096 ca. 2.5% fehler
	{
		min = &min_safe; 
		min_index = &min_index_safe; 
	}
	else
	{
		return pdTRUE;
	}
	
	return Error_Flag; // 0 = ok, 1 = fehler
}


void get_slope(uint16_t index_0to319)  //
{
	
}



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
