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
uint16_t idle_calculate_offset(uint16_t max, uint16_t min);
bool start_symbol_search(void);
bool idel_check_all_zerophase(uint16_t max_index_check, uint16_t dc_offset, uint16_t* zerophase_index);
void decipher_ringbuffer_1symbol(uint16_t index , uint16_t first_zerophase, uint16_t idle_max_value); //daten auswerten
uint32_t get_320_index(uint32_t index);
bool idle_get_min_max(uint32_t index, uint16_t* max, uint16_t* min, uint16_t* max_index, uint16_t* min_index);
bool get_phase_maxindex( uint32_t zero_index, uint16_t max_value, uint16_t min_value, uint16_t dc_offset, uint8_t* symbol_return );
bool idle_get_constant_values(uint32_t index, uint16_t* max, uint16_t* min, uint32_t* zero_index);
bool idle_check_all_zerophase_new(uint32_t* zero_index, uint16_t dc_offset, uint8_t check_strength);

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




void vQuamDecAnalysis(void* pvParameters)
{
	#define STATE_IDLE_INIT 1
	#define STATE_WAIT_STARTBIT 2
	#define STATE_START 3
	xLastWakeTime = xTaskGetTickCount();
	
	
	uint8_t State_Switch = 1; 
	uint16_t idle_max_value = 0; 
	uint16_t idle_min_value = 0; 


	uint16_t first_zerophase = 0; 
	uint16_t DC_Offset = 0; 
	
	
	
	
	bool ErrorTest = pdFALSE; 
	uint16_t Index = 0; 
	uint8_t sanity_check; 
	uint32_t decoder_index = 0; 
	uint32_t zero_phaseindex = 0; 
	
	uint32_t raw_buffer_index_ululu = 0; 
	uint16_t timeout_time = 0;
	
	uint8_t Symbol_Data_array[120] = {};

	while (1)
	{
		switch(State_Switch)
		{
			case STATE_IDLE_INIT:
				
				if (raw_data_buffer_index >= 1000)
				{
					if (raw_data_buffer_index >=  decoder_index + 200) //Warten auf neue Werte
					{
						decoder_index = raw_data_buffer_index - 100; //nach raw data buffer
						
						
						if (idle_get_constant_values(decoder_index, &idle_max_value, &idle_min_value, &zero_phaseindex ))
						{
							sanity_check+= 1; 
						}
						else
						{
							sanity_check = 0; 
						}
						if (sanity_check >= 5)
						{
							//DC_Offset = idle_calculate_offset(idle_max_value, idle_min_value);
							
							decoder_index = raw_data_buffer_index - 100;
							 
							DC_Offset = idle_calculate_offset(idle_max_value, idle_min_value);
							if (idle_check_all_zerophase_new(&zero_phaseindex, DC_Offset, 5 ) )  //wenn 0 phase ok == 1
							{
								//next mode --> wait for start bit
								State_Switch = STATE_WAIT_STARTBIT; 
							}
							else
							{
								sanity_check = 0; 
							}
						}
					}
				}
			
				
				
			break; 
			
			case STATE_WAIT_STARTBIT:
						
						
						decoder_index = zero_phaseindex;
						while( (decoder_index + 40) < raw_data_buffer_index)
						{
							
							
							//deode_ringbuffer_symbol();
							decoder_index+= 32; 
						}
						
							
				
						//todo --> eher mit absolutem index rechne --> sonst muss einmal modulo zurückgerechnet werden bei setzen. 
						//das if eher mit while ersetzen. --> sonst wird while ausgelöstA
						
						if (  (decoder_index + 40)   <  raw_data_buffer_index ) //so gelöst
						{
							
							decipher_ringbuffer_1symbol(decoder_index , &first_zerophase, idle_max_value); //daten auswerten
							decoder_index = raw_data_buffer_index + 32; 
							
							
							if (start_symbol_search() == 1)
							{
								State_Switch = STATE_START; //
							}
							
							
						
							if (pdFALSE)  //falls fehler erkannt wieder in init/idle zustand
							{
								State_Switch = STATE_IDLE_INIT; 
							}
						}
						else
						{
							
						}
						
						
			break; 
			
			case STATE_START:
						if (pdFALSE) //checksumme überprüft und stimmt --> bereit für weitere Daten
						{
							State_Switch = STATE_WAIT_STARTBIT; 
						}
						if (pdFALSE) //checksumme falsch oder fehler
						{
							State_Switch = STATE_IDLE_INIT; 
						}
			break;
			default:
			break; 
		}
		vTaskDelayUntil( &xLastWakeTime, 5 / portTICK_RATE_MS);
	}
}

bool deode_ringbuffer_symbol_new(uint32_t zero_index, uint16_t dc_offset)
{
	//get_phase_maxindex();  //irgendwie alles in dieser funktion 
	// write to array?? oder alles in einer funktion??
	
	
}

bool get_phase_maxindex( uint32_t zero_index, uint16_t max_value, uint16_t min_value, uint16_t dc_offset, uint8_t* symbol_return )
{
	
	if ( get_320_index(zero_index) )  // 0 durchgang überprüfen +- 2.5 % abbweichung
	{
		return pdFALSE;
	}
	uint16_t half_ampl_pos = (max_value - dc_offset) / 2 ; 
	uint16_t half_ampl_neg = dc_offset - half_ampl_pos;
	
	//0 phase, max ampli		1	
	//180 phase, max ampli		2	
	//0 phase, 1/2 ampli		3
	//180 phase, 1/2 ampli		4
	*symbol_return = 0; 
	
	//todo --> doppel ifs mit & um mehrfach zu verhindern
	if (get_320_index(zero_index + 8) > (max_value - 50) )  //0 phase, max ampli 1
	{
		if (*symbol_return == 0)
		{
			*symbol_return = 1;
		}
	}
	if (get_320_index(zero_index + 8) > (half_ampl_pos - 50) ) //0 phase, half ampli
	{
		if (*symbol_return == 0)
		{
			*symbol_return = 3;
		}
	}
	if (get_320_index(zero_index + 8) > (min_value + 50) ) //180 ° phase, max ampli
	{
		if (*symbol_return == 0)
		{
			*symbol_return = 2;
		}
	}
	if (get_320_index(zero_index + 8) > (half_ampl_neg + 50) ) //180 ° phase, max ampli
	{
		if (*symbol_return == 0)
		{
			*symbol_return = 4;
		}
	}
	return pdTRUE;
	
}



bool start_symbol_search(void)
{
	
}
uint32_t get_320_index(uint32_t index)
{
	return index % 320;
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


void decipher_ringbuffer_1symbol(uint16_t index , uint16_t first_zerophase, uint16_t idle_max_value) //daten auswerten
{
	//index welcher übergeben wird --> die nächsten 4 * 32 datenwerte auswerte == 1 byte
	
	uint16_t indexoffset = get_320_index(index);
	
	//uint8_t phase_faktor = start_index / 32; 
	//uint16_t zero_index = firtzerophase + (i * 32);
	
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
	
	for (uint8_t i = 0; i < 32; i++)
	{
		y1_index = i+indexoffset;
		y2_index = i+1+indexoffset;
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
	//uint8_t get_phase_maxindex( uint16_t index); 
	
	for (uint8_t i = 0; i < 32; i++)
	{
		
	}
}

uint8_t get_symbol(uint16_t max_value , uint16_t max_value_2 , uint8_t phase)
{
	
}




bool idle_check_all_zerophase_new(uint32_t* zero_index, uint16_t dc_offset, uint8_t check_strength)
{
	uint8_t zerophase_dedection_count = 0; 
	
	for(uint8_t i = 0; i < check_strength; i++) //up to 10
	{
		if ((adc_rawdata_buffer[get_320_index(*zero_index + (i*32))] > (dc_offset-100))  &  
		   ((adc_rawdata_buffer[get_320_index(*zero_index + (i*32))] < (dc_offset+100)) )) //+- 100 von 4096 ca. 2.5% fehler
		{
			zerophase_dedection_count++;
		}
	}
	if (zerophase_dedection_count == check_strength) //daten valide  0 bis 10 mal im array überprüfen
	{
		*zero_index = *zero_index + (check_strength*32);  //update zero_index to new value 
		return pdTRUE;  //wenn berechnung ok 1
	}
	else
	{
		return pdFALSE;
	}
}


bool idel_check_all_zerophase(uint16_t max_index_check, uint16_t dc_offset, uint16_t* zerophase_index)
{
	uint16_t start_zerophase = 0; 
	uint8_t zerophase_dedection_count = 0; 
	bool b_first_index_too_low = pdFALSE; 
	
	if (max_index_check <= 7)
	{
		start_zerophase = max_index_check - 8 + 32; //nächster 0° durchgang
		b_first_index_too_low = pdTRUE;
	}
	else
	{
		start_zerophase = max_index_check - 8;
	}

	for(uint8_t i = 0; i < 10; i++)
	{
		start_zerophase = start_zerophase - 32;
		if (start_zerophase >= 321)  //overflow detected
		{
			if (b_first_index_too_low)
			{
				start_zerophase = (max_index_check - 8 + 32) - (i * 32);
			}
			else
			{
				start_zerophase =  (max_index_check - 8) - (i * 32);
			}
			break; 
		}
	}
	for (uint8_t i = 0; i < 5; i++)  //alle 0 punkte koorinaten in einem Array speichern? wieder über pointer?
	{
		if (   (adc_rawdata_buffer[start_zerophase] > (dc_offset-100))  &  (adc_rawdata_buffer[start_zerophase] < (dc_offset+100)) ) //+- 100 von 4096 ca. 2.5% fehler
		{
			zerophase_dedection_count++; 
		}
		start_zerophase+= 32; 
	}
	if (zerophase_dedection_count == 5) //daten valide 
	{
		return pdTRUE;  //wenn berechnung ok 1
		*zerophase_index = start_zerophase - (5 * 32); //startindex übergeben, - 5*32 um auf startwert zu kommen. 
	}
	else
	{
		return pdFALSE;
	}
}

uint16_t idle_calculate_offset(uint16_t max, uint16_t min)
{
	uint16_t dc_offset = 0; 
	dc_offset = ((max - min) / 2) + min; 
	return dc_offset; 
}


bool idle_get_constant_values(uint32_t index, uint16_t* max, uint16_t* min, uint32_t* zero_index)  //calculate min, max and zerophase index
{
	uint32_t max_index_safe = 0; 
	uint32_t min_index_safe = 0; 
	uint16_t y1 = 0;
	uint16_t y2 = 0;
	uint16_t max_safe = 0;
	uint16_t min_safe = 4096; //für erste if abfrage
	uint16_t kontrolle_max = 0;
	uint16_t kontrolle_min = 0;
	for (uint8_t i = 0; i < 64; i++)
	{
		y1 = adc_rawdata_buffer[get_320_index(i + index)];
		y2 = adc_rawdata_buffer[get_320_index(i + index + 1)];
		if ( (y2 > y1) & (y2 > max_safe)  )
		{
			max_safe = y2;
			max_index_safe = i + index + 1;
		}
		else
		{
			if ( (y2 < y1) & (y2 < min_safe))
			{
				min_safe = y2;
				min_index_safe = i + index + 1;
			}
		}
	}
	if ( (adc_rawdata_buffer[get_320_index(max_index_safe+64)] > (max_safe-100)) & 
		 (adc_rawdata_buffer[get_320_index(max_index_safe+64)] < (max_safe+100)) )  //kontrolle maximalwert +- 100 von 4096 ca. 2.5% fehler --> +64 für nächsten wert in tabelle
	{
		*max = max_safe;
	}
	else
	{
		return pdFALSE; 
	}
	if ( (adc_rawdata_buffer[get_320_index(min_index_safe+64)] > (min_safe-100)) & 
		 (adc_rawdata_buffer[get_320_index(min_index_safe+64)] < (min_safe+100)) )  //kontrolle minimalwert +- 100 von 4096 ca. 2.5% fehler
	{
		*min = min_safe;
	}
	else
	{
		return pdFALSE; 
	}
	*zero_index = max_index_safe - 8; //-8 for zero phase
	return pdTRUE; // 1 = ok, 0 = fehler
}


bool idle_get_min_max(uint32_t index, uint16_t* max, uint16_t* min, uint16_t* max_index, uint16_t* min_index)
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
	uint16_t indexoffset = get_320_index(index);
	bool Error_Flag = pdFALSE; 
	
	
	for (uint8_t i = 0; i < 64; i++)
	{
		y1_index = i+indexoffset;
		y2_index = i+1+indexoffset;
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
		*max = max_safe;
		*max_index = max_index_safe; 
		return pdTRUE;
	}
	if ( (kontrolle_min > (min_safe-100)) & (kontrolle_min < (min_safe+100)) )  //kontrolle minimalwert +- 100 von 4096 ca. 2.5% fehler
	{
		*min = min_safe; 
		*min_index = min_index_safe; 
		return pdTRUE;
	}
	return Error_Flag; // 1 = ok, 0 = fehler
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
