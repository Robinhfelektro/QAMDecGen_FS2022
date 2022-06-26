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
QueueHandle_t receive_symbol_queue;


uint16_t adc_rawdata_buffer[320] = {};  //32 * 16bit var * 10 ms
uint32_t raw_data_buffer_index = 0; 	

TickType_t xLastWakeTime;


//Funktionen initialisieren
void send_to_data_buffer(uint16_t Data, uint16_t index);
uint16_t idle_calculate_offset(uint16_t max, uint16_t min);
uint32_t get_320_index(uint32_t index);
bool idle_get_constant_values(uint32_t index, uint16_t* max, uint16_t* min, uint32_t* zero_index);
bool idle_check_all_zerophase_new(uint32_t* zero_index, uint16_t dc_offset, uint8_t check_strength);
void process_symbol_array(uint8_t* symbol_array, uint8_t protocol_length);
bool check_value_inrange(uint16_t value, uint16_t reference, uint16_t range);
bool decode_ringbuffer_symbol_new(uint32_t* zero_index, uint16_t max_value, uint16_t min_value, uint16_t dc_offset);
bool decode_symbol_return( uint32_t* zero_index, uint16_t max_value, uint16_t min_value, uint16_t dc_offset, uint8_t* symbol_return);


#define PRS_IDLE				0
#define PRS_1SBIT_CHECKED		1  
#define PRS_2SBIT_CHECKED		2
#define PRS_RECEIVED_DATA		3
#define PRS_CHECK_SUM			4


void vProtocolDecoder(void* pvParameters)
{
	receive_symbol_queue = xQueueCreate( 50, sizeof(uint8_t) );  //datenarray
	uint8_t receive_symbol = 0; 
	uint8_t lokal_receive_array[200]; //200 symbol --> 400 bit --> 50 byte
	uint8_t receive_index = 0; 
	uint8_t protocol_receive_state =  PRS_IDLE;
	uint8_t protocol_length = 0; 
	uint8_t protocol_id = 0; 
	uint8_t checksum_received = 0;
	uint8_t checksum_calculated = 0; 
	
	for(;;)
	{
		PORTF.OUTCLR = PIN2_bm;
		while(uxQueueMessagesWaiting(receive_symbol_queue) > 0) 
		{
			if(xQueueReceive(receive_symbol_queue, &receive_symbol , portMAX_DELAY) == pdTRUE) 
			{
				switch(protocol_receive_state)
				{
					case PRS_IDLE:
						if(receive_symbol == 1)  //1. Start Symbol
						{
							protocol_receive_state = PRS_1SBIT_CHECKED; 
							lokal_receive_array[receive_index] = receive_symbol; 
							receive_index++; 
						}
					break; 
				
					case PRS_1SBIT_CHECKED:      
						if(receive_symbol == 2)  //2. Start Symbol
						{
							protocol_receive_state = PRS_2SBIT_CHECKED;
							lokal_receive_array[receive_index] = receive_symbol;
							receive_index++;
						}
						else
						{
							protocol_receive_state = PRS_IDLE; 
							receive_index = 0;
						}
					break;
					case PRS_2SBIT_CHECKED:
					
						//symbol 0,1 start
						//2 und 3 ID
						//4,5,6,7 length
						lokal_receive_array[receive_index] = receive_symbol;
						receive_index++;
					
						if(receive_index == 8)
						{
							protocol_length =  lokal_receive_array[4] << 0;
							protocol_length += lokal_receive_array[5] << 2;
							protocol_length += lokal_receive_array[6] << 4;
							protocol_length += lokal_receive_array[7] << 6;
							protocol_receive_state = PRS_RECEIVED_DATA; 
						}
					break; 
					case PRS_RECEIVED_DATA:
				
						lokal_receive_array[receive_index] = receive_symbol;
						receive_index++;
						if (receive_index == ((protocol_length*4) + 8))
						{
							protocol_receive_state = PRS_CHECK_SUM; 
						}
					break;
					case PRS_CHECK_SUM:
				
						lokal_receive_array[receive_index] = receive_symbol;
						receive_index++;
						if (receive_index == (12 + (protocol_length * 4)) )    //	fix value = 12 symbol --> protokoll length,  10 * 4 symbol  --> berechnet											
						{
							checksum_received =  lokal_receive_array[8 + 0 +(protocol_length * 4) ] << 0;    
							checksum_received += lokal_receive_array[8 + 1 +(protocol_length * 4) ] << 2;
							checksum_received += lokal_receive_array[8 + 2 +(protocol_length * 4) ] << 4;
							checksum_received += lokal_receive_array[8 + 3 +(protocol_length * 4) ] << 6;
					
							//Checksumme
							for(int i = 0; i<8+(protocol_length*4); i++) 
							{
								checksum_calculated += lokal_receive_array[i];
							}
							if (checksum_calculated == checksum_received )
							{
								process_symbol_array(&lokal_receive_array[0], protocol_length);  //protocol_length
							}
							protocol_receive_state = PRS_IDLE;
							receive_index = 0; 
						}
					break;
				}
			}
			else
			{
				vTaskDelay(1);
			}
		}
		PORTF.OUTSET = PIN2_bm;
		vTaskDelay( 10 / portTICK_RATE_MS );
	}
}

void process_symbol_array(uint8_t* symbol_array, uint8_t protocol_length)
{
	char received_string[24];
	for(uint8_t i = 0; i < protocol_length; i++)
	{
		received_string[i] =   symbol_array[8 + 0 +(i * 4) ] << 0;
		received_string[i] +=  symbol_array[8 + 1 +(i * 4) ] << 2;
		received_string[i] +=  symbol_array[8 + 2 +(i * 4) ] << 4;
		received_string[i] +=  symbol_array[8 + 3 +(i * 4) ] << 6;
	}
	for (uint8_t i = 0; i <(24 - protocol_length); i++) //setzt rest auf 0
	{
		received_string[i+protocol_length] = 0; 
	}
	taskENTER_CRITICAL();  //test mit critical section für lcd --> gaaaht nöd
	vDisplayClear();
	vDisplayWriteStringAtPos(0,0,"received string");
	vDisplayWriteStringAtPos(1,0,"%s", &received_string[0]);
	taskEXIT_CRITICAL();
	//vDisplayWriteStringAtPos(1,0,"ResetReason: %s", received_string[0]);  //id übergeben
}



#define STATE_IDLE_INIT 1
#define STATE_DECODE_RINGBUFFER 2
void vQuamDecAnalysis(void* pvParameters)
{
	
	xLastWakeTime = xTaskGetTickCount();
	uint8_t State_Switch = 1; 
	
	uint16_t idle_max_value = 0; 
	uint16_t idle_min_value = 0; 
	uint16_t first_zerophase = 0; 
	uint16_t DC_Offset = 0; 
	
	uint8_t sanity_check; 
	uint32_t decoder_index = 0; 
	uint32_t zero_phaseindex = 0; 
	uint8_t while_break = 0; 
	
	while (1)
	{
		PORTF.OUTCLR = PIN1_bm; 
		switch(State_Switch)
		{
			
			case STATE_IDLE_INIT:
				
				if (raw_data_buffer_index >= 10000)
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
								State_Switch = STATE_DECODE_RINGBUFFER; 
								decoder_index = zero_phaseindex;
							}
							else
							{
								sanity_check = 0; 
							}
						}
					}
				}
			break; 
			
			case STATE_DECODE_RINGBUFFER:	
						while( (decoder_index + 40) < raw_data_buffer_index)
						{
							if (decode_ringbuffer_symbol_new(&decoder_index, idle_max_value, idle_min_value, DC_Offset) == pdFALSE)
							{
								State_Switch = STATE_IDLE_INIT;  //fehler erkannt
							}
							decoder_index+= 32;
							//sanity check funktion 1 mal 
						}
			break; 
			default:
			break; 
		}
		PORTF.OUTSET = PIN1_bm;
		vTaskDelayUntil( &xLastWakeTime, 3 / portTICK_RATE_MS);
	}
}

uint8_t global_testarray[100];
bool decode_ringbuffer_symbol_new(uint32_t* zero_index, uint16_t max_value, uint16_t min_value, uint16_t dc_offset)
{
	static uint8_t i = 0; 
	uint8_t decoded_symbol = 0; 
	if (decode_symbol_return(zero_index, max_value, min_value, dc_offset, &decoded_symbol))
	{
		//get_phase_maxindex()         //in dieser funktion syymbole generieren --> neuer namen noch
		xQueueSend(receive_symbol_queue, &decoded_symbol, 0);
		
		global_testarray[i] = decoded_symbol;  
		
		i++; 
		if (i >= 100)
		{
			i = 0; 
		}
		
		return pdTRUE; 
	}	
	return pdFALSE; 
}


#define DECODE_RANGE 200
#define CORRECTION_RANGE 250
bool decode_symbol_return( uint32_t* zero_index, uint16_t max_value, uint16_t min_value, uint16_t dc_offset, uint8_t* symbol_return)
{
	uint8_t check = 0;
	uint16_t test = 0;
	test = adc_rawdata_buffer[get_320_index(*zero_index + 8)];
	uint16_t half_ampl_pos = ((max_value - dc_offset) >> 1) + dc_offset;  //durch 2, schneller und weniger leistung
	uint16_t half_ampl_neg = dc_offset - ((max_value - dc_offset) >> 1);
	
	if (check_value_inrange(adc_rawdata_buffer[get_320_index(*zero_index)], dc_offset, 500 ) == 0) // 0 durchgang überprüfen +- 25 % abbweichung +-100
	{
		test = adc_rawdata_buffer[get_320_index(*zero_index)];
		return pdFALSE;
	}
	
	/*
	0 = 100%, 0°         00
	1 = 100%, 180°       01
	2 = 50%, 0°			 10
	3 = 50%, 180°		 11
	*/
	//uint16_t debug = adc_rawdata_buffer[get_320_index(zero_index)]; 
	//uint16_t debug1 = adc_rawdata_buffer[get_320_index(zero_index+8)]; 

	if (check_value_inrange(adc_rawdata_buffer[get_320_index(*zero_index + 8)], max_value, DECODE_RANGE )) // 0 
	{
		*symbol_return = 0;
		check++; 
	}
	if (check_value_inrange(adc_rawdata_buffer[get_320_index(*zero_index + 8)], min_value, DECODE_RANGE )) // 0
	{
		*symbol_return = 1;
		check++; 
	}
	if (check_value_inrange(adc_rawdata_buffer[get_320_index(*zero_index + 8)], half_ampl_pos, DECODE_RANGE)) // 0
	{
		*symbol_return = 2;
		check++; 
	}
	if (check_value_inrange(adc_rawdata_buffer[get_320_index(*zero_index + 8)], half_ampl_neg, DECODE_RANGE)) // 0
	{
		*symbol_return = 3;
		check++; 
	}
	if ( (*symbol_return == 0) || (*symbol_return == 2) ) //2 mal 0 phase, (0 index + 16, kontrollieren --> grösser oder kleiner dc offset)
	{
		if ( adc_rawdata_buffer[get_320_index(*zero_index + 16)] > (dc_offset + CORRECTION_RANGE) )
		{
			*zero_index = *zero_index + 1;
		}
		else if ( adc_rawdata_buffer[get_320_index(*zero_index + 16)] < (dc_offset - CORRECTION_RANGE ))
		{
			*zero_index = *zero_index - 1;
		}
	}
	if ((*symbol_return == 1) || (*symbol_return == 3)) //2 mal 180°
	{
		if ( adc_rawdata_buffer[get_320_index(*zero_index + 16)] > (dc_offset + CORRECTION_RANGE) )
		{
			*zero_index = *zero_index - 1;
		}
		else if ( adc_rawdata_buffer[get_320_index(*zero_index + 16)] < (dc_offset - CORRECTION_RANGE) )
		{
			*zero_index = *zero_index + 1;
		}
	}
	if (check == 0)
	{
		return pdFALSE;
	}
	return pdTRUE;
}


bool check_value_inrange(uint16_t value, uint16_t reference, uint16_t range)
{
	if ( (value > (reference - range)) & (value < (reference + range)) ) 
	{
		return pdTRUE;
	}
	return pdFALSE;
}



uint32_t get_320_index(uint32_t index)
{
	return index % 320;
}

void send_to_data_buffer(uint16_t Data, uint16_t index)
{
	adc_rawdata_buffer[index] = Data;
	
	raw_data_buffer_index++;
	//uint16_t index = raw_data_buffer_index % 320;
	//adc_rawdata_buffer[raw_data_buffer_index % 320] = Data;
	//
}

uint16_t get_data_from_buffer(uint32_t index_adress)
{
	return adc_rawdata_buffer[index_adress % 320];
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
	return pdFALSE;
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
	if ( (adc_rawdata_buffer[get_320_index(max_index_safe+64)] > (max_safe-200)) & 
		 (adc_rawdata_buffer[get_320_index(max_index_safe+64)] < (max_safe+200)) )  //kontrolle maximalwert +- 100 von 4096 ca. 2.5% fehler --> +64 für nächsten wert in tabelle
	{
		*max = max_safe;
	}
	else
	{
		return pdFALSE; 
	}
	if ( (adc_rawdata_buffer[get_320_index(min_index_safe+64)] > (min_safe-200)) & 
		 (adc_rawdata_buffer[get_320_index(min_index_safe+64)] < (min_safe+200)) )  //kontrolle minimalwert +- 100 von 4096 ca. 2.5% fehler
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



void vQuamDec(void* pvParameters)
{
	( void ) pvParameters;
	
	//decoderQueue = xQueueCreate( 4, NR_OF_SAMPLES * sizeof(int16_t) );  //datenarray
	
	while(evDMAState == NULL) {
		vTaskDelay(3/portTICK_RATE_MS);
	}
	
	uint16_t bufferelement[NR_OF_SAMPLES];
	
	xEventGroupWaitBits(evDMAState, DMADECREADY, false, true, portMAX_DELAY);
	
	uint8_t lcd_count = 0; 
	for(;;)
	{
		PORTF.OUTCLR = PIN0_bm; 
		//if (lcd_count == 100)  //test für lcd Ausgabe
		//{
			//taskENTER_CRITICAL();
			//vDisplayClear();
			//vDisplayWriteStringAtPos(0,0,"FreeRTOS 10.0.1");
			//lcd_count = 0; 
			//taskEXIT_CRITICAL();
		//}
		//lcd_count++; 
		PORTF.OUTSET = PIN0_bm; 	
		vTaskDelay(10/portTICK_RATE_MS);
	}
}


void fillDecoderQueue(uint16_t buffer[NR_OF_SAMPLES])
{
	PORTF.OUTCLR = PIN3_bm;
	uint16_t index = raw_data_buffer_index % 320;
	for (uint8_t i = 0; i < 32; i++)
	{
		if (index > 320)
		{
			index = 0;
		}
		adc_rawdata_buffer[index] = buffer[i];
		index++;
		raw_data_buffer_index++;
	}
	PORTF.OUTSET = PIN3_bm; 
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
