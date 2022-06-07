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
bool start_symbol_search(void);
bool idel_check_all_zerophase(uint16_t max_index_check, uint16_t dc_offset, uint16_t* zerophase_index);
void decipher_ringbuffer_1symbol(uint16_t index , uint16_t first_zerophase, uint16_t idle_max_value); //daten auswerten
uint32_t get_320_index(uint32_t index);
bool idle_get_min_max(uint32_t index, uint16_t* max, uint16_t* min, uint16_t* max_index, uint16_t* min_index);
uint8_t get_phase_maxindex( uint16_t index);

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
	#define STATE_IDLE_INIT 1
	#define STATE_WAIT_STARTBIT 2
	#define STATE_START 3
	xLastWakeTime = xTaskGetTickCount();
	
	
	uint8_t State_Switch = 1; 
	uint16_t idle_max_value = 0; 
	uint16_t idle_min_value = 0; 
	uint16_t idle_max_index = 0; 
	uint16_t idle_min_index = 0; 
	uint16_t first_zerophase = 0; 
	uint16_t DC_Offset = 0; 
	
	
	
	bool ErrorTest = pdFALSE; 
	uint16_t Index = 0; 
	uint8_t sanity_check; 
	uint32_t decoder_index = 0; 
	
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
						if (idle_get_min_max(decoder_index, &idle_max_value, &idle_min_value, &idle_max_index, &idle_min_index))  //test für error check
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
							if (idel_check_all_zerophase(idle_max_index, DC_Offset, &first_zerophase))  //wenn 0 phase ok == 1
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
		
		//ms_count_test++;   //irgendwie geht das display nicht
		//if(ms_count_test >= 200)//jede sekunde für test
		//{
			//vDisplayClear();
			//ms_count_test = 0; 
			//vDisplayWriteStringAtPos(3,0,"ResetReason: %d", DC_Offset);
		//}
		
		vTaskDelayUntil( &xLastWakeTime, 5 / portTICK_RATE_MS);
	}
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
	uint8_t get_phase_maxindex( uint16_t index); 
	
	for (uint8_t i = 0; i < 32; i++)
	{
		
	}
}

uint8_t get_symbol(uint16_t max_value , uint16_t max_value_2 , uint8_t phase)
{
	
}

uint8_t get_phase_maxindex( uint16_t index)
{
	/*
		- 0 phase bekannt
		- 32 daten ab 0 phasenindex überprüfen  , 32 ab 0°
		- min max berechnen mit index
		nicht invertiert: 
			- max 0 bis 16 --> 0° phase
			- min 16 bis 32 --> 0° phase
		invertiert: 
			- wenn max index nach 16 bis 32 --> phase invertiert
			- wenn min index vor 0 bis 16 --> phase invertiert
			
		- Jetzt mithilfe von phase und min max value herausfinden welches  symbol
		- Die Daten in einem Protokollbufferarry abspeichern --> vielleicht doppelt so gross wie eine übertragung
	*/
	
	if (index > 16)
	{
		return 0; 
	}
	else
	{
		return 160; 
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
