/*
 * QAMDecGen.c
 *
 * Created: 20.03.2018 18:32:07
 * Author : chaos
 */ 

//#include <avr/io.h>
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
#include "stack_macros.h"

#include "mem_check.h"

#include "init.h"
#include "utils.h"
#include "errorHandler.h"
#include "NHD0420Driver.h"

#include "qaminit.h"
#include "qamgen.h"
#include "qamdec.h"


extern void vApplicationIdleHook( void );
void vLedBlink(void *pvParameters);

TaskHandle_t ledTask;

void vApplicationIdleHook( void )
{	
	
}

int main(void)
{
    resetReason_t reason = getResetReason();

	vInitClock();
	vInitDisplay();
	
	initDAC();
	initDACTimer();
	initGenDMA();
	initADC();
	initADCTimer();
	initDecDMA();
	
	PORTF.DIRSET = PIN0_bm; /*LED1*/
	PORTF.DIRSET = PIN1_bm; /*LED2*/
	PORTF.DIRSET = PIN2_bm; /*LED3*/
	PORTF.DIRSET = PIN3_bm; /*LED4*/

	
	
	xTaskCreate(vQuamGen, NULL, configMINIMAL_STACK_SIZE+500, NULL, 3, NULL);
	xTaskCreate(vQuamDec, NULL, configMINIMAL_STACK_SIZE+100, NULL, 3, NULL);  //led 1
	xTaskCreate(vQuamDecAnalysis, NULL, configMINIMAL_STACK_SIZE+700, NULL, 2, NULL);  //LED 2
	xTaskCreate(vProtocolDecoder, NULL, configMINIMAL_STACK_SIZE+700, NULL, 2, NULL);  //LED 3
	
	
	
	//char received_string[24];
	//char received_string[11] = "HelloWorld\0";
	//
	vDisplayClear();
	//vDisplayWriteStringAtPos(0,0,"FreeRTOS 10.0.1");
	//vDisplayWriteStringAtPos(1,0,"EDUBoard 1.0");
	//vDisplayWriteStringAtPos(2,0,"QAMDECGEN-Base");
	//vDisplayWriteStringAtPos(3,0,"ResetReason: %d", reason);
	
	//
	//vDisplayWriteStringAtPos(0,0,"received string");
	////vDisplayWriteStringAtPos(1,0,"%s", received_string[0]);
	//vDisplayWriteStringAtPos(1,0,"%s", &received_string[0]);
	//vDisplayWriteStringAtPos(1,0,"%d", &received_string[0]);
	//
		//char received_string[24];
		//for(uint8_t i = 0; i < protocol_length; i++)
		//{
			//received_string[i] =   symbol_array[8 + 0 +(i * 4) ] << 0;
			//received_string[i] +=  symbol_array[8 + 1 +(i * 4) ] << 2;
			//received_string[i] +=  symbol_array[8 + 2 +(i * 4) ] << 4;
			//received_string[i] +=  symbol_array[8 + 3 +(i * 4) ] << 6;
		//}
		//received_string[23] = "\0";
		//vDisplayClear();
		//vDisplayWriteStringAtPos(0,0,"received string");
		//vDisplayWriteStringAtPos(1,0,"%s", received_string[0]);
	//
	
	vTaskStartScheduler();
	return 0;
}
