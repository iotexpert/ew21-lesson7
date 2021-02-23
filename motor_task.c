#include "motor_task.h"

#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "motor_task.h"
#include "cloud_task.h"

#include "tle9879_system.h"
#include "ws2812.h"

static QueueHandle_t motor_value_q;

/*******************************************************************************
* Global constants
*******************************************************************************/
#define		RPM_CHANGE_INTERVAL		(100)
#define		RPM_CHANGE_RATE			(10)
#define		RPM_PERCENT_MIN			(10)

#define  	RPM_MAX 				5500.0
#define  	RPM_MIN 				1000.0

#define		NUM_LEDS				(61)

void motor_task(void* param)
{
    (void)param;

	tle9879_sys_t tle9879_sys;

    uint8_t numberOfBoards = 1;
	BaseType_t rtos_api_result;

	/* LED color array - 10 different sets of colors each with RGB values */
	uint8_t ledColors[7][3] = {
			{ 0,  0,  0},	// Off
			{20,  0, 30},	// Violet
			{ 0,  0, 50},	// Blue
			{ 0, 50,  0},	// Green
			{30, 20,  0},	// Yellow
			{42,  8,  0},	// Orange
			{50,  0,  0},	// Red
	};

	uint8_t ledColorRow = 0;
	uint8_t ledColorRowPrev = 0;

	motor_value_q = xQueueCreate(1,sizeof(int));

    /* Initialize and configure the motor driver */
    tle9879sys_init(&tle9879_sys,
    					CYBSP_D11,
						CYBSP_D12,
						CYBSP_D13,
						NULL,
						CYBSP_D4,
						CYBSP_D5,
						CYBSP_D6,
						CYBSP_D7,
						&numberOfBoards);
    tle9879sys_setMode(&tle9879_sys, FOC, 1, false);
	
	bool motorState=false;
	int currentPercentage=0;
	int desiredPercentage=0;

    /* Initialize LED strips */
    ws2812_init(NUM_LEDS, P10_0, P10_1, P10_2);

    while(1)
    {
    	rtos_api_result = xQueueReceive(motor_value_q, &desiredPercentage, RPM_CHANGE_INTERVAL);

       	/* Value has been received from the queue (i.e. not a timeout) */
		if(rtos_api_result == pdTRUE)
		{
			if(desiredPercentage < RPM_PERCENT_MIN) /* Any value less than 10% will result in stopping the motor */
				desiredPercentage = 0;
		
			if(desiredPercentage>100)
				desiredPercentage = 100;
		}

		if(currentPercentage != desiredPercentage)
		{
			if(abs(currentPercentage-desiredPercentage) < RPM_CHANGE_RATE)
				currentPercentage = desiredPercentage;

			if (currentPercentage < desiredPercentage)
				currentPercentage = currentPercentage + RPM_CHANGE_RATE;
			if (currentPercentage > desiredPercentage)
				currentPercentage = currentPercentage - RPM_CHANGE_RATE;

			if(currentPercentage>0 && motorState==false)
			{
				tle9879sys_setMotorMode(&tle9879_sys, START_MOTOR, 1);
				motorState = true;
			}
			if(currentPercentage == 0 && motorState==true)
			{
				tle9879sys_setMotorMode(&tle9879_sys, STOP_MOTOR, 1);
				motorState = false;
			}

			float motorSpeed = ((float)(currentPercentage-RPM_PERCENT_MIN))/100.0 * (RPM_MAX - RPM_MIN) + RPM_MIN;
			tle9879sys_setMotorSpeed(&tle9879_sys, motorSpeed, 1);
			printf("Current %d%% Desired=%d%% Speed=%f\n",currentPercentage,desiredPercentage,motorSpeed);

			/* Calculate LED color and update if it has changed */
			if(motorState == false)
			{
				/* Turn off LEDs */
				ws2812_setMultiRGB(0, NUM_LEDS-1, 0, 0, 0);
				ws2812_update();
				ledColorRowPrev = 0;
			}
			else
			{
				ledColorRow = 1 + (uint8_t)((( (uint16_t)motorSpeed - (uint16_t)MIN_RPM ) * 5) / ((uint16_t)MAX_RPM - (uint16_t)MIN_RPM)); /* Determine row to use */
				if(ledColorRowPrev != ledColorRow)
				{
					ws2812_setMultiRGB(0, NUM_LEDS-1, ledColors[ledColorRow][0], ledColors[ledColorRow][1], ledColors[ledColorRow][2]);
					ws2812_update();
					ledColorRowPrev = ledColorRow;
				}
			}
		}
    }
}

void motor_update(int speed)
{
	if(motor_value_q)
		xQueueSend(motor_value_q,&speed,0);
}
