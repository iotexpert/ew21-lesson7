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

static QueueHandle_t motor_value_q;

/*******************************************************************************
* Global constants
*******************************************************************************/
#define		RPM_CHANGE_INTERVAL		(100)
#define		RPM_CHANGE_RATE			(10)
#define		RPM_PERCENT_MIN			(10)

#define  	RPM_MAX 				5500.0
#define  	RPM_MIN 				1000.0

void motor_task(void* param)
{
    (void)param;

	tle9879_sys_t tle9879_sys;

    uint8_t numberOfBoards = 1;
	BaseType_t rtos_api_result;

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
		}
    }
}

void motor_update(int speed)
{
	xQueueSend(motor_value_q,&speed,0);
}
