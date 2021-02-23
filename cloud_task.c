#include <stdlib.h>
#include <stdio.h>

#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "cy_wcm.h"
#include "cy_mqtt_api.h"

#include "cloud_task.h"

#include "motor_task.h"

#include "cy_json_parser.h"

static cy_mqtt_t mqtthandle;

static void cloud_connectWifi();
static void cloud_startMQTT();
static void cloud_subscribeMQTT();
static void cloud_mqtt_event_cb( cy_mqtt_t mqtt_handle, cy_mqtt_event_t event, void *user_data);
static cy_rslt_t json_cb(cy_JSON_object_t *json_object, void *arg);


#define CLOUD_WIFI_AP        "ew2021"
#define CLOUD_WIFI_PW        "ew2021ap"
#define CLOUD_WIFI_SECURITY  CY_WCM_SECURITY_WPA2_MIXED_PSK
#define CLOUD_WIFI_BAND      CY_WCM_WIFI_BAND_ANY

#define CLOUD_MQTT_BROKER        "mqtt.eclipseprojects.io"
#define CLOUD_MQTT_CLIENT_PREFIX "arh_drone"
#define CLOUD_MQTT_TOPIC         "arh_motor_speed"

#define MOTOR_KEY "motor"


void cloud_task(void* param)
{
    (void)param;

	cloud_connectWifi();
	cloud_startMQTT();
	cloud_subscribeMQTT();

    for(;;)
    {
		vTaskSuspend(NULL);
	}
}


static void cloud_connectWifi()
{
	cy_rslt_t result;

	cy_wcm_connect_params_t connect_param = {
		.ap_credentials.SSID = CLOUD_WIFI_AP,
		.ap_credentials.password = CLOUD_WIFI_PW,
		.ap_credentials.security = CLOUD_WIFI_SECURITY,
		.static_ip_settings = 0,
		.BSSID = {0},
		.band = CLOUD_WIFI_BAND,
	};
	cy_wcm_config_t config = {.interface = CY_WCM_INTERFACE_TYPE_STA}; // We are a station (not a Access Point)

	cy_wcm_init(&config); // Initialize the connection manager

	printf("\nWi-Fi Connection Manager initialized.\n");

	do
	{
		cy_wcm_ip_address_t ip_address;

		printf("Connecting to Wi-Fi AP '%s'\n", connect_param.ap_credentials.SSID);
		result = cy_wcm_connect_ap(&connect_param, &ip_address);

		if (result == CY_RSLT_SUCCESS)
		{
			printf("Successfully connected to Wi-Fi network '%s'.\n",
					connect_param.ap_credentials.SSID);

			// Print IP Address
			if (ip_address.version == CY_WCM_IP_VER_V4)
			{
				printf("IPv4 Address Assigned: %d.%d.%d.%d\n", (uint8_t)ip_address.ip.v4,
						(uint8_t)(ip_address.ip.v4 >> 8), (uint8_t)(ip_address.ip.v4 >> 16),
						(uint8_t)(ip_address.ip.v4 >> 24));
			}
			else if (ip_address.version == CY_WCM_IP_VER_V6)
			{
				printf("IPv6 Address Assigned: %0X:%0X:%0X:%0X\n", (unsigned int)ip_address.ip.v6[0],
						(unsigned int)ip_address.ip.v6[1], (unsigned int)ip_address.ip.v6[2],
						(unsigned int)ip_address.ip.v6[3]);
			}
			break; /* Exit the for loop once the connection has been made */
		}
		else
		{
			printf("WiFi Connect Failed Retrying\n");
			vTaskDelay(2000); // wait 2 seconds and try again;
		}

	} while (result != CY_RSLT_SUCCESS);
}

static void cloud_startMQTT()
{
	static cy_mqtt_connect_info_t    	connect_info;
	static cy_mqtt_broker_info_t     	broker_info;
    static uint8_t buffer[1024];

	cy_rslt_t result;

	result = cy_mqtt_init();
    broker_info.hostname = CLOUD_MQTT_BROKER;
    broker_info.hostname_len = strlen(broker_info.hostname);
    broker_info.port = 1883;

    result = cy_mqtt_create( buffer, sizeof(buffer),
                              NULL, &broker_info,
                              cloud_mqtt_event_cb, NULL,
                              &mqtthandle );

	CY_ASSERT(result == CY_RSLT_SUCCESS);

	static char clientId[32];
	srand(xTaskGetTickCount());
	snprintf(clientId,sizeof(clientId),"%s%6d",CLOUD_MQTT_CLIENT_PREFIX,rand());
    memset( &connect_info, 0, sizeof( cy_mqtt_connect_info_t ) );
    connect_info.client_id      = clientId;
    connect_info.client_id_len  = strlen(connect_info.client_id);
    connect_info.keep_alive_sec = 60;
    connect_info.will_info      = 0;
	connect_info.clean_session = true;


    result = cy_mqtt_connect( mqtthandle, &connect_info );
	CY_ASSERT(result == CY_RSLT_SUCCESS);
	printf("MQTT Connect Success to %s Client=%s\n",CLOUD_MQTT_BROKER,clientId);

}

static void cloud_subscribeMQTT()
{

	cy_rslt_t result;

	cy_mqtt_subscribe_info_t    sub_msg[1];

	/* Subscribe to motor speed MQTT messages */
    sub_msg[0].qos = 0;
    sub_msg[0].topic = CLOUD_MQTT_TOPIC;
    sub_msg[0].topic_len = strlen(sub_msg[0].topic);

    result = cy_mqtt_subscribe( mqtthandle, sub_msg, 1 );
	CY_ASSERT(result == CY_RSLT_SUCCESS);
	printf("Subscribe Success to Topic %s\n",CLOUD_MQTT_TOPIC);

    /* Register JSON callback function */
    cy_JSON_parser_register_callback(json_cb, NULL);

}

static void cloud_mqtt_event_cb( cy_mqtt_t mqtt_handle, cy_mqtt_event_t event, void *user_data )
{
    cy_mqtt_publish_info_t *received_msg;
    printf( "\nMQTT App callback with handle : %p \n", mqtt_handle );
    (void)user_data;
    switch( event.type )
    {
        case CY_MQTT_EVENT_TYPE_DISCONNECT :
            if( event.data.reason == CY_MQTT_DISCONN_TYPE_BROKER_DOWN )
            {
                printf( "\nCY_MQTT_DISCONN_TYPE_BROKER_DOWN .....\n" );
            }
            else
            {
                printf( "\nCY_MQTT_DISCONN_REASON_NETWORK_DISCONNECTION .....\n" );
            }
            break;
        case CY_MQTT_EVENT_TYPE_PUBLISH_RECEIVE :
            received_msg = &(event.data.pub_msg.received_message);
            printf( "Incoming Publish Topic Name: %.*s\n", received_msg->topic_len, received_msg->topic );
            printf( "Incoming Publish message Packet Id is %u.\n", event.data.pub_msg.packet_id );
            printf( "Incoming Publish Message : %.*s.\n\n", ( int )received_msg->payload_len, ( const char * )received_msg->payload );
			if(memcmp(received_msg->topic, CLOUD_MQTT_TOPIC, strlen(CLOUD_MQTT_TOPIC)) == 0) /* Topic matches the motor speed topic */
			{
					cy_JSON_parser(received_msg->payload, received_msg->payload_len);
			}
			
            break;
        default :
            printf( "\nUNKNOWN EVENT .....\n" );
            break;
    }
}


/* This is the callback from the cy_JSON_parser function. It is called whenever
 * the parser finds a JSON object. */
static cy_rslt_t json_cb(cy_JSON_object_t *json_object, void *arg)
{
	int motorSpeed;

	if(memcmp(json_object->object_string, MOTOR_KEY, json_object->object_string_length) == 0)
	{
		if(json_object->value_type == JSON_NUMBER_TYPE)
		{
			/* Add null termination to the value and then convert to a number */
			char resultString[json_object->value_length + 1];
			memcpy(resultString, json_object->value, json_object->value_length);
			resultString[json_object->value_length] = 0;
			motorSpeed = (uint8_t) atoi(resultString);
			printf("Received speed value from cloud: %d\n", motorSpeed);
			motor_update(motorSpeed);
		}
	}
	return CY_RSLT_SUCCESS;
}
