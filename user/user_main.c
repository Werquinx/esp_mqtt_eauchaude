/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"

#define LED_GPIO 2
#define LED_GPIO_MUX PERIPHS_IO_MUX_GPIO2_U
#define LED_GPIO_FUNC FUNC_GPIO2
#define DELAY 100 /* milliseconds */

MQTT_Client mqttClient;

LOCAL os_timer_t blink_timer;
LOCAL os_timer_t gpio_timer;
LOCAL uint8_t led_state=0;
LOCAL uint8_t gpio_status=2;

LOCAL void ICACHE_FLASH_ATTR blink_cb(void *arg)
{
	GPIO_OUTPUT_SET(LED_GPIO, led_state);
	led_state ^=1;
}

LOCAL void ICACHE_FLASH_ATTR gpio_check(void *arg)
{

    uint8_t gpio_read = GPIO_INPUT_GET(12);

	if (gpio_read != gpio_status){
		gpio_status = gpio_read;
		if (gpio_read == 0){
			GPIO_OUTPUT_SET(LED_GPIO, 1);
			MQTT_Publish(&mqttClient, "/xwhome/eauchaude", "ON", 2, 0, 0);
			INFO("Pompe ON\r\n");
		}
		else {
			GPIO_OUTPUT_SET(LED_GPIO, 0);
			MQTT_Publish(&mqttClient, "/xwhome/eauchaude", "OFF", 3, 0, 0);
			INFO("Pompe OFF\r\n");
		}
	}


}



void wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	} else {
		MQTT_Disconnect(&mqttClient);
	}
}
void mqttConnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");
	MQTT_Subscribe(client, "/xwhome/eauchaude/?", 0);
//	MQTT_Subscribe(client, "/mqtt/topic/1", 1);
//	MQTT_Subscribe(client, "/mqtt/topic/2", 2);
//
//	MQTT_Publish(client, "/mqtt/topic/0", "hello0", 6, 0, 0);
//	MQTT_Publish(client, "/mqtt/topic/1", "hello1", 6, 1, 0);
//	MQTT_Publish(client, "/mqtt/topic/2", "hello2", 6, 2, 0);

	os_timer_disarm(&blink_timer);
	MQTT_Publish(client, "/xwhome/eauchaude", "INIT", 4, 0, 0);

    os_timer_disarm(&gpio_timer);
    os_timer_setfn(&gpio_timer, (os_timer_func_t *)gpio_check, NULL);
    os_timer_arm(&gpio_timer, 1000, 1);


}

void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
}

void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char *topicBuf = (char*)os_zalloc(topic_len+1),
			*dataBuf = (char*)os_zalloc(data_len+1);

	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;

	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;

	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);

	if (GPIO_INPUT_GET(12) == 0){
		GPIO_OUTPUT_SET(LED_GPIO, 1);
		MQTT_Publish(&mqttClient, "/xwhome/eauchaude", "ON", 2, 0, 0);
		INFO("Pompe ON\r\n");
	}
	else {
		GPIO_OUTPUT_SET(LED_GPIO, 0);
		MQTT_Publish(&mqttClient, "/xwhome/eauchaude", "OFF", 3, 0, 0);
		INFO("Pompe OFF\r\n");
	}



	os_free(topicBuf);
	os_free(dataBuf);
}


void user_init(void)
{


	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	os_delay_us(1000000);

	// Configure pin as a GPIO
	PIN_FUNC_SELECT(LED_GPIO_MUX, LED_GPIO_FUNC);      // Set GPIO2 low output
	GPIO_OUTPUT_SET(LED_GPIO, 0);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);                // Set GPIO12 function
    gpio_output_set(0, 0, 0, GPIO_ID_PIN(12));     //input
	// Set up a timer to blink the LED
	// os_timer_disarm(ETSTimer *ptimer)
	os_timer_disarm(&blink_timer);
	// os_timer_setfn(ETSTimer *ptimer, ETSTimerFunc *pfunction, void *parg)
	os_timer_setfn(&blink_timer, (os_timer_func_t *)blink_cb, (void *)0);
	// void os_timer_arm(ETSTimer *ptimer,uint32_t milliseconds, bool repeat_flag)
	os_timer_arm(&blink_timer, DELAY, 1);

	CFG_Load();

	wifi_set_phy_mode (PHY_MODE_11B);
	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);

	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);
	//MQTT_InitConnection(&mqttClient, "192.168.11.122", 1880, 0);

	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
	//MQTT_InitClient(&mqttClient, "client_id", "user", "pass", 120, 1);

	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);


	INFO("\r\nSystem started ...\r\n");
//	os_printf("SDK version:%d.%d.%d\n", SDK_VERSION_MAJOR, SDK_VERSION_MINOR, SDK_VERSION_REVISION);
}
