
/*
 Name:		ESP8266_AgricultureIOT.ino
 Created:	2021-06-15 8:54:36 PM
 Author:	MrRide
*/
/*
--------------------プロジェクトの概略内容 (Begin)------------------
--テーマ：イチゴファーム管理システム
--要求：
-温度[17-20*C]以内を制御
-湿度[80-90%]以内を制御
--インタネットで監視・制御できること

※システム分析
--使用Hardware
-Board Control, NetworkComunicate: NODE MCU ESP8266
-Display: OLED 1.25inch
-Sensor: DHT, moil
-Load Device: Pump, Fan, Light
-Software: C, C++

--Server
-Cloud Azure
-Webserver: Protocol: WebscoketIO, NodeJS, javascript, html, css
--------------------プロジェクトの概略内容 (End)--------------------
*/

/*-------------------------------------------------------------*/
/* Include Library */
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <SSD1306.h>
#include <Wire.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>

#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>

#include "MainFarmIOT.h"
/*-------------------------------------------------------------*/
/* Object Declare */

const char* ssid = "IODATA-b0cb48-2G";
const char* pass = "3316246269881";

SSD1306 displayOLED(OLED_ID, OLED_SDA, OLED_SCL);
DHT sensorDHT(DHT_PIN, DHT_TYPE);

Ticker ticker300ms;
Ticker ticker1000ms;

WebSocketsClient webSocket;
/* Test Local*/
// const char* host = "192.168.0.6";			//use cmd ipconfig to know localhost ip
// const int port = 3000;	// port 3000 for test local

/* Deploy on free server Heroku*/
const char* host = "farm-iot-rider.herokuapp.com";
const int port = 80;		// 80 - Hyper-Text Transfer Protocol (HTTP) mac dinh de kn toi heroku

/* Global Variable Declare */
struct sensor_dht_data
{
	float humd;
	float temp;

} DHTDATA;

struct control_mode
{
	uint8_t ctrMode;			//0: Auto, 1: Manual
	bool pumpSts;
	bool lightSts;

	bool btnModePressFlag;
	bool btnCtrPressFlag;

	bool serverConSts;				// server connection status

} CTRLSYSTEM;

/*-------------------------------------------------------------*/
/* Function Prototype */
void ICACHE_RAM_ATTR handleINT_BTNmode();
void ICACHE_RAM_ATTR handleINT_BTNctr();

/*-------------------------------------------------------------*/
// the setup function runs once when you press reset or power the board
void setup() {
	systemInit();
	variableInit();

	networkInit();
	exINTbuttonInit();

	delay(200);
	updateDataToServerCycle1000ms();

	delay(100);

	ticker300ms.attach_ms_scheduled(TASK_SHORT_TIME, taskShortTimeSchedule);
	ticker1000ms.attach_ms_scheduled(TASK_LONG_TIME, taskLongTimeSchedule);

}


// the loop function runs over and over again until power down or reset
void loop() {
	//do no thing
}

/*-------------------------------------------------------------*/
void taskShortTimeSchedule()
{
	//Serial.printf("task100ms: %d \n", millis());
	mainControl();
	readSensorDHT();
	displaySystemInfor();
	webSocket.loop();

}

void taskLongTimeSchedule()
{
	blinkLED();
	updateDataToServerCycle1000ms();
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length)
{

	switch (type)
	{
	case WStype_DISCONNECTED:
		Serial.println("[WSc]Disconneted!");
		CTRLSYSTEM.serverConSts = false;
		break;
	case WStype_CONNECTED:
		Serial.println("[WSc]Connected!");
		CTRLSYSTEM.serverConSts = true;
		break;
	case WStype_TEXT:
		Serial.printf("[WSc]get text: %s\n", payload);
		if (strcmp((char*)payload, "changeMode") == 0)
		{
			syncControlSystem(DV_MODE);
		}
		else if (strcmp((char*)payload, "toggleLight") == 0)
		{
			syncControlSystem(DV_LIGHT);
		}
		else if (strcmp((char*)payload, "togglePump") == 0)
		{
			syncControlSystem(DV_PUMP);
		}

		break;
	case WStype_BIN:
		Serial.printf("[WSc]get binary length: %u\n", length);
		break;
	default:
		break;
	}
}

void syncControlSystem(uint8_t control_element)
{
	if (control_element == DV_MODE)
	{
		switch (CTRLSYSTEM.ctrMode)
		{
		case CTR_MODE_AUTO:
			updateCtrSystemValue(DV_MODE, CTR_MODE_MANU_PUMP);
			break;
		case CTR_MODE_MANU_PUMP:
			updateCtrSystemValue(DV_MODE, CTR_MODE_MANU_LIGH);
			break;
		case CTR_MODE_MANU_LIGH:
			updateCtrSystemValue(DV_MODE, CTR_MODE_AUTO);	//return auto
			break;
		default:
			break;
		}
		return;
	}
	
	if (control_element == DV_PUMP)
	{
		if (CTRLSYSTEM.ctrMode == CTR_MODE_MANU_PUMP)
		{
			togglePump();
		}
		return;
	}

	if (control_element == DV_LIGHT)
	{
		if (CTRLSYSTEM.ctrMode == CTR_MODE_MANU_LIGH)
		{
			toggleLight();
		}
		return;
	}
}

void updateDataToServerCycle1000ms()
{
	// update current Mode status
	switch (CTRLSYSTEM.ctrMode)
	{
	case CTR_MODE_AUTO: 
		webSocket.sendTXT("CURRENT_MODE_STS_AUTO");
		break;
	case CTR_MODE_MANU_PUMP: 
		webSocket.sendTXT("CURRENT_MODE_STS_MANU_PUMP");
		break;
	case CTR_MODE_MANU_LIGH:
		webSocket.sendTXT("CURRENT_MODE_STS_MANU_LIGH");
		break;
	default:
		break;
	}

	// update current Pump status
	if (CTRLSYSTEM.pumpSts == ON)
	{
		webSocket.sendTXT("CURRENT_PUMP_STS_ON");
	}
	else if (CTRLSYSTEM.pumpSts == OFF)
	{
		webSocket.sendTXT("CURRENT_PUMP_STS_OFF");
	}

	// update current Light status
	if (CTRLSYSTEM.lightSts == true)
	{
		webSocket.sendTXT("CURRENT_LIGH_STS_ON");
	}
	else if (CTRLSYSTEM.lightSts == false)
	{
		webSocket.sendTXT("CURRENT_LIGH_STS_OFF");
	}

	// update current DHT temperature
	String temp = "Temp:" + String(DHTDATA.temp, 1) + "*C";
	String humd = "Humd:" + String(DHTDATA.humd, 1) + "%";

	webSocket.sendTXT(temp);
	webSocket.sendTXT(humd);

}

void sendSetResultToServer(uint8_t device, uint8_t message)
{
	switch (device)
	{
	case DV_MODE:
		switch (message)
		{
		case SUCCEEDED_SET_MODE_AUTO:
			webSocket.sendTXT("SUCCEEDED_SET_MODE_AUTO");
			break;
		case FAILED_SET_MODE_AUTO:
			webSocket.sendTXT("FAILED_SET_MODE_AUTO");
			break;
		case SUCCEEDED_SET_MODE_MANU_PUMP:
			webSocket.sendTXT("SUCCEEDED_SET_MODE_MANU_PUMP");
			break;
		case FAILED_SET_MODE_MANU_PUMP:
			webSocket.sendTXT("FAILED_SET_MODE_MANU_PUMP");
			break;
		case SUCCEEDED_SET_MODE_MANU_LIGH:
			webSocket.sendTXT("SUCCEEDED_SET_MODE_MANU_LIGH");
			break;
		case FAILED_SET_MODE_MANU_LIGH:
			webSocket.sendTXT("FAILED_SET_MODE_MANU_LIGH");
			break;
		default:
			break;
		}
		break;
	case DV_PUMP:
		switch (message)
		{
		case SUCCEEDED_SET_PUMP_ON:
			webSocket.sendTXT("SUCCEEDED_SET_PUMP_ON");
			break;
		case FAILED_SET_PUMP_ON:
			webSocket.sendTXT("FAILED_SET_PUMP_ON");
			break;
		case SUCCEEDED_SET_PUMP_OFF:
			webSocket.sendTXT("SUCCEEDED_SET_PUMP_OFF");
			break;
		case FAILED_SET_PUMP_OFF:
			webSocket.sendTXT("FAILED_SET_PUMP_OFF");
			break;
		default:
			break;
		}
		break;
	case DV_LIGHT:
		switch (message)
		{
		case SUCCEEDED_SET_LIGH_ON:
			webSocket.sendTXT("SUCCEEDED_SET_LIGH_ON");
			break;
		case FAILED_SET_LIGH_ON:
			webSocket.sendTXT("FAILED_SET_LIGH_ON");
			break;
		case SUCCEEDED_SET_LIGH_OFF:
			webSocket.sendTXT("SUCCEEDED_SET_LIGH_OFF");
			break;
		case FAILED_SET_LIGH_OFF:
			webSocket.sendTXT("FAILED_SET_LIGH_OFF");
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

void updateCtrSystemValue(uint8_t system_element, uint8_t value)
{
	switch (system_element)
	{
	case DV_MODE:
		CTRLSYSTEM.ctrMode = value;
		break;
	case DV_PUMP:
		CTRLSYSTEM.pumpSts = value;
		break;
	case DV_LIGHT:
		CTRLSYSTEM.lightSts = value;
		break;
	default:
		break;
	}
}

void togglePump()
{
	uint8_t ctr_state_before;

	ctr_state_before = digitalRead(CTR_PUMP);
	if (ctr_state_before == HIGH)
	{
		//turn ON PUMP
		if (controlSetPumpState(LOW) == true)
		{
			updateCtrSystemValue(DV_PUMP, ON);
			sendSetResultToServer(DV_PUMP, SUCCEEDED_SET_PUMP_ON);
		}
		else
		{
			sendSetResultToServer(DV_PUMP, FAILED_SET_PUMP_ON);
		}
	}
	else if (ctr_state_before == LOW)
	{
		// turn OFF PUMP
		if (controlSetPumpState(HIGH) == true)
		{
			updateCtrSystemValue(DV_PUMP, OFF);
			sendSetResultToServer(DV_PUMP, SUCCEEDED_SET_PUMP_OFF);
		}
		else
		{
			sendSetResultToServer(DV_PUMP, FAILED_SET_PUMP_OFF);
		}
	}
}

bool controlSetPumpState(bool set_status)
{
	digitalWrite(CTR_PUMP, set_status);
	delay(5);
	if (digitalRead(CTR_PUMP) == set_status)
	{
		return true;	// set command OK
	}
	else
	{
		return false;	// set command NG
	}
}

void toggleLight()
{
	uint8_t ctr_state_before;

	ctr_state_before = digitalRead(CTR_LIGHT);
	if (ctr_state_before == HIGH)
	{
		//turn ON LIGHT
		if (controlSetLightState(LOW) == true)
		{
			updateCtrSystemValue(DV_LIGHT, ON);
			sendSetResultToServer(DV_LIGHT, SUCCEEDED_SET_LIGH_ON);
		}
		else
		{
			sendSetResultToServer(DV_LIGHT, FAILED_SET_LIGH_ON);
		}
	}
	else if (ctr_state_before == LOW)
	{
		// turn OFF LIGH
		if (controlSetLightState(HIGH) == true)
		{
			updateCtrSystemValue(DV_LIGHT, OFF);
			sendSetResultToServer(DV_LIGHT, SUCCEEDED_SET_LIGH_OFF);
		}
		else
		{
			sendSetResultToServer(DV_LIGHT, FAILED_SET_LIGH_OFF);
		}
	}
}

bool controlSetLightState(bool set_status)
{
	digitalWrite(CTR_LIGHT, set_status);
	delay(5);
	if (digitalRead(CTR_LIGHT) == set_status)
	{
		return true;	// set command OK
	}
	else
	{
		return false;	// set command NG
	}
}

void mainControl()
{
	checkButtonControlManu();
	if (CTRLSYSTEM.ctrMode == CTR_MODE_AUTO)
	{
		controlAutoMode();
	}

}

void checkButtonControlManu()
{
	if (CTRLSYSTEM.btnModePressFlag == true)
	{
		switch (CTRLSYSTEM.ctrMode)
		{
		case CTR_MODE_AUTO:
			updateCtrSystemValue(DV_MODE, CTR_MODE_MANU_PUMP);
			break;
		case CTR_MODE_MANU_PUMP:
			updateCtrSystemValue(DV_MODE, CTR_MODE_MANU_LIGH);
			break;
		case CTR_MODE_MANU_LIGH:
			updateCtrSystemValue(DV_MODE, CTR_MODE_AUTO);	//return auto
			break;
		default:
			break;
		}

		CTRLSYSTEM.btnModePressFlag = false;	//reset for next check
	}

	if (CTRLSYSTEM.btnCtrPressFlag == true)
	{
		switch (CTRLSYSTEM.ctrMode)
		{
		case CTR_MODE_MANU_PUMP:
			togglePump();
			break;
		case CTR_MODE_MANU_LIGH:
			toggleLight();
			break;
		default:
			break;
		}

		CTRLSYSTEM.btnCtrPressFlag = false;		//reset for next check
	}
}

void controlAutoMode()
{
	if (DHTDATA.temp < CTR_TEMP_MIN)
	{
		controlSetLightState(LOW);			// turn ON light
		updateCtrSystemValue(DV_LIGHT, ON);
	}
	else if (DHTDATA.temp > CTR_LIGHT)
	{
		controlSetLightState(HIGH);			// turn OFF light
		updateCtrSystemValue(DV_LIGHT, OFF);
	}

	if (DHTDATA.humd < CTR_HUMD_MIN)
	{
		controlSetPumpState(LOW);			// turn  ON pump
		updateCtrSystemValue(DV_PUMP, ON);
	}
	else if (DHTDATA.humd > CTR_PUMP)
	{
		controlSetPumpState(HIGH);			// turn OFF pump
		updateCtrSystemValue(DV_PUMP, OFF);
	}

}

void displaySystemInfor()
{
	displayOLED.clear();
	displayOLED.setFont(ArialMT_Plain_10);
	displayOLED.drawString(0, 0, "__FARM SYSTEM__");
	displayOLED.drawString(0, 10, "Server: " + converStrConectSts(CTRLSYSTEM.serverConSts));
	displayOLED.drawString(0, 20, "Mode: " + converStrCtrMode(CTRLSYSTEM.ctrMode));
	displayOLED.drawString(0, 30, "Temp:" + String(DHTDATA.temp, 1) + "*C");
	displayOLED.drawString(0, 40, "Humd:" + String(DHTDATA.humd, 1) + "%");
	displayOLED.drawString(0, 50, "PumpSts: " + String(CTRLSYSTEM.pumpSts) + "LighSts: " + String(CTRLSYSTEM.lightSts));
	displayOLED.display();

}

String converStrConectSts(bool sts)
{
	String rteString;

	if (sts == true)
	{
	 	rteString = "Connected";
	}
	else
	{
		rteString = "Disconnected";
	}

	return rteString;
}

String converStrCtrMode(uint8_t control_mode)
{
	String rteString;

	switch (control_mode)
	{
	case CTR_MODE_AUTO:
		rteString = "Auto";
		break;
	case CTR_MODE_MANU_LIGH:
		rteString = "Manu-Light";
		break;
	case CTR_MODE_MANU_PUMP:
		rteString = "Manu-Pump";
		break;
	default:
		break;
	}

	return rteString;
}

void readSensorDHT()
{
	float temp_t;
	float humd_t;

	temp_t = sensorDHT.readTemperature();
	humd_t = sensorDHT.readHumidity();

	if (isnan(temp_t))
	{
		Serial.println("Failled to read Temp");
	}
	else
	{
		DHTDATA.temp = temp_t;
	}

	if (isnan(humd_t))
	{
		Serial.println("Failed to read Humd");
	}
	else
	{
		DHTDATA.humd = humd_t;
	}
}

void blinkLED()
{
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void exINTbuttonInit()
{
	attachInterrupt(digitalPinToInterrupt(CTR_BTNMODE), handleINT_BTNmode, FALLING);
	attachInterrupt(digitalPinToInterrupt(CTR_BTNCTR), handleINT_BTNctr, FALLING);
}

void ICACHE_RAM_ATTR handleINT_BTNmode()
{
	if (digitalRead(CTR_BTNMODE) == 0)
	{
		if (digitalRead(CTR_BTNMODE) == 0)
			CTRLSYSTEM.btnModePressFlag = true;
	}
}

void ICACHE_RAM_ATTR handleINT_BTNctr()
{
	if (digitalRead(CTR_BTNCTR) == 0)
	{
		if (digitalRead(CTR_BTNCTR) == 0)
			CTRLSYSTEM.btnCtrPressFlag = true;
	}
}

void networkInit()
{
	Serial.print("Connect to WiFi: ");
	Serial.print(ssid);
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, pass);
	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		Serial.print('.');
	}

	Serial.println();
	Serial.print("Connected. ESP8266 IP: ");
	Serial.println(WiFi.localIP());

	webSocket.begin(host, port);
	delay(500);

	webSocket.onEvent(webSocketEvent);
	delay(500);
}

void variableInit()
{
	DHTDATA.humd = 0;
	DHTDATA.temp = 0;

	CTRLSYSTEM.ctrMode = CTR_MODE_AUTO;
	CTRLSYSTEM.lightSts = false;
	CTRLSYSTEM.pumpSts = false;

	CTRLSYSTEM.btnCtrPressFlag = false;	//not press;
	CTRLSYSTEM.btnCtrPressFlag = false;

	CTRLSYSTEM.serverConSts = false;
}

void systemInit()
{
	// Init Serial for debug
	Serial.begin(115200);
	Serial.println("Init Serial");

	// Init Port IO
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(CTR_PUMP, OUTPUT);
	pinMode(CTR_LIGHT, OUTPUT);

	pinMode(CTR_BTNMODE, INPUT_PULLUP);
	pinMode(CTR_BTNCTR, INPUT_PULLUP);

	digitalWrite(LED_BUILTIN, HIGH);		//Set OFF
	digitalWrite(CTR_LIGHT, HIGH);
	digitalWrite(CTR_PUMP, HIGH);
	Serial.println("Init IO port");

	// Init OLED
	displayOLED.init();
	delay(50);
	displayOLED.flipScreenVertically();
	displayOLED.setFont(ArialMT_Plain_16);
	displayOLED.drawString(0, 5, "_WELCOME_");
	displayOLED.drawString(0, 20, "_Farm_");
	displayOLED.drawString(0, 35, "_System_");
	displayOLED.display();
	Serial.println("Init OLED");

	// Init Sensor DHT
	sensorDHT.begin();
	delay(50);
	readSensorDHT();
	Serial.println("Init Sensor DHT");
	Serial.printf("Frist read: Temp: %.1f(*C) - Humd: %.1f(%%)", DHTDATA.temp, DHTDATA.humd);

	delay(1000);

}