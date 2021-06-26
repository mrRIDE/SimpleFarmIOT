
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

/*-------------------------------------------------------------*/
/* Define IO Port */
#define OLED_SCL		9			//GPIO9
#define OLED_SDA		10			//GPIO10
#define OLED_ID			0x3C		//I2c Address

#define DHT_PIN			12			//GPIO12
#define DHT_TYPE		DHT11

#define CTR_PUMP		5			//GPIO5
#define CTR_LIGHT		0			//GPIO0
#define CTR_BTNMODE 	4			//GPIO4
#define CTR_BTNCTR		14			//GPIO15

#define CTR_MODE_AUTO		0
#define CTR_MODE_MANU_PUMP	1
#define CTR_MODE_MANU_LIGH	2

#define CTR_TEMP_MIN	17
#define CTR_TEMP_MAX	20
#define CTR_HUMD_MIN	70
#define CTR_HUMD_MAX	80

#define ON				1
#define OFF				0

/* Define Name */
// device name
#define	DV_MODE			1
#define DV_PUMP			2
#define DV_LIGHT		3

// message control device
#define SUCCEEDED_SET_MODE_AUTO				1
#define FAILED_SET_MODE_AUTO				2
#define SUCCEEDED_SET_MODE_MANU_PUMP		3
#define FAILED_SET_MODE_MANU_PUMP			4
#define SUCCEEDED_SET_MODE_MANU_LIGH		5
#define FAILED_SET_MODE_MANU_LIGH			6

#define SUCCEEDED_SET_PUMP_ON				1
#define FAILED_SET_PUMP_ON					2
#define SUCCEEDED_SET_PUMP_OFF				3
#define FAILED_SET_PUMP_OFF					4

#define SUCCEEDED_SET_LIGH_ON				1
#define FAILED_SET_LIGH_ON					2
#define SUCCEEDED_SET_LIGH_OFF				3
#define FAILED_SET_LIGH_OFF					4

#define CURRENT_MODE_STS_AUTO				1
#define CURRENT_MODE_STS_MANU_PUMP			2
#define CURRENT_MODE_STS_MANU_LIGH			3
#define CURRENT_PUMP_STS_ON					4
#define CURRENT_PUMP_STS_OFF				5
#define CURRENT_LIGH_STS_ON					6
#define CURRENT_LIGH_STS_OFF				7

/*-------------------------------------------------------------*/
/* Object Declare */

const char* ssid = "IODATA-b0cb48-2G";
const char* pass = "3316246269881";

SSD1306 displayOLED(OLED_ID, OLED_SDA, OLED_SCL);
DHT sensorDHT(DHT_PIN, DHT_TYPE);

//Ticker ticker10ms;
//Ticker ticker100ms;
//Ticker ticker1000ms;

WebSocketsClient webSocket;
const char* host = "192.168.0.6";			//use cmd ipconfig to know localhost ip
const int port = 7777;

/* Global Variable Declare */
struct sensor_dht_data
{
	float humd;
	float temp;

} DHTDATA;

struct control_mode
{
	uint8_t ctrMode;			//0: Auto, 1: Manual
	uint8_t pumpSts;
	uint8_t lightSts;

} CTRLSYSTEM;

/*-------------------------------------------------------------*/
/* Function Prototype */
void systemInit();
void variableInit();
void task10ms_Schedule();
void task100ms_Schedule();
void task1000ms_Schedule();
void mainControl();

void readSensorDHT();
void displaySystemInfor();
void blinkLED();
bool isButtonPress_3000ms();
bool isButtonPress_150ms();
void controlAutoMode();
void controlManualMode(uint8_t manu_mode);
void checkChangeMode();
void selectControlMode();

void syncControlSystem(uint8_t control_element);
void updateDataToServerCycle1000ms();
void sendSetResultToServer(uint8_t device, uint8_t message);
void updateCtrSystemValue(uint8_t system_element, uint8_t value);
void togglePump();
bool controlSetPumpState(bool set_status);
void toggleLight();
bool controlSetLightState(bool set_status);
String returnStrCtrMode(uint8_t control_mode);

/*-------------------------------------------------------------*/
/* Task List */
void task10ms_Schedule()
{
	//Serial.printf("task10ms: %d \n", millis());
	mainControl();

}

void task100ms_Schedule()
{
	//Serial.printf("task100ms: %d \n", millis());
	blinkLED();
	readSensorDHT();
	displaySystemInfor();

}

void task1000ms_Schedule()
{
	updateDataToServerCycle1000ms();
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length)
{

	switch (type)
	{
	case WStype_DISCONNECTED:
		Serial.println("[WSc]Disconneted!");
		break;
	case WStype_CONNECTED:
		Serial.println("[WSc]Connected!");
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
	switch (CTRLSYSTEM.pumpSts)
	{
	case ON:
		webSocket.sendTXT("CURRENT_PUMP_STS_ON");
		break;
	case OFF:
		webSocket.sendTXT("CURRENT_PUMP_STS_OFF");
		break;
	default:
		break;
	}

	// update current Light status
	switch (CTRLSYSTEM.lightSts)
	{
	case ON:
		webSocket.sendTXT("CURRENT_LIGH_STS_ON");
		break;
	case OFF:
		webSocket.sendTXT("CURRENT_LIGH_STS_OFF");
		break;
	default:
		break;
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


/*-------------------------------------------------------------*/
// the setup function runs once when you press reset or power the board
void setup() {
	systemInit();
	variableInit();

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
	delay(100);

	webSocket.onEvent(webSocketEvent);
	delay(500);

	updateDataToServerCycle1000ms();
	delay(200);

	//ticker10ms.attach_ms_scheduled(10, task10ms_Schedule);
	//ticker100ms.attach_ms_scheduled(100, task100ms_Schedule);
	//ticker1000ms.attach_ms_scheduled(1000, task1000ms_Schedule);
	//delay(100);

}

void testBlinkLED()
{
	for (int i = 0; i < 6; i++)
	{
		digitalWrite(LED_BUILTIN, HIGH);
		delay(30);
		digitalWrite(LED_BUILTIN, LOW);
		delay(30);
	}
}

// the loop function runs over and over again until power down or reset
void loop() {
	//Use Ticker 10ms. 100ms for control OS
	mainControl();
	blinkLED();
	readSensorDHT();
	displaySystemInfor();
	updateDataToServerCycle1000ms();
	webSocket.loop();
  delay(500);

}

/*-------------------------------------------------------------*/
void mainControl()
{
	checkChangeMode();
	selectControlMode();

}

void selectControlMode()
{
	switch (CTRLSYSTEM.ctrMode)
	{
	case CTR_MODE_AUTO:
		controlAutoMode();
		break;
	case CTR_MODE_MANU_PUMP:
		controlManualMode(CTR_MODE_MANU_PUMP);
		break;
	case CTR_MODE_MANU_LIGH:
		controlManualMode(CTR_MODE_MANU_LIGH);
		break;
	default:
		break;
	}
}

void checkChangeMode()
{
	if (isButtonPress_3000ms())
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

void controlManualMode(uint8_t manu_mode)
{
	if (isButtonPress_150ms())
	{
		switch (manu_mode)
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
	}
}

bool isButtonPress_150ms()
{
	static uint16_t pressCnt = 0;

	//Serial.printf("press100ms: %d, mili: %d \n", pressCnt, millis());
	if (digitalRead(CTR_BTNCTR) == 0)
	{
		if (pressCnt < 20)
		{
			pressCnt++;
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		pressCnt = 0;
		return false;
	}

}

bool isButtonPress_3000ms()
{
	static uint16_t pressCnt = 0;

	//Serial.printf("press3000ms: %d, mili: %d \n", pressCnt, millis());
	if (digitalRead(CTR_BTNMODE) == 0)
	{
		if (pressCnt < 20)
		{
			pressCnt++;
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		pressCnt = 0;
		return false;
	}

}

void displaySystemInfor()
{
	displayOLED.clear();
	displayOLED.setFont(ArialMT_Plain_10);
	displayOLED.drawString(0, 0, "__FARM SYSTEM__");
	displayOLED.drawString(0, 10, "Mode: " + returnStrCtrMode(CTRLSYSTEM.ctrMode));
	displayOLED.drawString(0, 20, "Temp:" + String(DHTDATA.temp, 1) + "*C");
	displayOLED.drawString(0, 30, "Humd:" + String(DHTDATA.humd, 1) + "%");
	displayOLED.drawString(0, 40, "PumpSts: " + String(!CTRLSYSTEM.pumpSts) + "LighSts: " + String(!CTRLSYSTEM.lightSts));
	displayOLED.display();

}

String returnStrCtrMode(uint8_t control_mode)
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

void variableInit()
{
	DHTDATA.humd = 0;
	DHTDATA.temp = 0;

	CTRLSYSTEM.ctrMode = CTR_MODE_AUTO;
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
	pinMode(CTR_BTNMODE, INPUT);
	pinMode(CTR_BTNCTR, INPUT);

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

	delay(2000);

}