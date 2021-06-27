/*-------------------------------------------------------------------------------*/
//Project name: FarmIOT
//Coder: Mr.RIDE
//Start code: 2021/06/20
//Last Edied: 
/*-------------------------------------------------------------------------------*/
#ifndef MAIN_FARMIOT_H_
#define MAIN_FARMIOT_H_
/********************************** BEGIN CODE ***********************************/
#include <Arduino.h>
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

#define TASK_SHORT_TIME                     300     //300ms
#define TASK_LONG_TIME                      1000    //1000ms

// Function Prototype
void systemInit();
void variableInit();
void networkInit();
void exINTbuttonInit();

void taskShortTimeSchedule();
void taskLongTimeSchedule();

void mainControl();
void checkButtonControlManu();
void readSensorDHT();
void displaySystemInfor();
void blinkLED();
void controlAutoMode();

void syncControlSystem(uint8_t control_element);
void updateDataToServerCycle1000ms();
void sendSetResultToServer(uint8_t device, uint8_t message);
void updateCtrSystemValue(uint8_t system_element, uint8_t value);
void togglePump();
bool controlSetPumpState(bool set_status);
void toggleLight();
bool controlSetLightState(bool set_status);
String converStrCtrMode(uint8_t control_mode);
String converStrConectSts(bool sts);

void pushDataToThingspeak();


/*********************************** END CODE ************************************/
#endif
/*-------------------------------------------------------------------------------*/
//EOF
/*-------------------------------------------------------------------------------*/