// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license.

#include <cstdarg>
#include <vector>
#include <iostream>     // std::cout, std::ios
#include <sstream>      // std::ostringstream
#include <M5Stack.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "AzureIotHub.h"
#include "Esp32MQTTClient.h"

#define INTERVAL 10000 //既定のメッセージ送信周期(msec)
#define SAMP_MSEC 10   //既定のサンプリング周期 msec

//その他設定
#define DISP_INTERVAL 100 //画面表示周期
#define SPEAKER_PIN 25
#define AMP_FALLEDGE_THRESHOLD  2.0     //立下り閾値検知

//Function prototype
static void hangup(const char *msg);
void disp();

//WiFi初期化完了フラグ
static bool hasWifi = false;
//IoTHubメッセージ送信成功フラグ
static bool mqttConnected = false;
//IoTHubメッセージ送信有効化フラグ
static bool messageSending = true;
//IoTHubメッセージ送信最短周期
static uint64_t send_interval_ms;
//データサンプリング最短周期
static uint64_t sampling_ms;
//IoT Hub接続かどうか(違う場合はMQTTClient)
static bool isIotHub;

//ローカルタイム文字列取得
static String GetLocalTimeString()
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("unknown time");
    }
    char ts_buffer[128];
    strftime(ts_buffer, sizeof(ts_buffer), "%FT%T", &timeinfo);
    Serial.printf("getLocalTime: %s\r\n", ts_buffer);

    return String(ts_buffer);
}

//設定情報
class Settings
{
private:
    //設定ファイル名
    const char *SETTINGS_FILE_NAME = "/settings.json";

public:
    //送信周期(msec)
    int interval = INTERVAL;

    //WiFi接続SSID
    String ssid;

    //WiFi接続パスワード
    String password;

    //String containing Hostname, Device Id & Device Key in the format:
    //  "HostName=<host_name>;DeviceId=<device_id>;SharedAccessKey=<device_key>"
    //  "HostName=<host_name>;DeviceId=<device_id>;SharedAccessSignature=<device_sas_token>"
    //Azure IoT Hub接続文字列
    String connectionString;

    //PubSubClinet接続設定
    //サーバー/ポート/ユーザー/パスワード/ルーティングキー
    String mqttServer;
    int mqttPort = 1883;
    String mqttUser;
    String mqttPass;
    String mqttRoutingKey;
    
    //センササンプリング周期
    int samp_msec = SAMP_MSEC;

    //デバイスIP
    IPAddress ip;
    //サブネット
    IPAddress subnet;
    //デフォルトゲートウェイ
    IPAddress gateway;
    //DNSサーバ
    IPAddress dns;

    //センサ0サンプリング有効フラグ
    bool sensor0enabled = false;
    //センサ0入力ピン
    int sensor0pin;
    //センサ0設備連携キー
    String sensor0userKey;
    //センサ0入力最小電圧
    float sensor0vMin;
    //センサ0入力最大電圧
    float sensor0vMax;
    //センサ0クランプ最大電流
    float sensor0ampMax;
    //センサ0閾値
    float sensor0ampThreshold;
    //センサ0サンプリング回数
    int sensor0sampCount;

    //add by mrRIDE
    unsigned long sensor0waitForNextTimetCount;
    
    //センサ1サンプリング有効フラグ
    bool sensor1enabled = false;
    //センサ1入力ピン
    int sensor1pin;
    //センサ1設備連携キー
    String sensor1userKey;
    //センサ1入力最小電圧
    float sensor1vMin;
    //センサ1入力最大電圧
    float sensor1vMax;
    //センサ1クランプ最大電流
    float sensor1ampMax;
    //センサ1閾値
    float sensor1ampThreshold;
    //センサ1サンプリング回数
    int sensor1sampCount;

    // add by mrRIDE
    unsigned long sensor1waitForNextTimetCount;

    //設定読み込み(from SD Card)
    bool Load()
    {
        File fp = SD.open(SETTINGS_FILE_NAME, FILE_READ);
        if (!fp)
        {
            M5.Lcd.println("not found file settings.json.");
            return false;
        }

        std::string jsonBuffer;
        while (fp.available())
        {
            jsonBuffer += (char)fp.read();
        }
        Serial.println(SETTINGS_FILE_NAME);
        Serial.println(jsonBuffer.c_str());
        Serial.println("");
        Serial.println("deserializeJson");

        StaticJsonDocument<JSON_OBJECT_SIZE(128)> jsonDoc;
        deserializeJson(jsonDoc, jsonBuffer.c_str());

        interval = jsonDoc["interval"];
        Serial.printf("interval: %d\r\n", interval);
        ssid = jsonDoc["ssid"].as<String>();
        Serial.printf("ssid: %s\r\n", ssid.c_str());
        password = jsonDoc["password"].as<String>();
        Serial.printf("password: %s\r\n", password.c_str());
        connectionString = jsonDoc["connectionString"].as<String>();
        Serial.printf("connectionString: %s\r\n", connectionString.c_str());
        mqttServer = jsonDoc["mqttServer"].as<String>();
        Serial.printf("mqttServer: %s\r\n", mqttServer.c_str());
        mqttPort = jsonDoc["mqttPort"];
        Serial.printf("mqttPort: %d\r\n", mqttPort);
        mqttUser = jsonDoc["mqttUser"].as<String>();
        Serial.printf("mqttUser: %s\r\n", mqttUser.c_str());
        mqttPass = jsonDoc["mqttPass"].as<String>();
        Serial.printf("mqttPass: %s\r\n", mqttPass.c_str());
        mqttRoutingKey = jsonDoc["mqttRoutingKey"].as<String>();
        Serial.printf("mqttRoutingKey: %s\r\n", mqttRoutingKey.c_str());

        samp_msec = jsonDoc["samp_msec"];
        Serial.printf("samp_msec: %d\r\n", samp_msec);

        bool validAddress = true;
        const char* address;

        address = (const char *)jsonDoc["ip"];
        Serial.printf("ip: %s\r\n", address);
        validAddress &= ip.fromString(address);

        address = (const char *)jsonDoc["subnet"];
        Serial.printf("subnet: %s\r\n", address);
        validAddress &= subnet.fromString(address);

        address = (const char *)jsonDoc["gateway"];
        Serial.printf("gateway: %s\r\n", address);
        validAddress &= gateway.fromString(address);

        address = (const char *)jsonDoc["dns"];
        Serial.printf("dns: %s\r\n", address);
        validAddress &= dns.fromString(address);

        if (!validAddress)
        {
            Serial.println("ERRROR: Invalid address settings.");
            return false;
        }

        try
        {
            //sensor settings.
            JsonArray ar = jsonDoc["sensors"];
            {
                JsonObject o = ar[0];
                
                bool enabled = o["enabled"];
                Serial.printf("sensors[0]:enabled: %s\r\n", enabled ? "true":"false");
                sensor0enabled = enabled;
                
                int pin = o["pin"];
                Serial.printf("sensors[0]:pin: %d\r\n", pin);
                sensor0pin = pin;
                
                String userKey = o["userKey"];
                Serial.printf("sensors[0]:userKey: %s\r\n", userKey.c_str());
                sensor0userKey = userKey;
                
                float ampThreshold = o["ampThreshold"];
                Serial.printf("sensors[0]:ampThreshold: %f\r\n", ampThreshold);
                sensor0ampThreshold = ampThreshold;

                float vMin = o["vMin"];
                Serial.printf("sensors[0]:vMin: %f\r\n", vMin);
                sensor0vMin = vMin;

                float vMax = o["vMax"];
                Serial.printf("sensors[0]:vMax: %f\r\n", vMax);
                sensor0vMax = vMax;

                float ampMax = o["ampMax"];
                Serial.printf("sensors[0]:ampMax: %f\r\n", ampMax);
                sensor0ampMax = ampMax;
                
                int sampCount = o["sampCount"];
                Serial.printf("sensors[0]:sampCount: %d\r\n", sampCount);
                sensor0sampCount = sampCount;

                // add by mr RIDE
                unsigned long waitForNextTimetCount = o["waitForNextTimetCount"];
                Serial.printf("sensors[0]:waitForNextTimetCount: %d\r\n", waitForNextTimetCount);
                sensor0waitForNextTimetCount = waitForNextTimetCount;
            }
            {
                JsonObject o = ar[1];

                bool enabled = o["enabled"];
                Serial.printf("sensors[1]:enabled: %s\r\n", enabled ? "true":"false");
                sensor1enabled = enabled;
                
                int pin = o["pin"];
                Serial.printf("sensors[1]:pin: %d\r\n", pin);
                sensor1pin = pin;
                
                String userKey = o["userKey"];
                Serial.printf("sensors[1]:userKey: %s\r\n", userKey.c_str());
                sensor1userKey = userKey;

                float ampThreshold = o["ampThreshold"];
                Serial.printf("sensors[1]:ampThreshold: %f\r\n", ampThreshold);
                sensor1ampThreshold = ampThreshold;
                
                float vMin = o["vMin"];
                Serial.printf("sensors[1]:vMin: %f\r\n", vMin);
                sensor1vMin = vMin;

                float vMax = o["vMax"];
                Serial.printf("sensors[1]:vMax: %f\r\n", vMax);
                sensor1vMax = vMax;

                float ampMax = o["ampMax"];
                Serial.printf("sensors[1]:ampMax: %f\r\n", ampMax);
                sensor1ampMax = ampMax;
                
                int sampCount = o["sampCount"];
                Serial.printf("sensors[1]:sampCount: %d\r\n", sampCount);
                sensor1sampCount = sampCount;

                // add by mr RIDE
                unsigned long waitForNextTimetCount = o["waitForNextTimetCount"];
                Serial.printf("sensors[1]:waitForNextTimetCount: %d\r\n", waitForNextTimetCount);
                sensor1waitForNextTimetCount = waitForNextTimetCount;
            }
            
            delay(5000);
        }
        catch(const std::exception& e)
        {
            hangup(e.what());
            return false;
        }
        return true;
    }
};

//設定情報
static Settings settings;

//センサークラス
class ClampSensor
{
public:
    float milliAmp = 0;
    unsigned long count = 0;
    bool state = false;
    bool enable = false;
    bool sendRequest = true; //初回は0で送信するためリクエストはON

    ClampSensor()
    {
        enable = false;
    }
    
    ClampSensor(bool sensorEnable, int pin, float vMin, float vMax, float ampMax, float ampThreshold, int sampCount, uint16_t waitForNextCount)
    {
        enable = sensorEnable;
        _pin = pin;
        _vMin = vMin;
        _vMax = vMax;
        _ampMax = ampMax;
        _ampTh = ampThreshold;
        _sampCount = sampCount;

        // add by mrRIDE
        _isCountUp = true;
        _isFallingEdge = false;
        _waitForNextCount = waitForNextCount;        //waitForNextCount*10ms sampling -> wait time s
        _waitForNextCountCnt = 0;
        
        Serial.print("SensorInit:");
        Serial.printf("enable: %d\r\n", enable);
        Serial.printf("pin: %d\r\n", _pin);
        Serial.printf("vMin: %f\r\n", _vMin);
        Serial.printf("vMax: %f\r\n", _vMax);
        Serial.printf("ampMaxax: %f\r\n", _ampMax);
        Serial.printf("ampThreshold: %f\r\n", _ampTh);
        Serial.printf("sampCount: %d\r\n", _sampCount);

        // add by mr RIDE
        Serial.printf("waitForNextCount: %d\r\n", _waitForNextCount);
    }

    void sampling()
    {
        if (!enable)
        {
            return;
        }
        checkCountUpCondition();    // add by mrRIDE
        milliAmp = getSensorValue();
        checkSensorOneCount(milliAmp);
    }

    void countup()
    {
        count++;
        sendRequest = true;
    }

private:
    int _pin;
    float _vMin;
    float _vMax;
    float _ampMax;
    float _ampTh;
    int _sampCount;
    float _calcSumAmp = 0;
    float _calcSumCount = 0;

    uint64_t beforePrint = 0;

    // add by mrRIDE
    bool _isCountUp;
    bool _isFallingEdge;
    unsigned long _waitForNextCount;     //300*10ms sampling -> 3s
    unsigned long _waitForNextCountCnt;
    
    void printAnalogRead(int e)
    {
        uint64_t now = millis();
        if(now - beforePrint >= 1000)
        {
            Serial.print("AnalogRead:");
            Serial.println(e);
            beforePrint = now;
        }
    }

    //アナログ値取得
    float getSensorValue()
    {
        //アナログ読み出し 0-4095(0-3.3v)
        int e = analogRead(_pin);
        printAnalogRead(e);

        //入力電圧と電流値の変換
        //vMin-vMax ampMin-ampMax(1-4v 0-30mA)(1-4v 0-300mA)(0-5v 0-5000mA)
        float v = e / 4095.0 * 3.3;
        if (v < _vMin)
        {
            v = _vMin;
        }
        return (v - _vMin) / (_vMax - _vMin) * _ampMax;
    }
    //センササンプリング
    void checkSensorOneCount(float amp)
    {
        _calcSumCount++;
        _calcSumAmp += amp;

        if (_calcSumCount >= _sampCount)
        {
            float aveAmp = _calcSumAmp / _calcSumCount;
            bool preSensorState = state;

            // debug
            Serial.printf("aveAmp: %.1f   _ampTh:%.1f  count: %d  _isCountUp: %d  _isFallingEdge: %d  _waitForNextCountCnt: %d  ms: %d", aveAmp, _ampTh, count, _isCountUp, _isFallingEdge, _waitForNextCountCnt, millis());
            Serial.println();
            if (aveAmp > _ampTh && _isCountUp && _isFallingEdge)
            {
                state = true;
                if (!preSensorState)
                {
                   countup();  
                }

                //set Disable coutup time flag anh waitForNextCount
                _isCountUp = false;
                _isFallingEdge = false;
                _waitForNextCountCnt = millis();
            }
            else
            {
                state = false;
            }

            // add by mrRIDE
            // 無視カウンター時間は立下り検知なし
            if (aveAmp < AMP_FALLEDGE_THRESHOLD && _isCountUp) 
            {
                //立下り検知
                _isFallingEdge = true;
            }

            _calcSumCount = 0;
            _calcSumAmp = 0;
        }
    }

    // Add by mrRIDE
    void checkCountUpCondition()
    {
      
      if (!_isCountUp)
      {
         unsigned long disableTimeRemmain = millis() -  _waitForNextCountCnt;
         if (millis() < _waitForNextCount || disableTimeRemmain > _waitForNextCount) //_waitForNextCount 3000
         {
           _isCountUp = true;
         }

         Serial.printf("DisableTimeRemain: %d   _isCountUp: %d", disableTimeRemmain, _isCountUp);
         Serial.println();
      }
    }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Utilities

//WiFi接続初期化
static void InitWifi()
{
    M5.Lcd.println("Network begin");
    WiFi.mode(WIFI_STA);
    
    M5.Lcd.println("Wifi disconnect");
    WiFi.disconnect(true);
    delay(5000);
    
    if(settings.ip != NULL && settings.ip != INADDR_NONE)
    {   //WiFi.configを呼ばない場合はDHCP接続。
        M5.Lcd.println("Wifi config");
        M5.Lcd.println(settings.ip);
        M5.Lcd.println(settings.gateway);
        M5.Lcd.println(settings.subnet);
        M5.Lcd.println(settings.dns);
        WiFi.config(settings.ip, settings.gateway, settings.subnet, settings.dns);
    }

    M5.Lcd.println("Wifi begin");
    M5.Lcd.println(settings.ssid);
    M5.Lcd.println(settings.password);
    WiFi.begin(settings.ssid.c_str(), settings.password.c_str());
    M5.Lcd.println("Connecting...");

    /*
    if(!checkWiFi(1000)){
        hangup("Timeout waiting for WiFi connection.");
        return;
    }
    */
    delay(500);
    Serial.printf("wf status: %d", WiFi.status());
    
    hasWifi = true;
    M5.Lcd.println("WiFi connected");
    M5.Lcd.println("IP address: ");
    M5.Lcd.println(WiFi.localIP().toString());
}

//WiFi接続確認
static bool checkWiFi(int retryCount)
{
    if (retryCount == 0)
    {
        return WiFi.status() == WL_CONNECTED;
    }
    wl_status_t status = WiFi.status();
    while (status != WL_CONNECTED)
    {
        Serial.print(".");
        if (--retryCount < 0)
        {
            return false;
        }
        //delay(10);

        status = WiFi.status();
        // test debug
        Serial.printf("wf status: %d", status);
    }

    return status == WL_CONNECTED;
}

//IoTHub送信応答ハンドラ
static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
    if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
    {
        Serial.println("Send Confirmation Callback finished.");
    }
}

//IoTHubメッセージ受信ハンドラ
static void MessageCallback(const char *payLoad, int size)
{
    Serial.println("Message callback:");
    Serial.println(payLoad);
}

//IoTHubデバイスツイン更新通知ハンドラ
static void DeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payLoad, int size)
{
    char *temp = (char *)malloc(size + 1);
    if (temp == NULL)
    {
        return;
    }
    memcpy(temp, payLoad, size);
    temp[size] = '\0';
    // Display Twin message.
    Serial.println(temp);
    free(temp);
}

//IoTHubデバイスメソッド呼び出しハンドラ
static int DeviceMethodCallback(const char *methodName, const unsigned char *payload, int size, unsigned char **response, int *response_size)
{
    Serial.printf("Try to invoke method %s\r\n", methodName);
    const char *responseMessage = "\"Successfully invoke device method\"";
    int result = 200;

    if (strcmp(methodName, "start") == 0)
    {
        Serial.println("Start sending data");
        messageSending = true;
    }
    else if (strcmp(methodName, "stop") == 0)
    {
        Serial.println("Stop sending data");
        messageSending = false;
    }
    else
    {
        Serial.printf("No method %s found\r\n", methodName);
        responseMessage = "\"No method found\"";
        result = 404;
    }

    *response_size = strlen(responseMessage) + 1;
    *response = (unsigned char *)strdup(responseMessage);

    return result;
}

//IoTHubメッセージ送信
static bool sendCount(const char *userKey, unsigned long count)
{
    Serial.printf("sendCount(%lu)\r\n", count);

    StaticJsonDocument<JSON_OBJECT_SIZE(64)> jsonDoc;
    jsonDoc["$type"] = "Uniface.IBMes.PLCMessage.PLCSignalWord, PLCMessage";
    //Timestampがない場合にはDataSaver側で現在時刻を使用する仕様となっているようなので省略。
    //jsonDoc["Timestamp"] = GetLocalTimeString();
    jsonDoc["Data"] = count;
    jsonDoc["BlockKey"] = userKey;
    jsonDoc["BlockName"] = userKey;
    jsonDoc["DataType"] = "Word";
    jsonDoc["Version"] = "1";
    jsonDoc["MessageType"] = "PLCSignalWord";
    jsonDoc["DataKey"] = "ProductionResultCounter";

    String messagePayload;
    serializeJson(jsonDoc, messagePayload);
    Serial.println(messagePayload.c_str());

    EVENT_INSTANCE *message = Esp32MQTTClient_Event_Generate(messagePayload.c_str(), MESSAGE);
    //Esp32MQTTClient_Event_AddProp(message, "temperatureAlert", "true");
    mqttConnected = Esp32MQTTClient_SendEventInstance(message);
    Serial.println("sent message.");

    return true;
}

//異常時停止
static void hangup(const char *msg)
{
    M5.Lcd.println(msg);
    M5.Lcd.println("restart M5Stack...");
    delay(10*1000);
    ESP.restart();
    return;
}

//MQTTクライアント
WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);
class MQTTClient{
 public:
  bool begin(){
    std::ostringstream oss;
    oss << "MQTT begin: " << settings.mqttServer << ":" << settings.mqttPort;
    M5.Lcd.println(oss.str().c_str());
    sprintf(clientid, "%03d-%03d-%03d-%03d", settings.ip[0], settings.ip[1], settings.ip[2], settings.ip[3]);
    pubSubClient.setServer(settings.mqttServer.c_str(), settings.mqttPort);
    if(!checkMQTT(100)){
      return false;
    }
    pubSubClient.setSocketTimeout(5);
    M5.Lcd.println("MQTT connected");
    return true;
  }

  bool checkMQTT(int retryCount){
    int retry = 0;
    if(!pubSubClient.connected())
    {
        Serial.println("connecting MQTT server.");
        while (!pubSubClient.connected())
        {
            pubSubClient.connect(clientid, settings.mqttUser.c_str(), settings.mqttPass.c_str());
            delay(1000);
            retry++;
            if(retry >= retryCount){
                Serial.print(".");
                break;
            }
        }
    }
    return pubSubClient.connected();
  }

  bool sendCount(const char *userKey,int count){
    StaticJsonDocument<JSON_OBJECT_SIZE(64)> jsonDoc;
  
    jsonDoc["$type"] = "Uniface.IBMes.PLCMessage.PLCSignalWord, PLCMessage";
    jsonDoc["Data"] = count;
    jsonDoc["BlockKey"] = userKey;
    jsonDoc["BlockName"] = "";
    jsonDoc["DataType"] = "Word";
    jsonDoc["Version"] = "1";
    jsonDoc["MessageType"] = "PLCSignalWord";
    jsonDoc["DataKey"] = "ProductionResultCounter";
  
    String messagePayload;
    serializeJson(jsonDoc, messagePayload);
    Serial.println(messagePayload);

    return pubSubClient.publish(settings.mqttRoutingKey.c_str(), messagePayload.c_str());
  }
 
 private:
    char clientid[30];
};


//グローバル変数初期化
ClampSensor sensor1 = ClampSensor();
ClampSensor sensor2 = ClampSensor();
MQTTClient mqttClient;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino sketch

//setup
void setup()
{
    //M5.begin(bool LCDEnable = true, bool SDEnable = true, bool SerialEnable = true, bool I2CEnable = false)
    M5.begin(true, true, true/*, true*/);
    M5.Power.begin();

    M5.Lcd.println("ESP32 Device");
    M5.Lcd.println("Initializing...");
    
    dacWrite(SPEAKER_PIN, 0); // アナログ入力ノイズ音のためスピーカーOFF
    M5.Lcd.setBrightness(50);

    if (!settings.Load())
    {
        hangup("Failed load settings.");
        return;
    }

    pinMode(settings.sensor0pin, INPUT);
    pinMode(settings.sensor1pin, INPUT);

    sensor1 = ClampSensor(settings.sensor0enabled, settings.sensor0pin, settings.sensor0vMin, settings.sensor0vMax, settings.sensor0ampMax, settings.sensor0ampThreshold, settings.sensor0sampCount, settings.sensor0waitForNextTimetCount);
    sensor2 = ClampSensor(settings.sensor1enabled, settings.sensor1pin, settings.sensor1vMin, settings.sensor1vMax, settings.sensor1ampMax, settings.sensor1ampThreshold, settings.sensor1sampCount, settings.sensor1waitForNextTimetCount);

    // Initialize the WiFi module
    M5.Lcd.println(" > WiFi");
    hasWifi = false;
    InitWifi();
    if (!hasWifi)
    {
        return;
    }
    randomSeed(analogRead(0));

    isIotHub = settings.connectionString != "";
    if(isIotHub){
      M5.Lcd.println(" > IoT Hub");
      const char *connectionString = settings.connectionString.c_str();
      Esp32MQTTClient_SetOption(OPTION_MINI_SOLUTION_NAME, "IBMesM5Stack");
      if (!Esp32MQTTClient_Init((const uint8_t *)connectionString, false, true))
      {
          hangup("Initializing IoT hub failed.");
          return;
      }
      Esp32MQTTClient_SetSendConfirmationCallback(SendConfirmationCallback);
      Esp32MQTTClient_SetMessageCallback(MessageCallback);
      Esp32MQTTClient_SetDeviceTwinCallback(DeviceTwinCallback);
      Esp32MQTTClient_SetDeviceMethodCallback(DeviceMethodCallback);  
    }
    else{
      M5.Lcd.println(" > MQTT Client");
      if(!mqttClient.begin()){
          hangup("Initializing MQTT Client failed.");
          return;
      }
    }
    send_interval_ms = millis();
    sampling_ms = millis();
        
    //debug test
    //M5.Lcd.println("WiFi debug test");
    
    M5.Lcd.fillScreen(BLACK);

    
}

bool isMQTTConnected()
{
    bool connected = false;

    if(!isIotHub){
        //mqttClientのloopを呼ばないと定期的にmqtt接続が切れる
        if(!pubSubClient.connected())
        {
            connected =  mqttClient.checkMQTT(0);
        }
        else
        {
            connected = pubSubClient.loop();
        }
    }
    else
    {
        Esp32MQTTClient_Check();
        connected = true;
    }
    return connected;
}

//loop
void loop()
{
    M5.update();
    const unsigned long now = millis();
    if (now < sampling_ms || (now - sampling_ms) > settings.samp_msec)
    {
        //センサー値更新
        sensor1.sampling();
        sensor2.sampling();
        sampling_ms = millis();
        
    }

    if (checkWiFi(0))
    {
        if(mqttConnected = isMQTTConnected())
        {
            int forceSend  = M5.BtnA.wasReleasefor(1000);
            if(forceSend)
            {
                Serial.println("Force countup.");
                sensor1.countup();
            }
            if ((messageSending && (int)(millis() - send_interval_ms) >= settings.interval) || forceSend)
            {
                if (sensor1.enable && sensor1.sendRequest)
                {
                    if(!isIotHub){
                        if (mqttClient.sendCount(settings.sensor0userKey.c_str(), sensor1.count))
                        {
                            sensor1.sendRequest = false;
                        }
                    }
                    else{
                        if (sendCount(settings.sensor0userKey.c_str(), sensor1.count))
                        {
                            sensor1.sendRequest = false;
                        }
                    }
                }
                if (sensor2.enable && sensor2.sendRequest)
                {
                    if(!isIotHub){
                        if (mqttClient.sendCount(settings.sensor1userKey.c_str(), sensor2.count))
                        {
                            sensor2.sendRequest = false;
                        }
                    }
                    else {
                        if (sendCount(settings.sensor1userKey.c_str(), sensor2.count))
                        {
                            sensor2.sendRequest = false;
                        }
                    }
                }
                send_interval_ms = millis();
            }
        }
    }
    else
    {
        hangup("WiFi not connected.");
    }
    //画面表示
    disp();

    //Test
    M5.Lcd.println("Test Connect Sensor:");
}

//画面表示ウェイトカウンタ
uint64_t beforeDisp = 0;

//画面表示
void disp()
{
    if (millis() - beforeDisp < DISP_INTERVAL)
        return;

    beforeDisp = millis();

    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print("IB-Clamp");
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.print(" - ");
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.println("IB-Mes");
    M5.Lcd.setTextSize(1);

    M5.Lcd.println("");
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.println("Sensor1:");
    M5.Lcd.print(sensor1.milliAmp, 1);
    M5.Lcd.println(" mA     ");
    M5.Lcd.print(sensor1.state, 1);
    M5.Lcd.println(" State     ");
    M5.Lcd.print(sensor1.count, 1);
    M5.Lcd.println(" Count     ");
    M5.Lcd.setTextSize(1);
    M5.Lcd.println("");
    M5.Lcd.setTextSize(2);
    M5.Lcd.println("Sensor2:");
    M5.Lcd.print(sensor2.milliAmp, 1);
    M5.Lcd.println(" mA     ");
    M5.Lcd.print(sensor2.state, 1);
    M5.Lcd.println(" State     ");
    M5.Lcd.setTextSize(1);
    M5.Lcd.println("");
    M5.Lcd.setTextSize(2);
    M5.Lcd.println("Wifi:");
    wl_status_t status = WiFi.status();
    switch (status)
    {
    case WL_NO_SHIELD:
        M5.Lcd.println("WL_NO_SHIELD          ");
        break;
    case WL_IDLE_STATUS:
        M5.Lcd.println("WL_IDLE_STATUS        ");
        break;
    case WL_NO_SSID_AVAIL:
        M5.Lcd.println("WL_NO_SSID_AVAIL      ");
        break;
    case WL_SCAN_COMPLETED:
        M5.Lcd.println("WL_SCAN_COMPLETED     ");
        break;
    case WL_CONNECTED:
        M5.Lcd.println("WL_CONNECTED          ");
        break;
    case WL_CONNECT_FAILED:
        M5.Lcd.println("WL_CONNECT_FAILED     ");
        break;
    case WL_CONNECTION_LOST:
        M5.Lcd.println("WL_CONNECTION_LOST    ");
        break;
    case WL_DISCONNECTED:
        M5.Lcd.println("WL_DISCONNECTED       ");
        break;
    default:
        M5.Lcd.println("UNKNOWN               ");
        break;
    }
    if (mqttConnected)
    {
        M5.Lcd.println("MQTT Connected          ");
    }
    else
    {
        M5.Lcd.println("MQTT Disconnected       ");
    }
    //M5.Lcd.print(WiFi.RSSI());
    M5.Lcd.printf("%d dBm     \r\n", WiFi.RSSI());
    M5.Lcd.printf("Free heap %u\r\n", xPortGetFreeHeapSize());
}