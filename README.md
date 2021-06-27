# SimpleFarmIOT
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
-Cloud Azure -> free host heroku
-Webserver: Protocol: WebscoketIO, NodeJS, javascript, html, css
--------------------プロジェクトの概略内容 (End)--------------------
