#include <Arduino.h>
#include <ESP.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>

const char* ssid = "[wifissid]";
const char* password = "[wifipassword]";

AsyncWebServer server(80);

StaticJsonDocument<6144> data;
String dataStr;
int idx = 0;
float hitsX;
float hitsY;
float angleX;
float angleY;
float accel;

void setup() {
	Serial.begin(115200);												//Initialize serial (UART) with baudrate 115200
	WiFi.begin(ssid, password);											//Connect to WIFI access point
	while (WiFi.status() != WL_CONNECTED){								//Wait for WIFI to connect
		delay(1000);
		Serial.println("Connecting to WiFi..");
	}

	if (!LittleFS.begin()){												//Initialize LittleFS for SPIFFS filesystem
		Serial.println("An Error has occurred while mounting LittleFS");
		return;
	}

	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ 		//Handle "/" route and serve the web app HTML file
		request->send(LittleFS, "/index.html", "text/html"); 
	});

	data["id"] = 0;
	data["hitsX"] = 0;
	data["hitsY"] = 0;
	data["angleX"] = 0;
	data["angleY"] = 0;
	data["accel"] = 0;
	serializeJson(data, dataStr);										//Serialize JSON object (Convert to Text)

	server.on("/getData", HTTP_GET, [](AsyncWebServerRequest *request){ //Handle "/getData" API and respond with the JSON data
		request->send(200, "application/javascript", dataStr);
	});
	server.begin();														//Initialize the HTTP server

	Serial.println(WiFi.localIP());
	ArduinoOTA.begin();													//Allow OTA (Over-The-Air) code and filesystem image update

}

void loop() {
	ArduinoOTA.handle();
	if (Serial.available() > 0){
		String inputString = Serial.readStringUntil('\n');
		String values[5];
		int valueIndex = 0;
		int startIndex = 0;
		for (unsigned int i = 0; i < inputString.length(); i++) {
			if (inputString[i] == ',') {
				values[valueIndex++] = inputString.substring(startIndex, i);
				startIndex = i + 1;
			}
		}
		values[valueIndex] = inputString.substring(startIndex);
		idx++;
		data["id"] = idx;
		data["hitsX"] = values[0].toFloat();
		data["hitsY"] = values[1].toFloat();
		data["angleX"] = values[2].toFloat();
		data["angleY"] = values[3].toFloat();
		data["accel"] = values[4].toFloat();
		dataStr = "";
		serializeJson(data, dataStr);
	}
}