/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp8266-client-server-wi-fi/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <stdio.h>

const char* ssid = "ESP8266-Access-Point";
const char* password = "123456789";

//Your IP address or domain name with URL path
const char* serverNameZen = "http://192.168.4.1/ZenPod";


uint8_t zendata;
uint8_t prev_data;
char buf[50];
String tempPayload;
unsigned long previousMillis = 0;
const long interval = 5000; 

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to WiFi");
}

void loop() {
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis >= interval) {
     // Check WiFi connection status
    if ((WiFi.status() == WL_CONNECTED)) {
      tempPayload = httpGETRequest(serverNameZen);
        if(tempPayload != NULL)
        {
          tempPayload.toCharArray(buf, 50);
    
          zendata = strtoul(buf, NULL, 16); // convert to hex
          
          if(zendata != prev_data)
          {
            prev_data = zendata;
            Serial.println("Current Zen Hex Data: " + String(zendata) + "\n");
            // Pass data over serial to output port, GPIO RX (1) to KL25Z board
          }
          else
          {
            //data is the same, do not update internals
          }
        }
       // save the last HTTP GET Request
      previousMillis = currentMillis;
    }
      else {
        Serial.println("WiFi Disconnected");
      }
    }
}

String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
    
  // Your IP address with path or Domain name with URL path 
  http.begin(client, serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload;
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}
