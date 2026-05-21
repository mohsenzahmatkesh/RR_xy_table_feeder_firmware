/*
 * wif_management.cpp
 *
 *  Created on: 8 Jan 2023
 *      Author: swacil-electronic
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "WifiManager.h"

void WifiManager::WiFiOn() {
	IPAddress ip(192, 168, 1, 1);
	IPAddress gateway(192, 168, 1, 1);
	IPAddress dhcpLeaseStart(192, 168, 1, 2);
	IPAddress subnet(255, 255, 255, 0); // set subnet mask to match your
	IPAddress dns(8,8,8,8);
	IPAddress dns2(4,2,2,4);


	int counter = 0;
	//Serial.println(F("[WiFi] Connecting ... "));
//	if(flash.GetWifiType() == WifiType::WifiType_AP){
		WiFi.mode(WIFI_AP);
		while(!WiFi.softAP("ESP", "Day$88@88#") && counter < 10){
			vTaskDelay(500);
			//Serial.print(":");
			counter ++;
		}
		//vTaskDelay(2000);
		WiFi.softAPConfig(ip, gateway, subnet, dhcpLeaseStart);
//	}
//	else{
//		if (!WiFi.mode(WIFI_STA))
//		{
//			//Serial.println(F("[WiFi] Failed to set STA mode"));
//		}
//
//		if (!WiFi.config(ip, gateway, subnet, dns))//, dns2))
//		{
//			//Serial.println(F("[WiFi] STA Failed to configure"));
//		}
//		WiFi.begin("SwaCIL", "xxxxxxx");
////		while (WiFi.status() != WL_CONNECTED && counter < 10) {
////			vTaskDelay(500);
////			//Serial.print(".");
////			counter ++;
////		}
////	}
//	//Serial.print("\r\n");
//
//	// Print local IP address and start web server
//	if(WiFi.status() == WL_CONNECTED){
//		//Serial.println(F("WiFi connected."));
//		//Serial.println(F("IP address: "));
//		//Serial.println(WiFi.localIP().toString());
//	}
//	else{
//		//Serial.println(F("[WiFi] WiFi NOT connected."));
//	}

}


void WifiManager::WiFiOff() {
	//Serial.println(F("[WiFi] Disconnecting client and wifi"));

	WiFi.disconnect();
	vTaskDelay(1000);
	WiFi.mode(WIFI_OFF);
}

bool WifiManager::IsConnected()
{
	return (WiFi.status() == WL_CONNECTED);
}
