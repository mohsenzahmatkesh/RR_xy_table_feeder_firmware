/*
 * debug.h
 *
 *  Created on: 22 Mar 2023
 *      Author: swacil-electronic
 */

#ifndef FUNCTIONS_DEBUG_H_
#define FUNCTIONS_DEBUG_H_

#include <WiFiUdp.h>
#include "WifiManager.h"

static WiFiUDP	udp;
template<typename... Args>
void DEBUG_printf(Args... args){
//	Serial.printf(args...);
	if(WifiManager::IsConnected())
	{
		udp.beginPacket("192.168.1.255", 5480);
		udp.printf(args...);
		udp.endPacket();
	}
}


#endif /* FUNCTIONS_DEBUG_H_ */
