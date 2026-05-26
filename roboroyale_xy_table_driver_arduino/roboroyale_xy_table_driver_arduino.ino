#include <dummy.h>

#include "Arduino.h"

#include "WifiManager.h"
#include "Commands.h"
#include "MotorDriver.h"
#include "FeederDriver.h"

//The setup function is called once at startup of the sketch
void setup()
{
	Serial.begin(115200);
	WifiManager::WiFiOn();
	WifiManager::WiFiOff();
	
	// 1. Initialize and Home the Z-Feeder FIRST
	feederDriver.begin();
	feederDriver.homing();
	
	// 2. Start the background RTOS Tasks for X/Y and Comms
	commands.begin();
	motorDriver.begin();
}


void loop()
{
	feederDriver.update();
	vTaskDelay(5 / portTICK_PERIOD_MS);
}
