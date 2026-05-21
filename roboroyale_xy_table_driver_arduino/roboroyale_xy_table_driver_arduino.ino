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
	//Serial.printf("Started!\r\n");
	WifiManager::WiFiOn();
	WifiManager::WiFiOff();
	commands.begin();
	motorDriver.begin();
	feederDriver.begin();
  feederDriver.homing();
}

// The loop function is called in an endless loop
void loop()
{
	feederDriver.update();
	vTaskDelay(5 / portTICK_PERIOD_MS);
}
