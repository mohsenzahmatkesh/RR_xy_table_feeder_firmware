/**
 * Commands.cpp
 *
 *  Created on: Nov 18, 2021
 *      Author: M.Saadat (m.saadat@mail.com)
 * Github: https://github.com/mahmood-saadat/ESP32-Templates/tree/master/Task%20Templates
 * This file provide as is with no guarantee of any sort.
 * Any modification and redistribution of this file is allowed as long as this description is kept at the top of the file.
 *
 */

#include "Commands.h"

#include <esp_task_wdt.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "MotorDriver.h"
#include "debug.h"
#include "FeederDriver.h"

#define		WDT_TASK_TIMEOUT 	3000
#define		WIFI_UDP_PORT		8745


Commands 		commands;
WiFiUDP 		wifiUDP;

/**
 * @fn  Commands()
 * @brief Constructor
 *
 */
Commands::Commands(){
	mainTask = NULL;
}

/**
 * @fn void begin()
 * @brief Initialize the module and start the async task
 *
 */
void Commands::begin(){
	//Serial.println("[Commands] Setup is started ...");

	xTaskCreate((TaskFunction_t)Commands::TaskStart, "CommandsTask", 8192, this, 20, &mainTask);

	//Serial.println("[Commands] Setup is done.");

}


/**
 * @fn TaskFunction_t TaskStart(void*)
 * @brief Task function needed to be static and in static members, we could not access class members easily,
 * so we created this function and we passed the class as input argument, then we call our main function from here.
 *
 * @param pvParameters
 * @return
 */
TaskFunction_t Commands::TaskStart(void * pvParameters){
	Commands* manager;
	esp_task_wdt_init(WDT_TASK_TIMEOUT, true);
	esp_task_wdt_add(NULL);
	manager = (Commands*) pvParameters;
	manager->MainTask();
	// It will never get here. The main task never returns.
	return 0;
}

/**
 * @fn void MainTask()
 * @brief  The main loop. Here we have the infinite loop. It checks for any incoming command.
 *
 */
void Commands::MainTask() {
	uint8_t 	receiveBuffer[255] = {0};
	int16_t 	packetSize = 0;
	bool		is_any_udp_packet_received = false;
	uint16_t 	receive_timeout = 0;
	wifiUDP.begin(WIFI_UDP_PORT);
	while(1){
		esp_task_wdt_reset();

		// Check if we have the command from UDP port
		packetSize = wifiUDP.parsePacket();
		if(packetSize)
		{
			int len = wifiUDP.read(receiveBuffer, 255);
			if (len == 16)
			{
				//! 4 bytes float: x position
				//! 4 bytes float: x speed
				//! 4 bytes float: y position
				//! 4 bytes float: y speed
				float x = ConvertToFloat((uint8_t*)&receiveBuffer[0]);
				float x_speed = ConvertToFloat((uint8_t*)&receiveBuffer[4]);
				float y = ConvertToFloat((uint8_t*)&receiveBuffer[8]);
				float y_speed = ConvertToFloat((uint8_t*)&receiveBuffer[12]);
				if(motorDriver.IsReady())
				{
					motorDriver.SetXY(x, y, x_speed, y_speed);
					is_any_udp_packet_received = true;
				}
			}
			else
			{
				//Serial.printf("[Commands] Wrong UDP data length: %d\n", len);
			}
		}

		//! If any packet is received, we have the IP address to send back data
		//! 4 bytes float: x position
		//! 4 bytes float: y position
		if(is_any_udp_packet_received)
		{
			float x = 0.0f;
			float y = 0.0f;
			uint8_t buffer[8] = {0};
			motorDriver.GetXY(&x, &y);
			memcpy(&buffer[0], &x, 4);
			memcpy(&buffer[4], &y, 4);
			wifiUDP.beginPacket(wifiUDP.remoteIP(), WIFI_UDP_PORT);
			wifiUDP.write(buffer, 8);
			wifiUDP.endPacket();
		}

		//! Check for commands from serial port
		//! The packet is as this:
		//! 3 bytes header:
		//! 	- Position and speed command: 0xFF 0xFE 0xFF
		//! 	- Reset the position to command value: 0xFF 0xFE 0xFE
		//! 	- Command for setting the Position, Speed, and acceleration: 0xFF 0xFE 0xFD

		// while(Serial.available() >= 3)
		// Commented out for adding feede:
		// {
		// 	DEBUG_printf("Serial bytes received: %d\r\n", Serial.available());
		// 	// Check if we have the header
		// 	if(Serial.read() == 0xFF)
		// 	{
		// 		if(Serial.read() == 0xFE)
		// 		{
		// 			uint8_t header = Serial.read();
		// 			//! Check if it's the position/speed command
		// 			//! The packet is as this:
		// 			//! 3 bytes header: 0xFF 0xFE 0xFF
		// 			//! 4 bytes float: X target
		// 			//! 4 bytes float: X speed
		// 			//! 4 bytes float: Y target
		// 			//! 4 bytes float: Y speed
		// 			//! 1 byte checksum: sum of the X, Y and speed bytes (everything except the header)
		// 			if(header == 0xFF)
		// 			{
		// 				receive_timeout = 10;
		// 				while(Serial.available() <= 17 && receive_timeout > 0)
		// 				{
		// 					receive_timeout --;
		// 					vTaskDelay(1/portTICK_PERIOD_MS);
		// 				}
		// 				DEBUG_printf("Serial command header detected.\r\n");
		// 				if(Serial.available() >= 17)
		// 				{
		// 					// Read the data bytes
		// 					Serial.read(receiveBuffer, 17);
		// 					if(receiveBuffer[16] == CalculateChecksum(receiveBuffer, 16))
		// 					{
		// 						float x = ConvertToFloat((uint8_t*)&receiveBuffer[0]);
		// 						float x_speed = ConvertToFloat((uint8_t*)&receiveBuffer[4]);
		// 						float y = ConvertToFloat((uint8_t*)&receiveBuffer[8]);
		// 						float y_speed = ConvertToFloat((uint8_t*)&receiveBuffer[12]);
		// 						if(motorDriver.IsReady())
		// 						{
		// 							motorDriver.SetXY(x, y, x_speed, y_speed);
		// 						}

		// 						SendBackCurrentPosition();
		// 					}
		// 					else
		// 					{
		// 						DEBUG_printf("Checksum failed; must be: %d is: %d\r\n", CalculateChecksum(receiveBuffer, 16), receiveBuffer[16]);
		// 						DEBUG_printf("Array: ");
		// 						for(uint16_t index = 0; index < 17; index ++)
		// 						{
		// 							DEBUG_printf("%02X ", receiveBuffer[index]);
		// 						}
		// 						DEBUG_printf("\r\n");
		// 					}
		// 				}
		// 				else
		// 				{
		// 					DEBUG_printf("Not enough bytes!\r\n");
		// 				}
		// 			}
		// 			// Reset the error command
		// 			else if(header == 0xFE)
		// 			{
		// 				DEBUG_printf("Serial reset error command header detected.\r\n");
		// 				receive_timeout = 10;
		// 				while(Serial.available() <= 9 && receive_timeout > 0)
		// 				{
		// 					receive_timeout --;
		// 					vTaskDelay(1/portTICK_PERIOD_MS);
		// 				}
		// 				if(Serial.available() >= 9)
		// 				{
		// 					// Read the data bytes
		// 					Serial.read(receiveBuffer, 9);
		// 					if(receiveBuffer[8] == CalculateChecksum(receiveBuffer, 8))
		// 					{
		// 						float x = ConvertToFloat((uint8_t*)&receiveBuffer[0]);
		// 						float y = ConvertToFloat((uint8_t*)&receiveBuffer[4]);
		// 						if(motorDriver.IsReady())
		// 						{
		// 							motorDriver.SetXYCurrentPosition(x, y);
		// 						}

		// 						SendBackCurrentPosition();
		// 					}
		// 					else
		// 					{
		// 						DEBUG_printf("Checksum failed; must be: %d is: %d\r\n", CalculateChecksum(receiveBuffer, 8), receiveBuffer[8]);
		// 						DEBUG_printf("Array: ");
		// 						for(uint16_t index = 0; index < 9; index ++)
		// 						{
		// 							DEBUG_printf("%02X ", receiveBuffer[index]);
		// 						}
		// 						DEBUG_printf("\r\n");
		// 					}
		// 				}
		// 				else
		// 				{
		// 					DEBUG_printf("Not enough bytes in reset command!\r\n");
		// 				}
		// 			}
		// 			//! Check if it's the position/speed/acceleration command
		// 			//! The packet is as this:
		// 			//! 3 bytes header: 0xFF 0xFE 0xFD
		// 			//! 4 bytes float: X target
		// 			//! 4 bytes float: X speed
		// 			//! 4 bytes float: X acceleration
		// 			//! 4 bytes float: Y target
		// 			//! 4 bytes float: Y speed
		// 			//! 4 bytes float: Y acceleration
		// 			//! 1 byte checksum: sum of the X, Y, speed and acceleration bytes (everything except the header)
		// 			else if(header == 0xFD)
		// 			{
		// 				receive_timeout = 10;
		// 				while(Serial.available() <= 25 && receive_timeout > 0)
		// 				{
		// 					receive_timeout --;
		// 					vTaskDelay(1/portTICK_PERIOD_MS);
		// 				}
		// 				DEBUG_printf("Serial command header detected.\r\n");
		// 				if(Serial.available() >= 25)
		// 				{
		// 					// Read the data bytes
		// 					Serial.read(receiveBuffer, 25);
		// 					if(receiveBuffer[24] == CalculateChecksum(receiveBuffer, 24))
		// 					{
		// 						float x = ConvertToFloat((uint8_t*)&receiveBuffer[0]);
		// 						float x_speed = ConvertToFloat((uint8_t*)&receiveBuffer[4]);
		// 						float x_accel = ConvertToFloat((uint8_t*)&receiveBuffer[8]);
		// 						float y = ConvertToFloat((uint8_t*)&receiveBuffer[12]);
		// 						float y_speed = ConvertToFloat((uint8_t*)&receiveBuffer[16]);
		// 						float y_accel = ConvertToFloat((uint8_t*)&receiveBuffer[20]);
		// 						if(motorDriver.IsReady())
		// 						{
		// 							motorDriver.SetXY(x, y, x_speed, y_speed, x_accel, y_accel);
		// 						}

		// 						SendBackCurrentPosition();
		// 					}
		// 					else
		// 					{
		// 						DEBUG_printf("Checksum failed; must be: %d is: %d\r\n", CalculateChecksum(receiveBuffer, 16), receiveBuffer[16]);
		// 						DEBUG_printf("Array: ");
		// 						for(uint16_t index = 0; index < 17; index ++)
		// 						{
		// 							DEBUG_printf("%02X ", receiveBuffer[index]);
		// 						}
		// 						DEBUG_printf("\r\n");
		// 					}
		// 				}
		// 				else
		// 				{
		// 					DEBUG_printf("Not enough bytes!\r\n");
		// 				}
		// 			}
		// 		}
		// 	}
		// }

		// --- MIXED PROTOCOL PARSER ---
		while(Serial.available() >= 3)
		{
			uint8_t firstByte = Serial.peek();
			
			// 1. Process X-Y Table Commands (0xFF)
			if(firstByte == 0xFF)
			{
				Serial.read(); // Consume the 0xFF
				if(Serial.read() == 0xFE)
				{
					uint8_t header = Serial.read();
					
					//! Check if it's the position/speed command (0xFF)
					if(header == 0xFF)
					{
						receive_timeout = 10;
						while(Serial.available() <= 17 && receive_timeout > 0)
						{
							receive_timeout --;
							vTaskDelay(1/portTICK_PERIOD_MS);
						}
						DEBUG_printf("Serial command header detected.\r\n");
						if(Serial.available() >= 17)
						{
							Serial.read(receiveBuffer, 17);
							if(receiveBuffer[16] == CalculateChecksum(receiveBuffer, 16))
							{
								float x = ConvertToFloat((uint8_t*)&receiveBuffer[0]);
								float x_speed = ConvertToFloat((uint8_t*)&receiveBuffer[4]);
								float y = ConvertToFloat((uint8_t*)&receiveBuffer[8]);
								float y_speed = ConvertToFloat((uint8_t*)&receiveBuffer[12]);
								if(motorDriver.IsReady())
								{
									motorDriver.SetXY(x, y, x_speed, y_speed);
								}
								SendBackCurrentPosition();
							}
							else
							{
								DEBUG_printf("Checksum failed; must be: %d is: %d\r\n", CalculateChecksum(receiveBuffer, 16), receiveBuffer[16]);
							}
						}
					}
					// Reset the error command (0xFE)
					else if(header == 0xFE)
					{
						DEBUG_printf("Serial reset error command header detected.\r\n");
						receive_timeout = 10;
						while(Serial.available() <= 9 && receive_timeout > 0)
						{
							receive_timeout --;
							vTaskDelay(1/portTICK_PERIOD_MS);
						}
						if(Serial.available() >= 9)
						{
							Serial.read(receiveBuffer, 9);
							if(receiveBuffer[8] == CalculateChecksum(receiveBuffer, 8))
							{
								float x = ConvertToFloat((uint8_t*)&receiveBuffer[0]);
								float y = ConvertToFloat((uint8_t*)&receiveBuffer[4]);
								if(motorDriver.IsReady())
								{
									motorDriver.SetXYCurrentPosition(x, y);
								}
								SendBackCurrentPosition();
							}
							else
							{
								DEBUG_printf("Checksum failed; must be: %d is: %d\r\n", CalculateChecksum(receiveBuffer, 8), receiveBuffer[8]);
							}
						}
					}
					//! Check if it's the position/speed/acceleration command (0xFD)
					else if(header == 0xFD)
					{
						receive_timeout = 10;
						while(Serial.available() <= 25 && receive_timeout > 0)
						{
							receive_timeout --;
							vTaskDelay(1/portTICK_PERIOD_MS);
						}
						DEBUG_printf("Serial command header detected.\r\n");
						if(Serial.available() >= 25)
						{
							Serial.read(receiveBuffer, 25);
							if(receiveBuffer[24] == CalculateChecksum(receiveBuffer, 24))
							{
								float x = ConvertToFloat((uint8_t*)&receiveBuffer[0]);
								float x_speed = ConvertToFloat((uint8_t*)&receiveBuffer[4]);
								float x_accel = ConvertToFloat((uint8_t*)&receiveBuffer[8]);
								float y = ConvertToFloat((uint8_t*)&receiveBuffer[12]);
								float y_speed = ConvertToFloat((uint8_t*)&receiveBuffer[16]);
								float y_accel = ConvertToFloat((uint8_t*)&receiveBuffer[20]);
								if(motorDriver.IsReady())
								{
									motorDriver.SetXY(x, y, x_speed, y_speed, x_accel, y_accel);
								}
								SendBackCurrentPosition();
							}
							else
							{
								DEBUG_printf("Checksum failed; must be: %d is: %d\r\n", CalculateChecksum(receiveBuffer, 16), receiveBuffer[16]);
							}
						}
					}
				}
			}
			
			// 2. Process Z-Feeder Commands (0xAA)
			else if (firstByte == 0xAA)
			{
				Serial.read(); // Consume 0xAA
				if (Serial.read() == 0xBB && Serial.read() == 0xCC) 
				{
					receive_timeout = 10;
					// Wait for the 22 payload bytes
					while(Serial.available() < 22 && receive_timeout > 0) {
						receive_timeout--;
						vTaskDelay(1 / portTICK_PERIOD_MS);
					}
					
					if (Serial.available() >= 22) {
						Serial.read(receiveBuffer, 22);
						
						// Checksum validation
						uint8_t calcCheck = 0;
						for (int i = 0; i < 21; i++) calcCheck += receiveBuffer[i];
						
						if (calcCheck == receiveBuffer[21]) {
							uint8_t cmdType = receiveBuffer[0];
							if (cmdType == 0) { // POSITION_REQUEST
								int32_t p, v;
								memcpy(&p, &receiveBuffer[1], 4);
								memcpy(&v, &receiveBuffer[5], 4);
								feederDriver.SetTarget(p, v);
							}
							else if (cmdType == 1) { // FEED_AXIS_PULSE_SET
								int32_t temp;
								memcpy(&temp, &receiveBuffer[1], 4);
								feederDriver.OverridePosition(temp);
							}
							else if (cmdType == 2) { // TOLERANCE_SET
								int32_t tb, ts;
								memcpy(&tb, &receiveBuffer[1], 4);
								memcpy(&ts, &receiveBuffer[5], 4);
								feederDriver.SetTolerances(tb, ts);
							}
							else if (cmdType == 3) { // QRE_SETTINGS_SET
								int32_t t;
								memcpy(&t, &receiveBuffer[1], 4);
								feederDriver.SetTau(t);
							}
						}
					}
				}
			} 
			else {
				// Unknown header, discard one byte to prevent an infinite loop
				Serial.read(); 
			}
		}

		PrintStackWatermark();
		// Periodic Send for Z-Feeder (Every 50ms)
        static uint32_t last_send_time = 0;
        if (millis() - last_send_time >= 50) {
            last_send_time = millis();
            
            byte packet[30];
            packet[0] = 0xAA; packet[1] = 0xBB; packet[2] = 0xCC;
            
            int32_t h = feederDriver.GetCurrentZ();
            int32_t des = feederDriver.GetDesiredZ();
            int32_t qre = feederDriver.GetQRE();
            int32_t tau = feederDriver.GetTau();
            uint64_t ts = feederDriver.GetTimestamp();
            
            memcpy(&packet[3],  &h,  4);  
            memcpy(&packet[7],  &des,   4);   
            memcpy(&packet[11], &qre,  4);   
            memcpy(&packet[15], &tau,    4);    
            memcpy(&packet[19], &ts,  8);

            packet[27] = feederDriver.GetStatusByte();
            packet[28] = feederDriver.GetError();

            byte checksum = 0;
            for (int i = 0; i < 26; i++) {
                checksum += packet[i + 3];
            }
            packet[29] = checksum;
            Serial.write(packet, 30);
        }
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

/**
 * @fn void PrintStackWatermark()
 * @brief Print the remaining stack
 *
 */
void Commands::PrintStackWatermark(){
	if(millis() > (stackWatermarkPrintLastMillis + 60000)){
		stackWatermarkPrintLastMillis = millis();
//		Serial.printf("[Commands] Task Stack left: %d\r\n", uxTaskGetStackHighWaterMark(NULL));
		DEBUG_printf("[Commands] Task Stack left: %d\r\n", uxTaskGetStackHighWaterMark(NULL));
	}
}

/**
 * @fn float ConvertToFloat(uint8_t*)
 * @brief Convert four bytes from a pointer to a float value. The buffer must be at least 4 bytes.
 *
 * @param buffer	The pointer to the bytes
 * @return float	The float value in the buffer
 */
float Commands::ConvertToFloat(uint8_t * buffer)
{
	float float_value = 0.0f;
	memcpy(&float_value, &buffer[0], 4);
	return float_value;
}

/**
 * @fn uint8_t CalculateChecksum(uint8_t*, uint16_t)
 * @brief Calculate the checksum from the bytes in the buffer. The checksum is the simple sum of all the bytes in the buffer.
 * Only the low eight bits of sum is used for checksum.
 *
 * @param buffer	The pointer to the buffer
 * @param len		Length of data in the buffer
 * @return uint8_t	The calculated checksum
 */
uint8_t Commands::CalculateChecksum(uint8_t * buffer, uint16_t len)
{
	uint8_t checksum = 0;
	for(int i = 0; i < len; i++)
	{
		checksum += buffer[i];
	}
	return checksum;
}

/**
 * @fn void SendBackCurrentPosition()
 * @brief Get the current x and y values from motor controller and send it over UART port.
 *
 */
void Commands::SendBackCurrentPosition()
{
	//! The response packet
	//! The packet is as this:
	//! 3 bytes header: 0xFF 0xFE 0xFF
	//! 4 bytes float: X position
	//! 4 bytes float: Y position
	//! 1 byte uint8: X start limit switch
	//! 1 byte uint8: X end limit switch
	//! 1 byte uint8: Y start limit switch
	//! 1 byte uint8: Y end limit switch

	// Send back the current location
	float x = 0.0f;
	float y = 0.0f;
	uint8_t buffer[16] = {0};
	motorDriver.GetXY(&x, &y);
	buffer[0] = 0xFF;
	buffer[1] = 0xFE;
	buffer[2] = 0xFF;
	memcpy(&buffer[3], &x, 4);
	memcpy(&buffer[7], &y, 4);
	buffer[8] = motorDriver.IsXStartLimitSwitchTriggered();
	buffer[9] = motorDriver.IsXEndLimitSwitchTriggered();
	buffer[10] = motorDriver.IsYStartLimitSwitchTriggered();
	buffer[11] = motorDriver.IsYEndLimitSwitchTriggered();
	buffer[15] = CalculateChecksum(&buffer[3], 12);
	Serial.write(buffer, 16);
}

