/*
 * PulseGenerator.h
 *
 *  Created on: Nov 18, 2021
 *      Author: M.Saadat (m.saadat@mail.com)
 * Github: https://github.com/mahmood-saadat/ESP32-Templates/tree/master/Task%20Templates
 * This file provide as is with no guarantee of any sort.
 * Any modification and redistribution of this file is allowed as long as this description is kept at the top of the file.
 *
 *  V2.0.0: Acceleration added
 */

#ifndef __TASKS_MOTORDRIVER_H__
#define __TASKS_MOTORDRIVER_H__

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


class MotorDriver{
public:

	typedef enum MOTOR_STATE{
		IDLE,
		ACCELERATION,
		DECELERATION,
		CONSTANT_SPEED
	}motor_state_t;
	//------------------------------ Public static variables  -------------------------------

private:
	//------------------------------ Private static variables  ------------------------------


	//------------------------------ Private variables  -------------------------------------
	TaskHandle_t 			mainTask;
	uint32_t				stackWatermarkPrintLastMillis = 0;
	bool					is_command_new = false;
	uint32_t				x_pulse_frequency = 1000;
	uint32_t				y_pulse_frequency = 1000;
	bool					is_acceleration_command = false;
	motor_state_t			x_motor_state = IDLE;
	motor_state_t			y_motor_state = IDLE;

	float					x_command_speed	= 5.0f; // mm/s - This is set from outside of library. It can be negative. But it will be converted to make sure it is positive value.
	float					x_current_speed	= 0.0f; // mm/s - The current speed
	float					x_target_speed = 0.0f; // mm/s - The target speed depending on the state
	float					x_command_acceleration = 1.0f;	//mm/s^2 and always positive
	float					x_current_acceleration = 1.0f;

	float					y_command_speed	= 5.0f;
	float					y_current_speed	= 0.0f;
	float					y_target_speed = 0.0f;
	float					y_command_acceleration = 1.0f;
	float					y_current_acceleration = 1.0f;


	bool					is_ready		= false;

	uint64_t				last_micros 	= 0L;

public:
	//------------------------------ Public functions  -------------------------------------
	MotorDriver();
	void begin();
	bool IsReady();
	void SetXY(float x, float y);
	void SetXY(float x, float y, float x_speed, float y_speed);
	void SetXY(float x, float y, float x_speed, float y_speed, float x_acceleration, float y_acceleration);
	void GetXY(float * x, float * y);
	void SetXYCurrentPosition(float x, float y);
	motor_state_t GetXState();
	motor_state_t GetYState();
	bool IsXStartLimitSwitchTriggered();
	bool IsXEndLimitSwitchTriggered();
	bool IsYStartLimitSwitchTriggered();
	bool IsYEndLimitSwitchTriggered();

	// There are the limit switch events. The class variables are not accessible from ISR. We call these function to resert the variables.
	void LimitXStartEvent();
	void LimitXEndEvent();
	void LimitYStartEvent();
	void LimitYEndEvent();

protected:

private:
	//------------------------------ Private functions  ------------------------------------
	static TaskFunction_t 	TaskStart(void * pvParameters);
	void 					MainTask();
	void					PrintStackWatermark();
	void					TimerInit();
	void 					StartPulses(uint32_t x_pulse_frequency, uint32_t y_pulse_frequency);
	void 					StartPulses(float x_speed, float y_speed);
	void 					StopPulses();

	uint32_t				XSpeedToFrequency(float speed);
	uint32_t				YSpeedToFrequency(float speed);

	void					UpdatePulseFrequency(void);
	void 					UpdateXState(void);
	void 					UpdateYState(void);

	float					GetXrem();
	float					GetYrem();

	void					SetXTargetSpeedToMin(float x_cur);
	void					SetYTargetSpeedToMin(float y_cur);

	void 					SetXCurrentPosition(float x);
	void 					SetYCurrentPosition(float y);

	// void 					UpdateActuator();
};

extern MotorDriver motorDriver;
#endif
