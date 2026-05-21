/*
 * MotorDriver.cpp
 *
 *  Created on: Nov 18, 2021
 *      Author: M.Saadat (m.saadat@mail.com)
 * Github: https://github.com/mahmood-saadat/ESP32-Templates/tree/master/Task%20Templates
 * This file provide as is with no guarantee of any sort.
 * Any modification and redistribution of this file is allowed as long as this description is kept at the top of the file.
 */

#include "MotorDriver.h"

#include "debug.h"

#include <esp_task_wdt.h>
#include <Arduino.h>

#define		MOTOR_DRIVER_LOOP_MS							5
#define 	MOTOR_DRIVER_WDT_PULSE_GENERATOR_TASK_TIMEOUT 	5000
#define		MOTOR_DRIVER_ESP_FREQUENCY						80000000
#define		MOTOR_DRIVER_TIMER_DIVIDER						2

/**
 *    __________________________________________________
 *   |							X Axis 					|
 *   |					Stepper Motor					|
 *   |	 	Pulse								IO14	|
 *   |		Direction							IO13	|
 *   |													|
 *   |		Start Limit switch(Normally Close)	IO4		|
 *   |		End limit switch(Normally Close)	IO16	|
 *    __________________________________________________
 *
 *    __________________________________________________
 *   |							Y Axis 					|
 *   |					Stepper Motor					|
 *   |	 	Pulse								IO12	|
 *   |		Direction							IO5		|
 *   |													|
 *   |		Start Limit switch(Normally Close)	IO2		|
 *   |		End limit switch(Normally Close)	IO15	|
 *    __________________________________________________
 */
#define		MOTOR_DRIVER_X_PULSE_PIN						14
#define		MOTOR_DRIVER_X_DIRECTION_PIN					13
#define		MOTOR_DRIVER_Y_PULSE_PIN						12
#define		MOTOR_DRIVER_Y_DIRECTION_PIN					5

#define		MOTOR_DRIVER_X_START_LIMIT_SWITCH_PIN			4
#define		MOTOR_DRIVER_Y_START_LIMIT_SWITCH_PIN			2
#define		MOTOR_DRIVER_X_END_LIMIT_SWITCH_PIN				16
#define		MOTOR_DRIVER_Y_END_LIMIT_SWITCH_PIN				15

// The physical characteristic of the screw and stepper driver setting
#define		MOTOR_DRIVER_X_SCREW_PITCH						4.0f	// mm
#define		MOTOR_DRIVER_X_PULSE_PER_REVOLUTION				1000
#define		MOTOR_DRIVER_Y_SCREW_PITCH						4.0f	// mm
#define		MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION				1000


// // --- Linear Actuator Definitions ---
// #define ACT_RPWM 17
// #define ACT_LPWM 18
// #define ACT_REN  19
// #define ACT_LEN  21

// // Linear Actuator Encoder Definitions
// #define ACT_ENC_A 34
// #define ACT_ENC_B 35


// #define ACT_PULSES_PER_MM 17.4f // From datasheet 

// #define ACT_ANCHOR_X  139.0f 
// #define ACT_ANCHOR_Y  0.0f
// #define ACT_MAX_SPEED 14.0f 

// volatile int32_t act_pulse_count = 0;


// The course of movement
#define		MOTOR_DRIVER_X_COARSE							278.0f//580.0f//364.0f
#define		MOTOR_DRIVER_Y_COARSE							540.0f//610.0f

#define		MOTOR_DRIVER_X_DEFAULT_ACCELERAYION				20.0f// mm/s^2
#define		MOTOR_DRIVER_Y_DEFAULT_ACCELERAYION				20.0f// mm/s^2

#define		MOTOR_DRIVER_X_MAX_ACCELERAYION					100.0f// mm/s^2
#define		MOTOR_DRIVER_Y_MAX_ACCELERAYION					100.0f// mm/s^2

// We use a small non-zero value for minimum speed. This will make sure that when we are close to the target(in dead-band), the
// speed will not be completely zero and slowly we reach to the exact position.
#define		MOTOR_DRIVER_X_MIN_SPEED						0.01f// mm/s
#define		MOTOR_DRIVER_Y_MIN_SPEED						0.01f// mm/s
#define		MOTOR_DRIVER_X_MAX_SPEED						40.0f// mm/s
#define		MOTOR_DRIVER_Y_MAX_SPEED						40.0f// mm/s

// The acceptable error in position.
#define		MOTOR_DRIVER_POSITION_DEADBAND					0.05f
#define		MOTOR_DRIVER_POSITION_REM_MIN					MOTOR_DRIVER_POSITION_DEADBAND


#define		ABS(x)  		(x<0)?-x:x
#define		SIGN(x)			((x>=0)?(int)1:(int)-1)

// Defining the variable. The prototype is in header file. After including the header, we can use this variable.
MotorDriver 			motorDriver;

// The timers variables. This struct will be used in manipulating the timer event
hw_timer_t * x_pulse_generator_timer = NULL;
hw_timer_t * y_pulse_generator_timer = NULL;

// These are used in isr. We can not use the class members unless we make it public or make static setter and getters. For simplicity, I declared them global here.
static float					x_command_position 			= 0.0f;
static float					y_command_position 			= 0.0f;
static bool						is_x_speed_positive 		= true;
static bool						is_y_speed_positive 		= true;

// Each timer interrupt will be used to toggle the pulse pin. This means, the number of interrupt for a specific number of pulses
// need to be twice of the number of pulses. We keep the doubled number in these variables.
static int32_t					x_current_half_pulse_counter			= 0;
static int32_t					y_current_half_pulse_counter			= 0;
// The target number of pulses are in this variable. This will be calculated from the input command.
static int32_t					x_target_half_pulse_counter				= 0;
static int32_t					y_target_half_pulse_counter				= 0;

static bool						is_x_zero_detected						= false;
static bool						is_y_zero_detected						= false;
static bool						is_x_end_detected						= false;
static bool						is_y_end_detected						= false;

/**
 * @brief This ISR will be called on the selected timeouts.
 * This ISR generates the pulses for the X axis stepper motor driver and set the direction pin.
 * In this ISR the states of the limit switches is checked before pulsing the driver.
 *
 */
void IRAM_ATTR onXTimer()
{
	// Check for direction of speed, and the state of movement. If we are not in idle mode at the desired position, we pulse the stepper motor.
	// Otherwise, we already reached the destination. We don't need to pulse the stepper motor.
	if(is_x_speed_positive && ((x_current_half_pulse_counter != x_target_half_pulse_counter) ||
			(motorDriver.GetXState() != MotorDriver::IDLE)))
	{
		// Only pulse the motor, if we are not reached the end limit switch.
		if(digitalRead(MOTOR_DRIVER_X_END_LIMIT_SWITCH_PIN) != 1)
		{
			digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 1);
			digitalWrite(MOTOR_DRIVER_X_PULSE_PIN, !digitalRead(MOTOR_DRIVER_X_PULSE_PIN));
			x_current_half_pulse_counter ++;
		}
	}
	//else if(x_current_half_pulse_counter > x_target_half_pulse_counter)
	else if(!is_x_speed_positive && ((x_current_half_pulse_counter != x_target_half_pulse_counter) ||
			(motorDriver.GetXState() != MotorDriver::IDLE)))
	{
		if(digitalRead(MOTOR_DRIVER_X_START_LIMIT_SWITCH_PIN) != 1)
		{
			digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 0);
			digitalWrite(MOTOR_DRIVER_X_PULSE_PIN, !digitalRead(MOTOR_DRIVER_X_PULSE_PIN));
			x_current_half_pulse_counter --;
		}
	}
	// Check the limit switches
	if(digitalRead(MOTOR_DRIVER_X_START_LIMIT_SWITCH_PIN) == 1)
	{
		digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 1);
		motorDriver.LimitXStartEvent();
		// Put a negative value, so we move away from the limit switch.
		x_current_half_pulse_counter = -1500;
		x_target_half_pulse_counter = 0;
		x_command_position = 0.0f;
		is_x_zero_detected = true;
	}
	if(digitalRead(MOTOR_DRIVER_X_END_LIMIT_SWITCH_PIN) == 1)
	{
		digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 0);
		motorDriver.LimitXEndEvent();
		is_x_end_detected = true;
	}

}

/**
 * @brief This ISR will be called on the selected timeouts.
 * This ISR generate the pulses for the Y axis stepper motor driver and set the direction pin.
 * In this ISR the states of the limit switches is checked before pulsing the driver.
 *
 */
void IRAM_ATTR onYTimer()
{
	// Check for direction of speed, and the state of movement. If we are not in idle mode at the desired position, we pulse the stepper motor.
	// Otherwise, we already reached the destination. We don't need to pulse the stepper motor.
	if(is_y_speed_positive && ((y_current_half_pulse_counter != y_target_half_pulse_counter) ||
			(motorDriver.GetYState() != MotorDriver::IDLE)))
	{
		// Pulse the motor only if the we didn't reach the end limit switch
		if(digitalRead(MOTOR_DRIVER_Y_END_LIMIT_SWITCH_PIN) != 1)
		{
			digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 1);
			digitalWrite(MOTOR_DRIVER_Y_PULSE_PIN, !digitalRead(MOTOR_DRIVER_Y_PULSE_PIN));
			y_current_half_pulse_counter ++;
		}
	}
	//else if(y_current_half_pulse_counter > y_target_half_pulse_counter)
	else if(!is_y_speed_positive && ((y_current_half_pulse_counter != y_target_half_pulse_counter) ||
			(motorDriver.GetYState() != MotorDriver::IDLE)))
	{
		if(digitalRead(MOTOR_DRIVER_Y_START_LIMIT_SWITCH_PIN) != 1)
		{
			digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 0);
			digitalWrite(MOTOR_DRIVER_Y_PULSE_PIN, !digitalRead(MOTOR_DRIVER_Y_PULSE_PIN));
			y_current_half_pulse_counter --;
		}
	}
	// Check the limit switches
	if(digitalRead(MOTOR_DRIVER_Y_START_LIMIT_SWITCH_PIN) == 1)
	{
		digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 1);
		motorDriver.LimitYStartEvent();
		// Put a negative value, so we move away from the limit switch.
		y_current_half_pulse_counter = -1500;
		y_target_half_pulse_counter = 0;
		y_command_position = 0.0f;
		is_y_zero_detected = true;
	}
	if(digitalRead(MOTOR_DRIVER_Y_END_LIMIT_SWITCH_PIN) == 1)
	{
		digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 0);
		motorDriver.LimitYEndEvent();
		is_y_end_detected = true;
	}
}

MotorDriver::MotorDriver(){
	mainTask = NULL;
}

// void IRAM_ATTR actEncoderISR() {
//     // If Channel B is high when A rises, we are extending (Signal A leads B) 
//     if (digitalRead(ACT_ENC_B) == HIGH) {
//         act_pulse_count++; 
//     } else {
//         // If Channel B is low, we are retracting 
//         act_pulse_count--; 
//     }
// }

/*
 * Initialise the class
 */
void MotorDriver::begin(){
	//Set pin directions
	pinMode(MOTOR_DRIVER_X_PULSE_PIN, OUTPUT);
	pinMode(MOTOR_DRIVER_X_DIRECTION_PIN, OUTPUT);
	pinMode(MOTOR_DRIVER_Y_PULSE_PIN, OUTPUT);
	pinMode(MOTOR_DRIVER_Y_DIRECTION_PIN, OUTPUT);

	// Enable the pull-ups on limit switches inputs
	pinMode(MOTOR_DRIVER_X_START_LIMIT_SWITCH_PIN, INPUT_PULLUP);
	pinMode(MOTOR_DRIVER_Y_START_LIMIT_SWITCH_PIN, INPUT_PULLUP);
	pinMode(MOTOR_DRIVER_X_END_LIMIT_SWITCH_PIN, INPUT_PULLUP);
	pinMode(MOTOR_DRIVER_Y_END_LIMIT_SWITCH_PIN, INPUT_PULLUP);

	digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 0);
	digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 0);

	// // Actuator Motor Pins
	// pinMode(ACT_RPWM, OUTPUT);
	// pinMode(ACT_LPWM, OUTPUT);
	// pinMode(ACT_REN, OUTPUT);
	// pinMode(ACT_LEN, OUTPUT);

	// digitalWrite(ACT_REN, HIGH);
	// digitalWrite(ACT_LEN, HIGH);

	// // Actuator Encoder Pins
	// pinMode(ACT_ENC_A, INPUT);
  // pinMode(ACT_ENC_B, INPUT);

	// // Attach the interrupt to Channel A
	// attachInterrupt(digitalPinToInterrupt(ACT_ENC_A), actEncoderISR, RISING);

	// Initialize the timers. After initialization, we can change the frequency of interrups. The interrupt the pulse for stepper motor is generated.
	TimerInit();

	// Create the task to manage the motor
	xTaskCreate((TaskFunction_t)MotorDriver::TaskStart, "MotorDriverTask", 8192, this, 50, &mainTask);

}

/*
 * Task function needed to be static and in static members, we could not access class members easily,
 * so we created this function and we passed the class as input argument, then we call our main function from here.
 */
TaskFunction_t MotorDriver::TaskStart(void * pvParameters)
{
	MotorDriver* manager;
	esp_task_wdt_init(MOTOR_DRIVER_WDT_PULSE_GENERATOR_TASK_TIMEOUT, true);
	esp_task_wdt_add(NULL);
	manager = (MotorDriver*) pvParameters;
	manager->MainTask();
	// It will never get here. The main task never returns.
	return 0;
}
/*
 * The main loop
 */
void MotorDriver::MainTask()
{
	uint16_t time_counter = 300;

	// Set the initial command for the each axis to a maximum negative value and 5 mm margin to make sure the mechanism reaches the limit switch.
	SetXY(-MOTOR_DRIVER_X_COARSE - 15.0f, -MOTOR_DRIVER_Y_COARSE - 15.0f, 10.0f, 10.0f);

	DEBUG_printf("Detecting the limit switches.");

	// Wait for limit switch event
	while(is_x_zero_detected == false || is_y_zero_detected == false)
	{
		// Calculate the speed from default acceleration and update the timer frequency
		UpdatePulseFrequency();
		esp_task_wdt_reset();
		vTaskDelay(MOTOR_DRIVER_LOOP_MS/portTICK_PERIOD_MS);
	}

	DEBUG_printf("Small delay to separate from limit switch.");

	// Some delay to make sure the setup is separated from limit switch
	while((time_counter--) > 0)
	{
		UpdatePulseFrequency();
		esp_task_wdt_reset();
		vTaskDelay(MOTOR_DRIVER_LOOP_MS/portTICK_PERIOD_MS);
	}

	DEBUG_printf("Starting ...");

	is_ready = true;
	
	// // Actuator homing
	// DEBUG_printf("Homing Linear Actuator...");

  // // 1. Command the actuator to retract
  // analogWrite(ACT_RPWM, 0);
  // analogWrite(ACT_LPWM, 255);

  // int32_t last_pulse_count = -99999;
  // uint8_t steady_count = 0;

  // // 2. Wait until the encoder stops pulsing 
  // while (steady_count < 10) // ~500ms of no movement
  // {
  //     if (act_pulse_count == last_pulse_count) {
  //         steady_count++;
  //     } else {
  //         steady_count = 0; // It moved, reset the counter
  //     }
      
  //     last_pulse_count = act_pulse_count;
      
  //     esp_task_wdt_reset();
  //     vTaskDelay(50 / portTICK_PERIOD_MS); // check every 50ms
  // }

  // // 3. Stop the motor and reset our zero point
  // analogWrite(ACT_RPWM, 0);
  // analogWrite(ACT_LPWM, 0);
  // act_pulse_count = 0; 

  // DEBUG_printf("Linear Actuator Homed!");


	// We need the x axis to move to the end at the beginning.
	SetXY(MOTOR_DRIVER_X_COARSE, 0.0f, 15.0f, 5.0f);

	while(1)
	{
		esp_task_wdt_reset();

		UpdatePulseFrequency();
		// UpdateActuator();
		PrintStackWatermark();
		vTaskDelay(MOTOR_DRIVER_LOOP_MS/portTICK_PERIOD_MS);
	}
}

void MotorDriver::PrintStackWatermark()
{
	if(millis() > (stackWatermarkPrintLastMillis + 10000)){
		stackWatermarkPrintLastMillis = millis();
		//Serial.printf("[MotorDriver] Task Stack left: %d\r\n", uxTaskGetStackHighWaterMark(NULL));
		//Serial.printf("[MotorDriver] X Current: %d, Target: %d, freq: %d\r\n", x_current_half_pulse_counter, x_target_half_pulse_counter, x_pulse_frequency);
		//Serial.printf("[MotorDriver] Y Current: %d, Target: %d\r\n", y_current_half_pulse_counter, y_target_half_pulse_counter);
	}
}

/**
 * This method should be called periodically to update the speed.
 * Here, depending on the state of motor, the new speed will be calculated and the timer interrupt will be adjusted according to the
 * speed.
 */
void MotorDriver::UpdatePulseFrequency()
{
	// We don't keep the sign from the command. The direction of the movement could be the opposite of the input command.
	// We make sure the command is always positive and find the correct sign for smooth movement in this function.
	int8_t x_accel_sign = 1;
	int8_t y_accel_sign = 1;
	// There is a case where the new command for speed is too close to the current speed. We always calculate the
	// acceleration needed to get to target speed and saturate it with acceleration command.
	float tmp_acceleration = 1.0f;

	// These are the current position variables
	float x_cur = 0.0f;
	float y_cur = 0.0f;
	GetXY(&x_cur, &y_cur);

	// This is the delta-T variable to keep the time difference between calling this function. It will be used
	// to calculate the speed from the acceleration command.
	float dt = 0.000001f;
	if(micros() > last_micros)
	{
		dt = (micros() - last_micros);
	}
	last_micros = micros();

	// These functions are handling the state machine
	UpdateXState();
	UpdateYState();

	// According to the state, we calculate the speed
	switch(x_motor_state)
	{
	case MOTOR_STATE::IDLE:
		SetXTargetSpeedToMin(x_cur);
		x_current_speed = x_target_speed;
		break;

	case MOTOR_STATE::ACCELERATION:
	case MOTOR_STATE::DECELERATION:
		// Check if the required acceleration to get to the target speed is less than the command; otherwise, saturate it with command.
		tmp_acceleration = abs(x_target_speed - x_current_speed)/(dt/1000000.0f);
		if(tmp_acceleration > x_command_acceleration)
		{
			tmp_acceleration = x_command_acceleration;
		}
		// The sign of acceleration is always positive. According the current speed and target speed, we select the sign of acceleration.
		if(x_current_speed < x_target_speed)
		{
			x_current_speed = ((tmp_acceleration*(dt/1000000.0f)) + x_current_speed);
			if(x_current_speed > x_target_speed)
			{
				x_current_speed = x_target_speed;
			}
		}
		else
		{
			x_current_speed = ((-1*tmp_acceleration*(dt/1000000.0f)) + x_current_speed);
			if(x_current_speed < x_target_speed)
			{
				x_current_speed = x_target_speed;
			}
		}
		break;
	case MOTOR_STATE::CONSTANT_SPEED:

		break;

	default:
		break;
	}
	// To keep the interrupts fast and avoid any complications, we removed any float variable out of interrupts.
	// We need the sign of speed in the interrupt. Here we set the bool value to be used in the interrupt.
	if(x_current_speed >= 0.0f)
	{
		is_x_speed_positive = true;
	}
	else
	{
		is_x_speed_positive = false;
	}


	// According to the state, we calculate the speed
	switch(y_motor_state)
	{
	case MOTOR_STATE::IDLE:
		SetYTargetSpeedToMin(y_cur);
		y_current_speed = y_target_speed;
		break;

	case MOTOR_STATE::ACCELERATION:
	case MOTOR_STATE::DECELERATION:
		// Check if the required acceleration to get to the target speed is less than the command; otherwise, saturate it with command.
		tmp_acceleration = abs(y_target_speed - y_current_speed)/(dt/1000000.0f);
		if(tmp_acceleration > y_command_acceleration)
		{
			tmp_acceleration = y_command_acceleration;
		}
		// The sign of acceleration is always positive. According the current speed and target speed, we select the sign of acceleration.
		if(y_current_speed < y_target_speed)
		{
			y_current_speed = ((tmp_acceleration*(dt/1000000.0f)) + y_current_speed);
			if(y_current_speed > y_target_speed)
			{
				y_current_speed = y_target_speed;
			}
		}
		else
		{
			y_current_speed = ((-1*tmp_acceleration*(dt/1000000.0f)) + y_current_speed);
			if(y_current_speed < y_target_speed)
			{
				y_current_speed = y_target_speed;
			}
		}
		break;
	case MOTOR_STATE::CONSTANT_SPEED:

		break;

	default:
		break;
	}
	// To keep the interrupts fast and avoid any complications, we removed any float variable out of interrupts.
	// We need the sign of speed in the interrupt. Here we set the bool value to be used in the interrupt.
	if(y_current_speed >= 0.0f)
	{
		is_y_speed_positive = true;
	}
	else
	{
		is_y_speed_positive = false;
	}

	// Update the pulse frequency
	StartPulses(x_current_speed, y_current_speed);

//	float x_rem = GetXrem();
//	DEBUG_printf("[X] speed: %f, com: %f, tar: %f, loc; cur: %f, com: %f, rem: %f, mode: %d, dt:%f\n",
//			x_current_speed, x_command_speed, x_target_speed, x_cur, x_command_position, x_rem, x_motor_state, dt);
}

/**
 * X motor state machine
 */
void MotorDriver::UpdateXState(void)
{
	float x_rem = 1.0f*GetXrem();
	float x_cur = 0.0f;
	float y_cur = 0.0f;
	GetXY(&x_cur, &y_cur);
	float x_diff = abs(x_cur - x_command_position);

	switch(x_motor_state)
	{
	case MOTOR_STATE::IDLE:
		// Exit the idle state only if the position error is bigger than the dead-band
		if(x_diff > MOTOR_DRIVER_POSITION_DEADBAND)
		{
			if(x_command_position > x_cur)
			{
				x_target_speed = x_command_speed;
			}
			else
			{
				x_target_speed = -1*x_command_speed;
			}
			x_motor_state = MOTOR_STATE::ACCELERATION;
		}
		break;

	case MOTOR_STATE::ACCELERATION:
		// Check if we reached the target speed, change the state to constant speed
		if((SIGN(x_current_speed) == SIGN(x_target_speed)) && (abs(x_current_speed) >= abs(x_target_speed)))
		{
			x_motor_state = MOTOR_STATE::CONSTANT_SPEED;
		}
		// Check if we are close to the target and we need to start slowing done. In that case, go to the deceleration.
		if(abs(x_cur - x_command_position) < x_rem)
		{
			SetXTargetSpeedToMin(x_cur);
			x_motor_state = MOTOR_STATE::DECELERATION;
		}
		else
		{
			// If we are stating in this state, according to the position, set the sign of target speed command.
			if(x_cur < x_command_position)
			{
				x_target_speed = x_command_speed;
			}
			else
			{
				x_target_speed = -1*x_command_speed;
			}
		}
		break;
	case MOTOR_STATE::DECELERATION:
		// If the speed is close to stopping, change the state to idle.
		if(abs(x_current_speed) <= MOTOR_DRIVER_X_MIN_SPEED)
		{
			x_motor_state = MOTOR_STATE::IDLE;
			SetXTargetSpeedToMin(x_cur);
		}
		// This checks if for example because of a new command, the target position is more distant the remaining
		// distance to zero speed. In that case, move to acceleration state.
		if(abs(x_cur - x_command_position) > (3.0f*x_rem))
		{
			if(x_cur < x_command_position)
			{
				x_target_speed = x_command_speed;
			}
			else
			{
				x_target_speed = -1*x_command_speed;
			}
			x_motor_state = MOTOR_STATE::ACCELERATION;
		}
		break;
	case MOTOR_STATE::CONSTANT_SPEED:
		// If we are close to target that we need to start lowering the speed, change the state to deceleration.
		if(abs(x_cur - x_command_position) < x_rem)
		{
			SetXTargetSpeedToMin(x_cur);
			x_motor_state = MOTOR_STATE::DECELERATION;
		}
		// Check if the target speed is changed because of a new command. In such a case, change the state to acceleration.
		else if(
				((SIGN(x_current_speed) == SIGN(x_target_speed)) && (abs(x_current_speed) < abs(x_target_speed))) ||
				(SIGN(x_current_speed) != SIGN(x_target_speed))
		)
		{
			x_target_speed = SIGN(x_current_speed)*x_command_speed;
			x_motor_state = MOTOR_STATE::ACCELERATION;
		}
		// If we are not in deceleration, set the sign of the speed according to the current position.
		if(x_motor_state != MOTOR_STATE::DECELERATION)
		{
			if(x_cur < x_command_position)
			{
				x_target_speed = x_command_speed;
			}
			else
			{
				x_target_speed = -1*x_command_speed;
			}
		}
		break;

	default:
		break;
	}
}

/**
 * Y motor state machine
 */
void MotorDriver::UpdateYState(void)
{
	// Needed to add a little margin so it wouldn't switch between modes and make a smooth stop
	float y_rem = 1.0f*GetYrem();
	float x_cur = 0.0f;
	float y_cur = 0.0f;
	GetXY(&x_cur, &y_cur);
	float y_diff = abs(y_cur - y_command_position);

	switch(y_motor_state)
	{
	case MOTOR_STATE::IDLE:
		// Exit the idle state only if the position error is bigger than the dead-band
		if(y_diff > MOTOR_DRIVER_POSITION_DEADBAND)
		{
			if(y_command_position > y_cur)
			{
				y_target_speed = y_command_speed;
			}
			else
			{
				y_target_speed = -1*y_command_speed;
			}
			y_motor_state = MOTOR_STATE::ACCELERATION;
		}
		break;

	case MOTOR_STATE::ACCELERATION:
		// Check if we reached the target speed, change the state to constant speed
		if((SIGN(y_current_speed) == SIGN(y_target_speed)) && (abs(y_current_speed) >= abs(y_target_speed)))
		{
			y_motor_state = MOTOR_STATE::CONSTANT_SPEED;
		}
		// Check if we are close to the target and we need to start slowing done. In that case, go to the deceleration.
		if(abs(y_cur - y_command_position) < y_rem)
		{
			SetYTargetSpeedToMin(y_cur);
			y_motor_state = MOTOR_STATE::DECELERATION;
		}
		else
		{
			// If we are stating in this state, according to the position, set the sign of target speed command.
			if(y_cur < y_command_position)
			{
				y_target_speed = y_command_speed;
			}
			else
			{
				y_target_speed = -1*y_command_speed;
			}
		}
		break;
	case MOTOR_STATE::DECELERATION:
		// If the speed is close to stopping, change the state to idle.
		if(abs(y_current_speed) <= MOTOR_DRIVER_Y_MIN_SPEED)
		{
			y_motor_state = MOTOR_STATE::IDLE;
			SetYTargetSpeedToMin(y_cur);
		}
		// This checks if for example because of a new command, the target position is more distant the remaining
		// distance to zero speed. In that case, move to acceleration state.
		if(abs(y_cur - y_command_position) > (3.0f*y_rem))
		{
			if(y_cur < y_command_position)
			{
				y_target_speed = y_command_speed;
			}
			else
			{
				y_target_speed = -1*y_command_speed;
			}
			y_motor_state = MOTOR_STATE::ACCELERATION;
		}
		break;
	case MOTOR_STATE::CONSTANT_SPEED:
		// If we are close to target that we need to start lowering the speed, change the state to deceleration.
		if(abs(y_cur - y_command_position) < y_rem)
		{
			SetYTargetSpeedToMin(y_cur);
			y_motor_state = MOTOR_STATE::DECELERATION;
		}
		// Check if the target speed is changed because of a new command. In such a case, change the state to acceleration.
		else if(
				((SIGN(y_current_speed) == SIGN(y_target_speed)) && (abs(y_current_speed) < abs(y_target_speed))) ||
				(SIGN(y_current_speed) != SIGN(y_target_speed))
				)
		{
			y_target_speed = SIGN(y_current_speed)*y_command_speed;
			y_motor_state = MOTOR_STATE::ACCELERATION;
		}
		// If we are not in deceleration, set the sign of the speed according to the current position.
		if(y_motor_state != MOTOR_STATE::DECELERATION)
		{
			if(y_cur < y_command_position)
			{
				y_target_speed = y_command_speed;
			}
			else
			{
				y_target_speed = -1*y_command_speed;
			}
		}
		break;

	default:
		break;
	}
}

/**
 * Calculate the distance that we need to start deceleration.
 * rem = 1/2 * V^2 / a
 * The formula is obtained by removing the t parameter from x and v formula:
 * x = 1/2 * a * t^2
 * v = a * t
 */
float MotorDriver::GetXrem()
{
	float rem = 0.5f*(x_current_speed)*(x_current_speed)/x_command_acceleration;
	if(rem < MOTOR_DRIVER_POSITION_REM_MIN)
	{
		rem = MOTOR_DRIVER_POSITION_REM_MIN;
	}
	return rem;
}

/**
 * Calculate the distance that we need to start deceleration.
 * rem = 1/2 * V^2 / a
 * The formula is obtained by removing the t parameter from x and v formula:
 * x = 1/2 * a * t^2
 * v = a * t
 */
float MotorDriver::GetYrem()
{
	float rem = 0.5f*(y_current_speed)*(y_current_speed)/y_command_acceleration;
	if(rem < MOTOR_DRIVER_POSITION_REM_MIN)
	{
		rem = MOTOR_DRIVER_POSITION_REM_MIN;
	}
	return rem;
}

/**
 * Check if we already found the limit switches after boot. Before that, we don't accept any commands.
 */
bool MotorDriver::IsReady()
{
	return is_ready;
}

/**
 * Initialize the timer with default interrupt frequency
 */
void MotorDriver::TimerInit()
{
	x_pulse_generator_timer = timerBegin(0, MOTOR_DRIVER_TIMER_DIVIDER, true);
	timerAttachInterrupt(x_pulse_generator_timer, &onXTimer, true);
	/* Repeat the alarm (third parameter), set to 1 to alarm every single clock */
	timerAlarmWrite(x_pulse_generator_timer, 40000000, true);
	/* Start an alarm */
	timerAlarmEnable(x_pulse_generator_timer);

	y_pulse_generator_timer = timerBegin(1, MOTOR_DRIVER_TIMER_DIVIDER, true);
	timerAttachInterrupt(y_pulse_generator_timer, &onYTimer, true);
	/* Repeat the alarm (third parameter), set to 1 to alarm every single clock */
	timerAlarmWrite(y_pulse_generator_timer, 40000000, true);
	/* Start an alarm */
	timerAlarmEnable(y_pulse_generator_timer);
}

/**
 * Update the timer parameter according to the new frequency for pulse output
 */
void MotorDriver::StartPulses(uint32_t x_pulse_frequency, uint32_t y_pulse_frequency)
{
	uint32_t x_prescaler = 0;
	uint32_t y_prescaler = 0;
	StopPulses();

	if(x_pulse_frequency == 0)
	{
		x_pulse_frequency = 10;
	}
	if(y_pulse_frequency == 0)
	{
		y_pulse_frequency = 10;
	}

	x_prescaler = MOTOR_DRIVER_ESP_FREQUENCY/MOTOR_DRIVER_TIMER_DIVIDER/x_pulse_frequency/2;
	if(0 == x_prescaler)
	{
		x_prescaler = 1;
	}
	/* Repeat the alarm (third parameter), set to 1 to alarm every single clock */
	timerAlarmWrite(x_pulse_generator_timer, x_prescaler, true);

	y_prescaler = MOTOR_DRIVER_ESP_FREQUENCY/MOTOR_DRIVER_TIMER_DIVIDER/y_pulse_frequency/2;
	if(0 == y_prescaler)
	{
		y_prescaler = 1;
	}
	/* Repeat the alarm (third parameter), set to 1 to alarm every single clock */
	timerAlarmWrite(y_pulse_generator_timer, y_prescaler, true);

}
/**
 * Overload for converting speed to frequency
 */
void MotorDriver::StartPulses(float x_speed, float y_speed)
{
	StartPulses(XSpeedToFrequency(x_speed), YSpeedToFrequency(y_speed));
}

/**
 * Nothing to be done for stopping the pulse at the moment. If we de-initialize the timer, next time we start, there would be a delay.
 */
void MotorDriver::StopPulses()
{

}

/**
 * Set the position command with default speed
 */
void MotorDriver::SetXY(float x, float y)
{
	SetXY(x, y, x_command_speed, y_command_speed);
}

/**
 * Set the position and speed with default acceleration
 * speed is mm/s
 */
void MotorDriver::SetXY(float x, float y, float x_speed, float y_speed)
{
	SetXY(x, y, x_speed, y_speed, MOTOR_DRIVER_X_DEFAULT_ACCELERAYION, MOTOR_DRIVER_Y_DEFAULT_ACCELERAYION);
}


/**
 * Set the position, speed and acceleration
 * speed is mm/s, acceleration is mm/s^2 and positive
 */
void MotorDriver::SetXY(float x, float y, float x_speed, float y_speed, float x_acceleration, float y_acceleration)
{
	if(x < MOTOR_DRIVER_X_COARSE)
	{
		x_command_position = x;
	}
	else
	{
		x_command_position = MOTOR_DRIVER_X_COARSE;
	}
	if(y < MOTOR_DRIVER_Y_COARSE)
	{
		y_command_position = y;
	}
	else
	{
		y_command_position = MOTOR_DRIVER_Y_COARSE;
	}
	if(x_acceleration > MOTOR_DRIVER_X_MAX_ACCELERAYION)
	{
		x_acceleration = MOTOR_DRIVER_X_MAX_ACCELERAYION;
	}
	if(y_acceleration > MOTOR_DRIVER_Y_MAX_ACCELERAYION)
	{
		y_acceleration = MOTOR_DRIVER_Y_MAX_ACCELERAYION;
	}

	// The command must be positive. We only need the value. The sign of acceleration will be generated depending on current
	// position and speed.
	if(x_acceleration < 0)
	{
		x_acceleration *= -1.0f;
	}
	if(y_acceleration < 0)
	{
		y_acceleration *= -1.0f;
	}
	// The command must be positive. The sign will be calculated according to the position.
	if(x_speed < 0)
	{
		x_speed *= -1.0f;
	}
	if(y_speed < 0)
	{
		y_speed *= -1.0f;
	}

	// total desired XY speed vector
  float total_speed = sqrt(pow(x_speed, 2) + pow(y_speed, 2));

  // Scale down speeds proportionally if they exceed the actuator's max limit
  // if (total_speed > ACT_MAX_SPEED) {
  //     float scale = ACT_MAX_SPEED / total_speed;
  //     x_speed *= scale;
  //     y_speed *= scale;
  // }

	// Convert the target position command to the number of interrupts. The number of interrupts is twice the number of
	// pulses. This is because on each timer interrupt we toggle the pulse output.
	x_target_half_pulse_counter = x_command_position * MOTOR_DRIVER_X_PULSE_PER_REVOLUTION / MOTOR_DRIVER_X_SCREW_PITCH * 2;
	y_target_half_pulse_counter = y_command_position * MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION / MOTOR_DRIVER_Y_SCREW_PITCH * 2;

	// Update teh command variables
	x_command_acceleration = x_acceleration;
	y_command_acceleration = y_acceleration;
	x_command_speed = x_speed;
	y_command_speed = y_speed;

	is_command_new = true;
}

/**
 * Convert the pulse counter to the position
 */
void MotorDriver::GetXY(float * x, float * y)
{
	*x = x_current_half_pulse_counter * MOTOR_DRIVER_X_SCREW_PITCH / 2 / MOTOR_DRIVER_X_PULSE_PER_REVOLUTION;
	*y = y_current_half_pulse_counter * MOTOR_DRIVER_Y_SCREW_PITCH / 2 / MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION;
}

/**
 * Update the current position. This is used if there is an error in the position and the client knows about it(for example
 * by any type of feedback on the axis position). Client will send the correct position and here we update the variables
 * accordingly.
 */
void MotorDriver::SetXYCurrentPosition(float x, float y)
{
	x_current_half_pulse_counter = x * MOTOR_DRIVER_X_PULSE_PER_REVOLUTION / MOTOR_DRIVER_X_SCREW_PITCH * 2;
	y_current_half_pulse_counter = y * MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION / MOTOR_DRIVER_Y_SCREW_PITCH * 2;
}

void MotorDriver::SetXCurrentPosition(float x)
{
	x_current_half_pulse_counter = x * MOTOR_DRIVER_X_PULSE_PER_REVOLUTION / MOTOR_DRIVER_X_SCREW_PITCH * 2;
}

void MotorDriver::SetYCurrentPosition(float y)
{
	y_current_half_pulse_counter = y * MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION / MOTOR_DRIVER_Y_SCREW_PITCH * 2;
}

MotorDriver::MOTOR_STATE MotorDriver::GetYState()
{
	return y_motor_state;
}
MotorDriver::MOTOR_STATE MotorDriver::GetXState()
{
	return x_motor_state;
}

/**
 * Get Limit switch states
 * @return True if limit switch is triggered
 */
bool MotorDriver::IsXStartLimitSwitchTriggered()
{
	return (0 == digitalRead(MOTOR_DRIVER_X_START_LIMIT_SWITCH_PIN));
}
bool MotorDriver::IsXEndLimitSwitchTriggered()
{
	return (0 == digitalRead(MOTOR_DRIVER_X_END_LIMIT_SWITCH_PIN));
}
bool MotorDriver::IsYStartLimitSwitchTriggered()
{
	return (0 == digitalRead(MOTOR_DRIVER_Y_START_LIMIT_SWITCH_PIN));
}
bool MotorDriver::IsYEndLimitSwitchTriggered()
{
	return (0 == digitalRead(MOTOR_DRIVER_Y_END_LIMIT_SWITCH_PIN));
}

/**
 * At the moment, for both start and end, we just need to reset everything
 */
void MotorDriver::LimitXStartEvent()
{
	x_command_speed = MOTOR_DRIVER_X_MIN_SPEED;
	x_target_speed = MOTOR_DRIVER_X_MAX_SPEED;
	x_current_speed = MOTOR_DRIVER_X_MAX_SPEED/10.0f;
	x_command_acceleration = MOTOR_DRIVER_X_DEFAULT_ACCELERAYION;
	x_current_acceleration = 0.0f;
	x_motor_state = MOTOR_STATE::IDLE;
}
void MotorDriver::LimitXEndEvent()
{
	x_command_speed = MOTOR_DRIVER_X_MAX_SPEED/5;
//	SetXCurrentPosition(MOTOR_DRIVER_X_COARSE + 5.0f);
//	x_target_speed = 0.0f;
//	x_current_speed = 0.0f;
	x_command_acceleration = MOTOR_DRIVER_X_DEFAULT_ACCELERAYION;
//	x_current_acceleration = 0.0f;
//	x_motor_state = MOTOR_STATE::IDLE;
}
void MotorDriver::LimitYStartEvent()
{
	y_command_speed = MOTOR_DRIVER_Y_MIN_SPEED;
	y_target_speed = MOTOR_DRIVER_Y_MAX_SPEED;
	y_current_speed = MOTOR_DRIVER_Y_MAX_SPEED/10.0f;
	y_command_acceleration = MOTOR_DRIVER_Y_DEFAULT_ACCELERAYION;
	y_current_acceleration = 0.0f;
	y_motor_state = MOTOR_STATE::IDLE;
}
void MotorDriver::LimitYEndEvent()
{
	y_command_speed = MOTOR_DRIVER_Y_MAX_SPEED/5;
//	SetYCurrentPosition(MOTOR_DRIVER_Y_COARSE + 5.0f);
//	y_target_speed = 0.0f;
//	y_current_speed = 0.0f;
	y_command_acceleration = MOTOR_DRIVER_Y_DEFAULT_ACCELERAYION;
//	y_current_acceleration = 0.0f;
//	y_motor_state = MOTOR_STATE::IDLE;
}

/**
 * Convert the speed parameter to the pulse frequency. This will be done knowing the screw pitch and stepper motor driver resolution.
 */
uint32_t MotorDriver::XSpeedToFrequency(float speed)
{
	uint32_t frequency = abs(speed) * MOTOR_DRIVER_X_PULSE_PER_REVOLUTION / MOTOR_DRIVER_X_SCREW_PITCH;
	if(frequency < 10)
	{
		frequency = 10;
	}
	if(frequency > 20000)
	{
		frequency = 20000;
	}
	return (frequency);
}
/**
 * Convert the speed parameter to the pulse frequency. This will be done knowing the screw pitch and stepper motor driver resolution.
 */
uint32_t MotorDriver::YSpeedToFrequency(float speed)
{
	uint32_t frequency = abs(speed) * MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION / MOTOR_DRIVER_Y_SCREW_PITCH;
	if(frequency < 10)
	{
		frequency = 10;
	}
	if(frequency > 20000)
	{
		frequency = 20000;
	}
	return (frequency);
}
/**
 * Check the current position and according to that, determine the sign of the minimum speed. The will be helpful when we are
 * entering the idle mode.
 */
void MotorDriver::SetXTargetSpeedToMin(float x_cur)
{
	if(x_cur < x_command_position)
	{
		x_target_speed = MOTOR_DRIVER_X_MIN_SPEED;
	}
	else
	{
		x_target_speed = -1.0f*MOTOR_DRIVER_X_MIN_SPEED;
	}
}
/**
 * Check the current position and according to that, determine the sign of the minimum speed. The will be helpful when we are
 * entering the idle mode.
 */
void MotorDriver::SetYTargetSpeedToMin(float y_cur)
{
	if(y_cur < y_command_position)
	{
		y_target_speed = MOTOR_DRIVER_Y_MIN_SPEED;
	}
	else
	{
		y_target_speed = -1.0f*MOTOR_DRIVER_Y_MIN_SPEED;
	}
}

// void MotorDriver::UpdateActuator() {
//     float x_cur = 0.0f;
//     float y_cur = 0.0f;
//     GetXY(&x_cur, &y_cur);

//     // 1. Calculate target length based on XY chord
//     float target_length_mm = sqrt(pow(x_cur - ACT_ANCHOR_X, 2) + pow(y_cur - ACT_ANCHOR_Y, 2));

//     // 2. Calculate current physical length from the encoder
//     float current_length_mm = (float)act_pulse_count / ACT_PULSES_PER_MM;

//     // 3. Calculate the error
//     float error = target_length_mm - current_length_mm;

//     // 4. Deadband to prevent stalling and oscillation (e.g., +/- 1.0 mm)
//     float deadband = 1.0f; 

//     // 5. Drive the motor based on the error
//     if (error > deadband) {
//         // We need to extend to reach the target
//         analogWrite(ACT_RPWM, 255);
//         analogWrite(ACT_LPWM, 0);
//     } 
//     else if (error < -deadband) {
//         // We need to retract to reach the target
//         analogWrite(ACT_RPWM, 0);
//         analogWrite(ACT_LPWM, 255);
//     } 
//     else {
//         // We are within the acceptable deadband; stop the motor
//         analogWrite(ACT_RPWM, 0);
//         analogWrite(ACT_LPWM, 0);
//     }
// }
