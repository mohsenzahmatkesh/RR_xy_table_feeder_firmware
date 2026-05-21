/*
 * FeederDriver.cpp
 *
 *  Created on: May 21, 2026
 *      Author: Mohsen Zahmatkesh (mohsenzahmatkesh1992@gmail.com)
 * Github: https://github.com/mahmood-saadat/ESP32-Templates/tree/master/Task%20Templates
 * This file provide as is with no guarantee of any sort.
 * Any modification and redistribution of this file is allowed as long as this description is kept at the top of the file.
 */

#include "FeederDriver.h"
#include <esp_task_wdt.h>

FeederDriver feederDriver;

// ISR must be outside the class
void IRAM_ATTR onZEncoder() {
    feederDriver.HandleEncoder();
}

void FeederDriver::HandleEncoder() {
    if (feed_z_moving > 0){
         feed_z_current_position++;
    } else if (feed_z_moving < 0){
        if (digitalRead(sw_z_top) == 1)
            feed_z_current_position--;
        else {
            feed_z_current_position = 0;
            stopmotor();
        }
    }
}

void FeederDriver::begin() {
    pinMode(mot_z_a1, OUTPUT);
    pinMode(mot_z_a2, OUTPUT);
    pinMode(enc_mot_z_a1, INPUT_PULLUP);
    pinMode(enc_mot_z_a2, INPUT_PULLUP);
    pinMode(sw_z_top, INPUT_PULLUP);
    pinMode(sw_z_bot, INPUT_PULLUP);
    pinMode(qre_sen, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(enc_mot_z_a1), onZEncoder, RISING);
}

void FeederDriver::stopmotor() {
    analogWrite(mot_z_a1, 0);   
    analogWrite(mot_z_a2, 0);
}

void FeederDriver::homing() {
    if (homing_complete) return;
    
    // Note: Replaced delay(1) with vTaskDelay to prevent Watchdog Timer crashes
    while (digitalRead(sw_z_bot) != 0) {
        analogWrite(mot_z_a1, 200);   
        analogWrite(mot_z_a2, 0);
        esp_task_wdt_reset();
        vTaskDelay(1 / portTICK_PERIOD_MS); 
    }
    feed_z_current_position = 0;  
    stopmotor();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    int32_t target = 1000;
    feed_z_moving = 1;
    
    while (feed_z_current_position < target) {
        analogWrite(mot_z_a1, 0);  
        analogWrite(mot_z_a2, 200);
        esp_task_wdt_reset();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    stopmotor();
    feed_z_moving = 0;
    homing_complete = true;   
}

void FeederDriver::update() {
    sw_z_top_status = 1 - digitalRead(sw_z_top);
    sw_z_bot_status = 1 - digitalRead(sw_z_bot);

    // Read QRE
    if (qre_filtered < 0) qre_filtered = analogRead(qre_sen);
    qre_filtered = (qre_filtered * qre_tau + analogRead(qre_sen) * (1000 - qre_tau)) / 1000;

    // Movement Logic
    if (!homing_complete) return;
    
    int32_t feed_z_error = feed_z_pos_des - feed_z_current_position;
    bool feed_z_move = (abs(feed_z_error) > pos_tolerance_big  && feed_z_moving == 0) ||
                       (abs(feed_z_error) > pos_tolerance_small && feed_z_moving != 0);
  
    if (feed_z_move) {
        if (feed_z_error > 0 ) feed_z_moving = 1;     
        else if (feed_z_error < 0 && sw_z_bot_status == 0) feed_z_moving = -1;    
        else feed_z_moving = 0;
    } else {
        feed_z_moving = 0;
    }
    
    int speed_z = abs(feed_z_error);
    if (speed_z > 255) speed_z = 255;
    if (speed_z > max_velocity) speed_z = max_velocity;

    if (feed_z_moving == 1) {
        analogWrite(mot_z_a1, 0);
        analogWrite(mot_z_a2, speed_z);
    } else if (feed_z_moving == -1) {
        analogWrite(mot_z_a1, speed_z);
        analogWrite(mot_z_a2, 0);
    } else {
        stopmotor();
    }
}