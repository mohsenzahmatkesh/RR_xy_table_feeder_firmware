#ifndef __FEEDER_DRIVER_H__
#define __FEEDER_DRIVER_H__

#include <Arduino.h>

//feeder
#define mot_z_a1   25 
#define mot_z_a2   26
#define enc_mot_z_a1 22
#define enc_mot_z_a2 23
#define sw_z_top 17
#define sw_z_bot 18
#define qre_sen 21


class FeederDriver {
private:
    int32_t max_velocity = 255; 
    int32_t feed_z_pos_des = 1000;
    volatile int32_t feed_z_current_position = 0;
    volatile int8_t feed_z_moving = 0;
    bool homing_complete = false;
    uint32_t pos_tolerance_big = 50;
    uint32_t pos_tolerance_small = 5;
    
    int32_t qre_filtered = 100; 
    int32_t qre_tau = 0;
    uint64_t completed_timestamp = 0;
    uint32_t error = 0;
    
    byte sw_z_top_status = 0;
    byte sw_z_bot_status = 0;

public:
    void begin();
    void update(); // Replaces the old loop()
    void homing();
    void stopmotor();
    
    // Getters for Commands.cpp to build the packet
    int32_t GetCurrentZ() { return feed_z_current_position; }
    int32_t GetDesiredZ() { return feed_z_pos_des; }
    int32_t GetQRE() { return qre_filtered; }
    int32_t GetTau() { return qre_tau; }
    uint64_t GetTimestamp() { return completed_timestamp; }
    uint8_t GetStatusByte() { return sw_z_bot_status + (2 * sw_z_top_status) + (64 * abs(feed_z_moving)); }
    uint8_t GetError() { return error; }

    // Setters for Commands.cpp when a packet arrives
    void SetTarget(int32_t pos, int32_t vel) { feed_z_pos_des = pos; max_velocity = vel; }
    void OverridePosition(int32_t pos) { feed_z_current_position = pos; }
    void SetTolerances(int32_t big, int32_t small) { pos_tolerance_big = big; pos_tolerance_small = small; }
    void SetTau(int32_t tau) { qre_tau = tau; }

    // ISR wrapper
    void HandleEncoder();
};

extern FeederDriver feederDriver;
#endif