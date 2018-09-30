#ifndef APP_IO_H
#define APP_IO_H

#include "stdint.h"
#include "stdbool.h"

#define MAX_PUMP_DRIVE ( 1023 )
#define NUM_SOLENOIDS ( 6 )

// Set up I/O for application
void appio_init();

// Set pump demand 0-1023
void appio_set_pump_drive(uint32_t level);

uint32_t appio_get_pump_drive();

// Turn solenoid 0-5 on/off, true for on/open
void appio_set_solenoid(uint32_t solenoid_index, bool open_state);

// Get solenoid state, based on register setting
bool appio_get_solenoid(uint32_t solenoid_index);

// True if any solenoid open based on saved state
bool appio_any_solenoid_open();

// So we don't have to remember
void appio_close_all_solenoids();

// Status light
void appio_set_blinkenlight(bool state);

int32_t appio_get_outlet_pressure_mbar();

int32_t appio_get_tank_level_percent_x10();

int32_t appio_get_pump_current_milliamps();

int32_t appio_get_supply_voltage_millivolts();

bool appio_get_flow_sensor_state();
uint16_t appio_get_flow_count();

#endif
