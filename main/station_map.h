#ifndef STATION_MAP_H_
#define STATION_MAP_H_

#include "stdbool.h"
#include "stdint.h"
#include "driver/i2c_master.h"


typedef struct 
{
    uint16_t addr; //expecting in the format of CSx_SWy
    uint8_t chip;  //expecting 1 or 2 to indicate U1 or U2

}station_t;

extern const station_t *red_line_stations[];
extern const station_t *blue_line_stations[];

uint32_t setStation(const station_t *station, int amplitude, i2c_master_dev_handle_t U1, i2c_master_dev_handle_t U2);
uint32_t clearStation(const station_t *station, i2c_master_dev_handle_t U1, i2c_master_dev_handle_t U2);

#endif