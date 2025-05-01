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
extern const station_t *orange_line_stations[];
extern const station_t *main_green_line_stations[];
extern const station_t *b_green_line_stations[];
extern const station_t *c_green_line_stations[];
extern const station_t *d_green_line_stations[];
extern const station_t *e_green_line_stations[];


extern int red_line_len;
extern int blue_line_len;
extern int orange_line_len;
extern int main_green_len;
extern int b_green_len;
extern int c_green_len;
extern int d_green_len;
extern int e_green_len;



uint32_t setStation(const station_t *station, int amplitude, i2c_master_dev_handle_t U1, i2c_master_dev_handle_t U2);
uint32_t clearStation(const station_t *station, i2c_master_dev_handle_t U1, i2c_master_dev_handle_t U2);
uint32_t updateLine(const station_t *line[], unsigned char *line_data, int line_len, int amplitude, i2c_master_dev_handle_t U1, i2c_master_dev_handle_t U2 );

#endif