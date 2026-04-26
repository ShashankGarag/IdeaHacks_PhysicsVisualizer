#ifndef MMA7361L_DRIVERH
#define MMA7361L_DRIVERH

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

typedef struct {
    adc_oneshot_unit_handle_t adc_handle;
    adc_channel_t x_chan;
    adc_channel_t y_chan;
    adc_channel_t z_chan;
} MMA7361L;

typedef struct {
    float voltage_x;
    float voltage_y;
    float voltage_z;
} adc_data;

void MMA7361L_Init (MMA7361L* sensor);
adc_data read_adc (MMA7361L* sensor);

#endif