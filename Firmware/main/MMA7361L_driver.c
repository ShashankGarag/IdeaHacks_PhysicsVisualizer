#include "MMA7361L_driver.h"

void MMA7361L_Init (MMA7361L* sensor) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &sensor->adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(sensor->adc_handle, sensor->x_chan, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(sensor->adc_handle, sensor->y_chan, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(sensor->adc_handle, sensor->z_chan, &config));
}

adc_data read_adc (MMA7361L* sensor) {
    int raw_x, raw_y, raw_z;
    adc_data data;

    adc_oneshot_read(sensor->adc_handle, sensor->x_chan, &raw_x);
    adc_oneshot_read(sensor->adc_handle, sensor->y_chan, &raw_y);
    adc_oneshot_read(sensor->adc_handle, sensor->z_chan, &raw_z);

    data.voltage_x = (float)raw_x;
    data.voltage_y = (float)raw_y;
    data.voltage_z = (float)raw_z;

    return data;
}
