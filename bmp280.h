/**
 * @file bmp280.h
 * @brief Define a interface pública (API) para o driver do sensor de pressão e temperatura BMP280.
 *
 * Este arquivo de cabeçalho declara as estruturas de dados e as funções que
 * podem ser chamadas por outras partes do sistema para interagir com o sensor.
 */

/**
 * https://github.com/ASCCJR
 */

#ifndef BMP280_H
#define BMP280_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define BMP280_ADDR 0x76

typedef struct {
    uint16_t dig_t1;
    int16_t dig_t2, dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2, dig_p3, dig_p4, dig_p5, dig_p6, dig_p7, dig_p8, dig_p9;
} bmp280_calib_params_t;


void bmp280_init(i2c_inst_t* i2c);
void bmp280_read_raw(i2c_inst_t* i2c, int32_t *temp, int32_t *pressure);
void bmp280_get_calib_params(i2c_inst_t* i2c, bmp280_calib_params_t *params);

// Funções de conversão de temperatura e pressão
int32_t bmp280_convert_temp(int32_t raw_temp, bmp280_calib_params_t *params);
int32_t bmp280_convert_pressure(int32_t raw_pressure, bmp280_calib_params_t *params);

#endif
