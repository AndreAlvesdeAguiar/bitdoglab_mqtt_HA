/**
 * @file bmp280.c
 * @brief Implementação do driver para o sensor de pressão e temperatura BMP280.
 *
 * Contém a lógica de baixo nível para comunicação I2C e a implementação
 * das fórmulas de compensação conforme o datasheet da Bosch.
 */

/**
 * https://github.com/ASCCJR
 */

#include "bmp280.h"

/**
 * @brief Variável interna para armazenar o valor de compensação de temperatura.
 * É calculada na função de temperatura e usada como entrada na função de pressão.
 * O uso de 'static' a torna "privada" a este arquivo.
 */
static int32_t t_fine;

/**
 * @brief Função helper interna para ler uma sequência de bytes de um registrador.
 */
static void read_bytes(i2c_inst_t* i2c, uint8_t reg, uint8_t *buf, uint16_t len) {
    // Seleciona o registrador inicial para leitura
    i2c_write_blocking(i2c, BMP280_ADDR, &reg, 1, true); // 'true' para manter o controle do barramento
    // Lê a sequência de bytes
    i2c_read_blocking(i2c, BMP280_ADDR, buf, len, false);
}

void bmp280_init(i2c_inst_t* i2c) {
    // Configura o sensor com oversampling x16 para temp/pressão e filtro IIR x16.
    uint8_t buf[2];
    buf[0] = 0xF4; // Registrador ctrl_meas
    buf[1] = 0xFF; // temp_os=x16, press_os=x16, modo normal
    i2c_write_blocking(i2c, BMP280_ADDR, buf, 2, false);

    buf[0] = 0xF5; // Registrador config
    buf[1] = 0x14; // standby 0.5ms, filtro IIR x16
    i2c_write_blocking(i2c, BMP280_ADDR, buf, 2, false);
}

void bmp280_get_calib_params(i2c_inst_t* i2c, bmp280_calib_params_t *params) {
    uint8_t buf[24];
    read_bytes(i2c, 0x88, buf, 24);

    // Converte os bytes (little-endian) para os tipos de dados corretos.
    params->dig_t1 = (buf[1] << 8) | buf[0];
    params->dig_t2 = (buf[3] << 8) | buf[2];
    params->dig_t3 = (buf[5] << 8) | buf[4];
    params->dig_p1 = (buf[7] << 8) | buf[6];
    params->dig_p2 = (buf[9] << 8) | buf[8];
    params->dig_p3 = (buf[11] << 8) | buf[10];
    params->dig_p4 = (buf[13] << 8) | buf[12];
    params->dig_p5 = (buf[15] << 8) | buf[14];
    params->dig_p6 = (buf[17] << 8) | buf[16];
    params->dig_p7 = (buf[19] << 8) | buf[18];
    params->dig_p8 = (buf[21] << 8) | buf[20];
    params->dig_p9 = (buf[23] << 8) | buf[22];
}

void bmp280_read_raw(i2c_inst_t* i2c, int32_t *temp, int32_t *pressure) {
    uint8_t buf[6];
    read_bytes(i2c, 0xF7, buf, 6);
    // Monta os valores de 20 bits a partir dos bytes lidos.
    *pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    *temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
}

// As duas funções abaixo são implementações diretas das fórmulas do datasheet da Bosch.
// É crucial usar tipos de 32 e 64 bits para os cálculos intermediários para evitar overflow.
int32_t bmp280_convert_temp(int32_t raw_temp, bmp280_calib_params_t *params) {
    int32_t var1, var2;
    var1 = ((((raw_temp >> 3) - ((int32_t) params->dig_t1 << 1))) * ((int32_t) params->dig_t2)) >> 11;
    var2 = (((((raw_temp >> 4) - ((int32_t) params->dig_t1)) * ((raw_temp >> 4) - ((int32_t) params->dig_t1))) >> 12) * ((int32_t) params->dig_t3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

int32_t bmp280_convert_pressure(int32_t raw_pressure, bmp280_calib_params_t *params) {
    int64_t var1, var2, p;
    var1 = ((int64_t) t_fine) - 128000;
    var2 = var1 * var1 * (int64_t) params->dig_p6;
    var2 = var2 + ((var1 * (int64_t) params->dig_p5) << 17);
    var2 = var2 + (((int64_t) params->dig_p4) << 35);
    var1 = ((var1 * var1 * (int64_t) params->dig_p3) >> 8) + ((var1 * (int64_t) params->dig_p2) << 12);
    var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) params->dig_p1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - raw_pressure;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t) params->dig_p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t) params->dig_p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t) params->dig_p7) << 4);
    return (uint32_t) p;
}
