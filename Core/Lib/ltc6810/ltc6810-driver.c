/**
 ********************************************************************************
 * @file    LTC6810 DRIVER
 * @author  Tommaso Canova
 * @date    24-02-2022
 * @brief   
 ********************************************************************************
 */

#include "ltc6810-driver.h"

uint8_t dummy_data[6] = {0};
uint8_t serial_id[6]  = {0};
char buf[128];

/**
 * @brief Pre computed table, used in the pec15 function
 * 
 */
const uint16_t crcTable[256] = {
    0x0,    0xc599, 0xceab, 0xb32,  0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac, 0xff35, 0x2cc8, 0xe951,
    0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1, 0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2,
    0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e, 0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0,
    0x6182, 0xa41b, 0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd, 0x2544,
    0x2be,  0xc727, 0xcc15, 0x98c,  0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c, 0x3d6e, 0xf8f7, 0x2b0a, 0xee93,
    0xe5a1, 0x2038, 0x7c2,  0xc25b, 0xc969, 0xcf0,  0xdf0d, 0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560,
    0x869d, 0x4304, 0x4836, 0x8daf, 0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72,
    0x6640, 0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba, 0x4a88, 0x8f11,
    0x57c,  0xc0e5, 0xcbd7, 0xe4e,  0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b, 0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d,
    0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921, 0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26,
    0x8edb, 0x4b42, 0x4070, 0x85e9, 0xf84,  0xca1d, 0xc12f, 0x4b6,  0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a,
    0x3528, 0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59, 0x2ac0,
    0xd3a,  0xc8a3, 0xc391, 0x608,  0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01, 0x5f98, 0x8c65, 0x49fc,
    0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9, 0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4,
    0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a, 0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8,
    0x32ea, 0xf773, 0x248e, 0xe117, 0xea25, 0x2fbc, 0x846,  0xcddf, 0xc6ed, 0x374,  0xd089, 0x1510, 0x1e22, 0xdbbb,
    0xaf8,  0xcf61, 0xc453, 0x1ca,  0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9,
    0xe89b, 0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3, 0x585a,
    0x8ba7, 0x4e3e, 0x450c, 0x8095};

/**
 * @brief Generate pec0 and pec1, needed in the protocol message after cmd0 and cm1
 * 
 * @param data 
 * @param len 
 * @return uint16_t 
 */
uint16_t ltc6810_pec15(uint8_t data[], uint8_t len) {
    uint16_t remainder, address;
    remainder = 16;  //PEC seed
    for (int i = 0; i < len; i++) {
        address   = ((remainder >> 7) ^ data[i]) & 0xff;   //calculate PEC table address
        remainder = (remainder << 8) ^ crcTable[address];  //pectable before
    }
    return (remainder * 2);  //The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
}

/**
 * @brief Set CS low to start the communication
 * 
 * @param spi 
 */
void ltc6810_enable_cs(SPI_HandleTypeDef *spi) {
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    while (spi->State != HAL_SPI_STATE_READY)
        ;
}
/**
 * @brief Set CS high to end the communication
 * 
 * @param spi 
 */
void ltc6810_disable_cs(SPI_HandleTypeDef *spi) {
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
    while (spi->State != HAL_SPI_STATE_READY)
        ;
}

/**
 * @brief Wakes up ltc6810 sending 0xFF, used before every SPI_Transmit
 * 
 * @param spi 
 */
void ltc6810_wakeup_idle(SPI_HandleTypeDef *spi) {
    uint8_t data = 0xFF;
    ltc6810_enable_cs(spi);
    HAL_SPI_Transmit(spi, &data, 1, 1);
    ltc6810_disable_cs(spi);
}

/**
 * @brief Start Cell Voltage ADC Conversion and Poll Status 
 *        with 7khz sampling mode, DCP disable and all cells selected 
 * @param spi 
 */
void ltc6810_basic_adcv(SPI_HandleTypeDef *spi) {
    uint8_t cmd[4];
    uint16_t cmd_pec;
    // ADCV message: 0 1 MD[1] MD[0] 1 1 DCP 0 CH[2] CH[1] CH[0]
    // example of broadcast message:
    // CMD0 = 0 0 0 0 0 0 1 MD[1]
    // CMD1 = MD[2] 1 1 DCP 0 CH[2] CH[1] CH[0]
    //cmd[0] = (uint8_t) 0b00000011; //0b;//(uint8_t)0b00000010 | (MD >> 1);
    cmd[0]  = (uint8_t)0b00000011;
    cmd[1]  = (uint8_t)0b01100000;
    cmd_pec = ltc6810_pec15(cmd, 2);    // returns 16bit pec code
    cmd[2]  = (uint8_t)(cmd_pec >> 8);  // extracts first pec code
    cmd[3]  = (uint8_t)(cmd_pec);       // extracts second pec code

    ltc6810_wakeup_idle(spi);
    ltc6810_enable_cs(spi);
    HAL_SPI_Transmit(spi, cmd, 4, 100);  // sends 32bits cmd
    ltc6810_disable_cs(spi);
}

/**
 * @brief Start Cell Voltage ADC Conversion and Poll Status 
 *        with custom sampling mode, DCP and cells selection 
 * 
 * @param spi 
 * @param MD  ADC sampling mode    [2 bit]
 * @param DCP Discharge permitted  [1 bit] 
 * @param CH  Cell selection       [3 bit]
 */
void ltc6810_adcv(SPI_HandleTypeDef *spi, uint8_t MD, uint8_t DCP, uint8_t CH) {
    //TODO: not tested yet
    uint8_t cmd[4];
    uint16_t cmd_pec;
    // ADCV message: 0 1 MD[1] MD[0] 1 1 DCP 0 CH[2] CH[1] CH[0]
    // example of broadcast message:
    // CMD0 = 0 0 0 0 0 0 1 MD[1]
    // CMD1 = MD[2] 1 1 DCP 0 CH[2] CH[1] CH[0]
    cmd[0] = (uint8_t)0b00000010 | (MD >> 1);
    ;
    cmd[1]  = (uint8_t)(uint8_t)0b01100000 | (MD << 7) | (DCP << 3) | CH;
    cmd_pec = ltc6810_pec15(cmd, 2);    // returns 16bit pec code
    cmd[2]  = (uint8_t)(cmd_pec >> 8);  // extracts first pec code
    cmd[3]  = (uint8_t)(cmd_pec);       // extracts second pec code

    ltc6810_wakeup_idle(spi);
    ltc6810_enable_cs(spi);
    HAL_SPI_Transmit(spi, cmd, 4, 100);  // sends 32bits cmd
    ltc6810_disable_cs(spi);
}

/**
 * @brief Write Configuration Register Group
 * 
 * @param spi 
 * @param cfgr Register
 */
void ltc6810_wrcfg(SPI_HandleTypeDef *spi, uint8_t cfgr[8]) {
    // Wrfg message: 0000000001
    uint8_t cmd[4];
    cmd[0]           = 0;
    cmd[1]           = 1;
    uint16_t cmd_pec = ltc6810_pec15(cmd, 2);    // returns 16bit pec code
    cmd[2]           = (uint8_t)(cmd_pec >> 8);  // extracts first pec code
    cmd[3]           = (uint8_t)(cmd_pec);       // extracts second pec code

    ltc6810_wakeup_idle(spi);
    ltc6810_enable_cs(spi);
    HAL_SPI_Transmit(spi, cmd, 4, 100);
    HAL_SPI_Transmit(spi, cfgr, 8, 100);
    ltc6810_disable_cs(spi);
}

/**
 * @brief Read ltc6810 serial id and saves it into serial_id variable
 * 
 * @param spi 
 */
void ltc6810_read_serial_id(SPI_HandleTypeDef *spi) {
    uint8_t cmd[4];
    cmd[0]           = 0b10000000;
    cmd[1]           = 0b00101100;
    uint16_t cmd_pec = ltc6810_pec15(cmd, 2);
    cmd[2]           = (uint8_t)(cmd_pec >> 8);  // extracts first pec code
    cmd[3]           = (uint8_t)(cmd_pec);       // extracts second pec code

    ltc6810_wakeup_idle(spi);

    ltc6810_enable_cs(spi);
    HAL_SPI_Transmit(spi, cmd, 4, 100);
    //HAL_SPI_Receive(spi, rx_buff, 6, 100);
    HAL_SPI_TransmitReceive(spi, dummy_data, serial_id, 6, 100);
    ltc6810_disable_cs(spi);
}

/**
 * @brief Read cell voltages from the chip then convert them in some
 *        readable values and it stores in the volts struct
 * 
 * @param spi 
 * @param volts voltages struct
 * @return uint8_t 
 */
uint8_t ltc6810_read_voltages(SPI_HandleTypeDef *spi, voltage_t *volts) {
    uint8_t ltc_error = 0;
    uint8_t cmd[4];
    uint16_t cmd_pec;
    uint8_t rx_data[8];

    cmd[0] = 0;

    if (ltc6810_pladc(spi, 10) == HAL_TIMEOUT) {
        ltc_error = 1;
    }

    for (uint8_t reg = 0; reg < LTC6810_REG_COUNT; reg++) {
        // Create rdcv message for each register
        cmd[1]  = (uint8_t)rdcv_cmds[reg];
        cmd_pec = ltc6810_pec15(cmd, 2);
        cmd[2]  = (uint8_t)(cmd_pec >> 8);
        cmd[3]  = (uint8_t)(cmd_pec);

        ltc6810_wakeup_idle(spi);
        ltc6810_enable_cs(spi);

        //Forces SPI Transmit
        do {
        } while (HAL_SPI_Transmit(spi, cmd, 4, 100) != HAL_OK);

        /* Each register can hold LTC6810_REG_CELL_COUNT, every value it's 2 byte long.
           2 more bytes are added to attach pec to the rx_data */
        HAL_SPI_TransmitReceive(spi, dummy_data, rx_data, LTC6810_REG_CELL_COUNT * 2 + 2, 100);
        ltc6810_disable_cs(spi);

#if LTC6810_EMU == 1
        // Writes 3.6v to each cell
        uint8_t emu_i;
        for (emu_i = 0; emu_i < LTC6810_REG_CELL_COUNT * 2; emu_i++) {
            // 36000
            rx_data[emu_i]   = 0b10100000;
            rx_data[++emu_i] = 0b10001100;
        }
        uint16_t emu_pec = ltc6810_pec15(rx_data, 6);
        rx_data[6]       = (uint8_t)(emu_pec >> 8);
        rx_data[7]       = (uint8_t)emu_pec;
#endif

#ifdef VOLTAGE_CONVERSION
        uint16_t rx_pec = (((uint16_t)rx_data[6]) << 8) | ((uint8_t)rx_data[7]);
        //if(ltc6810_pec15(rx_data,6) == (uint16_t)(rx_data[6] * 256 + rx_data[7])){
        if (ltc6810_pec15(rx_data, 6) == rx_pec) {
            ltc_error = 0;
            // For every cell in the register
            for (uint8_t cell = 0; cell < LTC6810_REG_CELL_COUNT; cell++) {
                uint16_t index = (reg * LTC6810_REG_CELL_COUNT) + cell;
                volts[index]   = ltc6810_convert_voltages(&rx_data[sizeof(voltage_t) * cell]);
            }
        }
#endif
    }
    return ltc_error;
}

/**
 * @brief Convertion function
 * 
 * @param v_data 
 * @return uint16_t 
 */
uint16_t ltc6810_convert_voltages(uint8_t v_data[]) {
    return *((uint16_t *)v_data);
}

/**
 * @brief Read both status registers
 * 
 * @param spi 
 */
void ltc6810_read_both_status_register(SPI_HandleTypeDef *spi) {
    uint8_t cmd[4];
    uint16_t cmd_pec;
    uint8_t rx_data[8];

    cmd[0] = 0;

    for (uint8_t reg = 0; reg < LTC6810_REG_COUNT; reg++) {
        cmd[1]  = (uint8_t)rdstat_cmds[reg];
        cmd_pec = ltc6810_pec15(cmd, 2);
        cmd[2]  = (uint8_t)(cmd_pec >> 8);
        cmd[3]  = (uint8_t)(cmd_pec);

        ltc6810_wakeup_idle(spi);
        ltc6810_enable_cs(spi);
        if (HAL_SPI_Transmit(spi, cmd, 4, 10) != HAL_OK) {
        }
        while (spi->State != HAL_SPI_STATE_READY)
            ;
        ltc6810_disable_cs(spi);
        /* Each register can hold LTC6810_REG_CELL_COUNT, every value it's 2 byte long.
        2 more bytes are added to attach pec to the rx_data */
        HAL_SPI_TransmitReceive(spi, dummy_data, rx_data, LTC6810_REG_CELL_COUNT * 2 + 2, 100);
        ltc6810_disable_cs(spi);
    }
}

/**
 * @brief Return serial id as a readable string
 * 
 * @return char* 
 */
char *ltc6810_return_serial_id() {
    ltc6810_read_serial_id(&SPI);
    memset(buf, 0, sizeof(buf));
    for (uint8_t i = 0; i < 6; i++) {
        sprintf(buf + strlen(buf), "%x", serial_id[i]);
    }
    //sprintf(buf+strlen(buf), "\r\n");
    return buf;
}

/**
 * @brief Poll ADC Conversion Status 
 * 
 * @param spi 
 * @param timeout 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef ltc6810_pladc(SPI_HandleTypeDef *spi, uint32_t timeout) {
    // PLADC COMMAND 11100010100
    uint8_t cmd[4];
    uint16_t cmd_pec;
    uint8_t rx_data;
    uint32_t tick;

    cmd[0]  = 0b00000111;
    cmd[1]  = 0b00010100;
    cmd_pec = ltc6810_pec15(cmd, 2);
    cmd[2]  = (uint8_t)(cmd_pec >> 8);
    cmd[3]  = (uint8_t)(cmd_pec);

    ltc6810_wakeup_idle(spi);
    ltc6810_enable_cs(spi);
    HAL_SPI_Transmit(spi, cmd, 4, 10);
    tick = HAL_GetTick();
    // Wait until the SDO line is pulled up high
    // When SDO is low the ADC is still converting
    // When SDO is pulled high the ADC has finished the conversion
    do {
        HAL_SPI_TransmitReceive(spi, dummy_data, &rx_data, 1, 1);
    } while (rx_data == 0x0 && (HAL_GetTick() - tick < timeout));
    ltc6810_disable_cs(spi);

    return rx_data ? HAL_OK : HAL_TIMEOUT;
}

void ltc6810_build_dcc(bms_balancing_cells cells, uint8_t cfgr[8]) {
    for (uint8_t i = 0; i < LV_CELLS_COUNT; i++) {
        if (((cells & (1 << i)) >> i) == 1) {  //extracts single bit and checks if it's 1
            cfgr[4] |= (1 << i);
        }
    }
    uint16_t pec = ltc6810_pec15(cfgr, 6);
    cfgr[2]      = (uint8_t)(pec >> 8);
    cfgr[3]      = (uint8_t)(pec);
}

void ltc6810_set_balancing(SPI_HandleTypeDef *spi, bms_balancing_cells cells, int dcto) {
    uint8_t cfgr[8] = {0};  // configuration register

    cfgr[0] |= 0b10;       // set DTEN ON
    cfgr[5] |= dcto << 4;  //Set timer value
    ltc6810_build_dcc(cells, cfgr);
    ltc6810_wakeup_idle(spi);
    ltc6810_wrcfg(spi, cfgr);
}