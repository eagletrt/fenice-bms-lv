
#include "ltc.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "fenice-config.h"
/*
 * default sample frequency -> 7kHz
 * 670 us needed to synchronize
 *
 */

ltc_struct ltc;

uint16_t get_ADCV_CC(uint8_t Ncell)
{
    if (Ncell > 6 || Ncell < 1)
        Ncell = 0;
    return (0x260 | (MD7K << 7) | Ncell);
}

void LTC_init(ltc_struct *_ltc, SPI_HandleTypeDef *hspi, uint8_t _address,
              GPIO_TypeDef *pinx, uint8_t pinn)
{
    _ltc->address = _address;
    _ltc->spi     = hspi;
    _ltc->CS_LTCx = pinx;
    _ltc->CS_LTCn = pinn;
}
bool ltc_set_REFON(ltc_struct *_ltc)
{
    bool     ret = true;
    uint16_t pec;
    uint8_t  cmd[4];
    uint16_t CC;
    uint16_t cmd_pec;
    uint8_t  data[12];
    uint8_t  payload[8];

    CC = WRCFG;

    cmd[0] = (uint8_t)0x80 |        // Address Command mode
             (_ltc->address << 3) | // LTC address
             (CC >> 8);             // Send 3 most significant bit of CC
    cmd[1]  = (uint8_t)(CC);
    cmd_pec = _pec15(2, cmd); // Calculate the PEC
    cmd[2]  = (uint8_t)(cmd_pec >> 8);
    cmd[3]  = (uint8_t)(cmd_pec);

    data[0]    = cmd[0];
    data[1]    = cmd[1];
    data[2]    = cmd[2];
    data[3]    = cmd[3];
    data[4]    = 0x04;
    payload[0] = data[4];
    data[5]    = 0x00;
    payload[1] = data[5];
    data[6]    = 0x00;
    payload[2] = data[6];
    data[7]    = 0x00;
    payload[3] = data[7];
    data[8]    = 0x00;
    payload[4] = data[8];
    data[9]    = 0x00;
    payload[5] = data[9];
    pec        = _pec15(6, payload);
    data[10]   = (uint8_t)(pec >> 8);
    data[11]   = (uint8_t)(pec);
    ltc6810_wakeup_idle(_ltc);
    spi_enable_cs(_ltc);
    if (HAL_SPI_Transmit(_ltc->spi, data, 12, 100) == HAL_OK)
    {
#ifndef NDEBUG_LTC
        sprintf(
            txt,
            "Trasmesso CMD: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n",
            data[0], data[1], data[2], data[3], data[4], data[5], data[6],
            data[7], data[8], data[9], data[10], data[11]);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 100);
#endif
    }
    else
    {
        ret = false;
    }
    spi_disable_cs(_ltc);

    return ret;
}
bool ltc_read_conf_reg(ltc_struct *_ltc)
{
    bool     ret = true;
    bool     pec;
    uint8_t  cmd[4];
    uint16_t CC;
    uint16_t cmd_pec;
    uint8_t  data[8];

    CC     = RDCFG;
    cmd[0] = (uint8_t)0x80 |        // Address Command mode
             (_ltc->address << 3) | // LTC address
             (CC >> 8);             // Send 3 most significant bit of CC
    cmd[1]  = (uint8_t)(CC);
    cmd_pec = _pec15(2, cmd); // Calculate the PEC
    cmd[2]  = (uint8_t)(cmd_pec >> 8);
    cmd[3]  = (uint8_t)(cmd_pec);
    ltc6810_wakeup_idle(_ltc);
    spi_enable_cs(_ltc);
    if (HAL_SPI_Transmit(_ltc->spi, cmd, 4, 100) == HAL_OK)
    {
#ifndef NDEBUG_LTC
        char txt[100];
#endif
#ifndef NDEBUG_LTC
        sprintf(txt, "Trasmesso RDCFG: %d, %d, %d, %d\r\n", cmd[0], cmd[1],
                cmd[2], cmd[3]);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 100);
#endif

        HAL_SPI_Receive(_ltc->spi, data, 8, 100);

#ifndef NDEBUG_LTC
        sprintf(txt, "Ricevuto: %d, %d, %d, %d, %d, %d, %d, %d\r\n", data[0],
                data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 100);
#endif
    }
    else
    {
#ifndef NDEBUG_LTC
        sprintf(txt, "Errore nella trasmissione\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif

        ret = false;
    }
    spi_disable_cs(_ltc);

    pec = (_pec15(6, data) == (uint16_t)(data[6] * 256 + data[7]));

    if (pec)
    {
    }
    else
    {
#ifndef NDEBUG_LTC
        sprintf(txt, "PEC sbagliata\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif

        ret = false;
    }
    return ret;
}
bool ltc_read_ID(ltc_struct *_ltc)
{
    bool     ret = true;
    bool     pec;
    uint8_t  cmd[4];
    uint16_t CC;
    uint16_t cmd_pec;
    uint8_t  data[8];

    CC     = RDSID;
    cmd[0] = (uint8_t)0x80 |        // Address Command mode
             (_ltc->address << 3) | // LTC address
             (CC >> 8);             // Send 3 most significant bit of CC
    cmd[1]  = (uint8_t)(CC);
    cmd_pec = _pec15(2, cmd); // Calculate the PEC
    cmd[2]  = (uint8_t)(cmd_pec >> 8);
    cmd[3]  = (uint8_t)(cmd_pec);
    ltc6810_wakeup_idle(_ltc);
    spi_enable_cs(_ltc);
    if (HAL_SPI_Transmit(_ltc->spi, cmd, 4, 100) == HAL_OK)
    {
#ifndef NDEBUG_LTC
        sprintf(txt, "Trasmesso RDSID: %d, %d, %d, %d\r\n", cmd[0], cmd[1],
                cmd[2], cmd[3]);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 100);
#endif

        HAL_SPI_Receive(_ltc->spi, data, 8, 100);
#ifndef NDEBUG_LTC
        sprintf(txt, "Ricevuto: %d, %d, %d, %d, %d, %d, %d, %d\r\n", data[0],
                data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 100);
#endif
    }
    else
    {
#ifndef NDEBUG_LTC
        sprintf(txt, "Errore nella trasmissione\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif

        ret = false;
    }
    spi_disable_cs(_ltc);

    pec = (_pec15(6, data) == (uint16_t)(data[6] * 256 + data[7]));

    if (pec)
    {
        _ltc->SERIAL_ID_L
            = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
        _ltc->SERIAL_ID_H = data[4] + (data[5] << 8);
    }
    else
    {
#ifndef NDEBUG_LTC
        sprintf(txt, "PEC sbagliata\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif

        ret = false;
    }
    return ret;
}
bool ltc_read_STATUS(ltc_struct *_ltc)
{
    bool     ret = true;
    bool     pec;
    uint8_t  cmd[4];
    uint16_t CC;
    uint16_t cmd_pec;
    uint8_t  data[8];

    CC     = ADSTAT;
    cmd[0] = (uint8_t)0x80 |        // Address Command mode
             (_ltc->address << 3) | // LTC address
             (CC >> 8);             // Send 3 most significant bit of CC
    cmd[1]  = (uint8_t)(CC);
    cmd_pec = _pec15(2, cmd); // Calculate the PEC
    cmd[2]  = (uint8_t)(cmd_pec >> 8);
    cmd[3]  = (uint8_t)(cmd_pec);

    ltc6810_wakeup_idle(_ltc);
    spi_enable_cs(_ltc);
    if (HAL_SPI_Transmit(_ltc->spi, cmd, 4, 100) == HAL_OK)
    {
#ifndef NDEBUG_LTC
        sprintf(txt, "Trasmesso ADSTAT_TMP: %d, %d, %d, %d\r\n", cmd[0], cmd[1],
                cmd[2], cmd[3]);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif
    }
    else
    {
#ifndef NDEBUG_LTC
        sprintf(txt, "Errore nella trasmissione\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif

        ret = false;
    }
    spi_disable_cs(_ltc);

    HAL_Delay(100);

    CC     = RDSTATA;
    cmd[0] = (uint8_t)0x80 |        // Address Command mode
             (_ltc->address << 3) | // LTC address
             (CC >> 8);             // Send 3 most significant bit of CC
    cmd[1] = (uint8_t)(CC);         // Send the other 8 bit of CC

    cmd_pec = _pec15(2, cmd); // Calculate the PEC
    cmd[2]  = (uint8_t)(cmd_pec >> 8);
    cmd[3]  = (uint8_t)(cmd_pec);

    ltc6810_wakeup_idle(_ltc);
    spi_enable_cs(_ltc);
    if (HAL_SPI_Transmit(_ltc->spi, cmd, 4, 100) == HAL_OK)
    {
#ifndef NDEBUG_LTC
        sprintf(txt, "Trasmesso RDSTATA: %d, %d, %d, %d\r\n", cmd[0], cmd[1],
                cmd[2], cmd[3]);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif

        HAL_SPI_Receive(_ltc->spi, data, 8, 100);
#ifndef NDEBUG_LTC
        sprintf(txt, "Ricevuto: %d, %d, %d, %d, %d, %d, %d, %d\r\n", data[0],
                data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif
    }
    else
    {
#ifndef NDEBUG_LTC
        sprintf(txt, "Errore nella trasmissione\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif

        ret = false;
    }
    HAL_Delay(10);
    spi_disable_cs(_ltc);

    pec = (_pec15(6, data) == (uint16_t)(data[6] * 256 + data[7]));

    if (pec)
    {
        _ltc->STATUS_SC   = data[0] * 256 + data[1];
        _ltc->STATUS_ITMP = data[2] * 256 + data[3];
        _ltc->STATUS_VA   = data[4] * 256 + data[5];
    }
    else
    {
#ifndef NDEBUG_LTC
        sprintf(txt, "PEC sbagliata\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif

        ret = false;
    }
    return ret;
}

/**
 * @brief		This function is used to read volteges.
 *
 * @param		_ltc	LTC struct
 * @retval		True if there aren't errors
 */
bool read_voltages(ltc_struct *_ltc)
{
    bool     ret = true;
    bool     pec;
    uint8_t  cmd[4];
    uint16_t CC;
    uint16_t cmd_pec;
    uint8_t  data[8];

    //--- Send start ADC conversion ---//
    CC     = get_ADCV_CC(ALL_CELLS);
    cmd[0] = (uint8_t)(0x80 |                 // Address Command mode
                       (_ltc->address << 3) | // LTC address
                       (CC >> 8)); // Send 3 most significant bit of CC
    cmd[1] = (uint8_t)(CC);        // Send the other 8 bit of CC

    cmd_pec = _pec15(2, cmd); // Calculate the PEC
    cmd[2]  = (uint8_t)(cmd_pec >> 8);
    cmd[3]  = (uint8_t)(cmd_pec);

    ltc6810_wakeup_idle(_ltc);
    spi_enable_cs(_ltc);
    if (HAL_SPI_Transmit(_ltc->spi, cmd, 4, 100) != HAL_OK)
    {
        ret = false;
#ifndef NDEBUG_LTC
        sprintf(txt, "Errore nella trasmissione\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif
    }
    HAL_Delay(5);
    spi_disable_cs(_ltc);
    HAL_Delay(10); // TODO: change waiting mode in interrupt -> it's only to
                   // test (ADC takes 1.2ms)
#ifndef NDEBUG_LTC
    sprintf(txt, "Trasmesso: %d, %d, %d, %d\r\n", cmd[0], cmd[1], cmd[2],
            cmd[3]);
    HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif

    //--- READ GROUP A VOLTAGES ---//
    CC     = RDCVA;                           // Read voltages from group A
    cmd[0] = (uint8_t)(0x80 |                 // Address Command mode
                       (_ltc->address << 3) | // LTC address
                       (CC >> 8)); // Send 3 most significant bit of CC
    cmd[1] = (uint8_t)(CC);        // Send the other 8 bit of CC

    cmd_pec = _pec15(2, cmd); // Calculate the PEC
    cmd[2]  = (uint8_t)(cmd_pec >> 8);
    cmd[3]  = (uint8_t)(cmd_pec);

    ltc6810_wakeup_idle(_ltc);
    spi_enable_cs(_ltc);
    if (HAL_SPI_Transmit(_ltc->spi, cmd, 4, 100) == HAL_OK)
    {
        HAL_SPI_Receive(_ltc->spi, data, 8, 100);
#ifndef NDEBUG_LTC
        sprintf(txt, "Trasmesso: %d, %d, %d, %d\r\n", cmd[0], cmd[1], cmd[2],
                cmd[3]);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif

#ifndef NDEBUG_LTC
        sprintf(txt, "Ricevuto: %d, %d, %d, %d, %d, %d, %d, %d\r\n", data[0],
                data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif
    }
    else
    {
        ret = false;
#ifndef NDEBUG_LTC
        sprintf(txt, "Errore nella trasmissione\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif
    }
    spi_disable_cs(_ltc);

    pec = (_pec15(6, data) == (uint16_t)(data[6] * 256 + data[7]));

    if (pec)
    {
        uint8_t cell = 0; // Counts the cell inside the register
        for (cell = 0; cell < 3; cell++)
        {
            _ltc->voltage[cell] = _convert_voltage(&data[2 * cell]);
        }
    }
    else
    {
        ret = false;
#ifndef NDEBUG_LTC
        sprintf(txt, "PEC sbagliata\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif
    }

    //--- READ GROUP B VOLTAGES ---//
    CC     = RDCVB;                 // Read voltages from group A
    cmd[0] = (uint8_t)0x80 |        // Address Command mode
             (_ltc->address << 3) | // LTC address
             (CC >> 8);             // Send 3 most significant bit of CC
    cmd[1] = (uint8_t)(CC);         // Send the other 8 bit of CC

    cmd_pec = _pec15(2, cmd); // Calculate the PEC
    cmd[2]  = (uint8_t)(cmd_pec >> 8);
    cmd[3]  = (uint8_t)(cmd_pec);

    ltc6810_wakeup_idle(_ltc);
    spi_enable_cs(_ltc);
    if (HAL_SPI_Transmit(_ltc->spi, cmd, 4, 100) == HAL_OK)
    {
        HAL_SPI_Receive(_ltc->spi, data, 8, 100);
        spi_disable_cs(_ltc);
#ifndef NDEBUG_LTC
        sprintf(txt, "Trasmesso: %d, %d, %d, %d\r\n", cmd[0], cmd[1], cmd[2],
                cmd[3]);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif

#ifndef NDEBUG_LTC
        sprintf(txt, "Ricevuto: %d, %d, %d, %d, %d, %d, %d, %d\r\n", data[0],
                data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif
    }
    else
    {
        ret = false;
#ifndef NDEBUG_LTC
        sprintf(txt, "Errore nella trasmissione\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif
    }
    spi_disable_cs(_ltc);

    pec = (_pec15(6, data) == (uint16_t)(data[6] * 256 + data[7]));

    if (pec)
    {
        uint8_t cell = 0; // Counts the cell inside the register
        for (cell = 0; cell < 3; cell++)
        {
            _ltc->voltage[cell + 3] = _convert_voltage(&data[2 * cell]);
        }
    }
    else
    {
        ret = false;
#ifndef NDEBUG_LTC
        sprintf(txt, "PEC sbagliata\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
#endif
    }
    return ret;
}

/**
 * @brief	This function is used to convert the 2 byte raw data from the
 * 				LTC68xx to a 16 bit unsigned integer
 *
 * @param 	v_data	Raw data bytes
 *
 * @retval	Voltage [mV]
 */
uint16_t _convert_voltage(uint8_t v_data[])
{
    return v_data[0] + (v_data[1] << 8);
}

void ltc6810_wakeup_idle(ltc_struct *_ltc)
{
    uint8_t data[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    spi_enable_cs(_ltc);
    HAL_SPI_Transmit(_ltc->spi, data, 6, 10);
    spi_disable_cs(_ltc);
}
void spi_enable_cs(ltc_struct *_ltc)
{
    HAL_GPIO_WritePin(_ltc->CS_LTCx, _ltc->CS_LTCn, GPIO_PIN_RESET);
    while ((_ltc->spi->State != HAL_SPI_STATE_READY)
           && (_ltc->spi->State != HAL_SPI_STATE_ERROR))
    {
    }
}

void spi_disable_cs(ltc_struct *_ltc)
{
    while ((_ltc->spi->State != HAL_SPI_STATE_READY)
           && (_ltc->spi->State != HAL_SPI_STATE_ERROR))
    {
    }
    HAL_GPIO_WritePin(_ltc->CS_LTCx, _ltc->CS_LTCn, GPIO_PIN_SET);
}

uint16_t _pec15(uint8_t len, uint8_t data[])
{
    uint16_t remainder, address;
    remainder = 16; // PEC seed
    for (int i = 0; i < len; i++)
    {
        // calculate PEC table address
        address   = ((remainder >> 7) ^ data[i]) & 0xff;
        remainder = (remainder << 8) ^ crcTable[address];
    }
    // The CRC15 has a 0 in the LSB so the final value must be multiplied by
    // 2
    return (remainder * 2);
}

uint16_t get_min_voltage(ltc_struct *_ltc)
{
    uint16_t min = UINT16_MAX;
    for (int i = 0; i < LTC6810_N_CELL; i++)
    {
        if (_ltc->voltage[i] < min)
            min = _ltc->voltage[i];
    }
    return 15;
}

uint16_t get_mean_voltage(ltc_struct *_ltc)
{
    uint64_t mean = 0;

    for (int i = 0; i < LTC6810_N_CELL; i++)
    {
        mean += _ltc->voltage[i];
    }
    return (mean / LTC6810_N_CELL);
}
