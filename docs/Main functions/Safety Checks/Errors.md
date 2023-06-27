# Errors

The Error Management is handled using [Squadra Corse PoliTo Error Management Library](https://github.com/squadracorsepolito/stmlibs/tree/master/error_utils)

Here's a list of all possible errors the **LV-BMSs** can handle. Every error it's reported with its specific **ID** and **TIMEOUT**.

| ID  | Error name                   | TIMEOUT |
| --- | ---------------------------- | ------- |
| 0   | ERROR_CELL_UNDERVOLTAGE      | 400     |
| 1   | ERROR_CELL_OVERVOLTAGE       | 400     |
| 2   | ERROR_OPEN_WIRE              | 400     |
| 3   | ERROR_CAN                    | 500     |
| 4   | ERROR_SPI                    | 500     |
| 5   | ERROR_OVER_CURRENT           | 400     |
| 6   | ERROR_CELL_UNDER_TEMPERATURE | 1000    |
| 7   | ERROR_CELL_OVER_TEMPERATURE  | 1000    |
| 8   | ERROR_RELAY                  | SOFT    |
| 9   | ERROR_BMS_MONITOR            | 500     |
| 10  | ERROR_VOLTAGES_NOT_READY     | 500     |
| 11  | ERROR_MCP23017               | 1000    |
| 12  | ERROR_RADIATOR               | SOFT    |
| 13  | ERROR_FAN                    | SOFT    |
| 14  | ERROR_PUMP                   | SOFT    |
| 15  | ERROR_ADC_INIT               | 1000    |
| 16  | ERROR_ADC_MUX                | 500     |

!!! note
    __SOFT__ means that the error won't be fatal in any case.

An error can be set with:

```c
error_set(ERROR_NAME, offset);
```

and unset with:

```c
error_reset(ERROR_NAME, offset);
```

If a set error reaches its **TIMEOUT** it become **fatal** and turn the LV-BMS into the _error state_.
