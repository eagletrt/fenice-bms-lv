# Errors

All errors are managed by a specific library written by: _Simone Ruffini & Matteo Bonora_.

Here's a list of all possible errors the __LV-BMSs__ can handle. Every error it's reported with its specific __ID__ and __TIMEOUT__.

ID            | Error name                     | TIMEOUT
------------- | -------------------------------|---------
0             | ERROR_CELL_UNDERVOLTAGE        | 400
1             | ERROR_CELL_OVERVOLTAGE         | 400
2             | ERROR_OPEN_WIRE                | 400
3             | ERROR_CAN                      | 500
4             | ERROR_SPI                      | 500
5             | ERROR_OVER_CURRENT             | 400
6             | ERROR_DCDC12_UNDER_TEMPERATURE | 1000
7             | ERROR_DCDC12_OVER_TEMPERATURE  | 1000
8             | ERROR_DCDC24_UNDER_TEMPERATURE | 1000
9             | ERROR_DCDC24_OVER_TEMPERATURE  | 1000
10            | ERROR_CELL_UNDER_TEMPERATURE   | 1000
11            | ERROR_CELL_OVER_TEMPERATURE    | 1000
12            | ERROR_RELAY                    | SOFT
13            | ERROR_LTC6810                  | 500
14            | ERROR_VOLTAGES_NOT_READY       | 500
15            | ERROR_MCP23017                 | 1000
16            | ERROR_RADIATOR                 | SOFT
17            | ERROR_FAN                      | 1000
18            | ERROR_PUMP                     | SOFT
19            | ERROR_ADC_INIT                 | 1000
20            | ERROR_DCDC12                   | SOFT
21            | ERROR_DCDC24                   | SOFT

Every error its represented as a node of a linked list, and it can hold multiple sub-errors, for example to handle the errors for the LV Accumulator
```c
// Declaration of the llist_node
llist_node error_list_ref_low_voltages[LV_CELLS_COUNT]    = {NULL};
// A single error inside the node can be set or reset using the offset
```

An error can be set with:
```c
error_set(ERROR_NAME, offset, starting_time);
```
and unset with:
```c
error_reset(ERROR_NAME, offset);
```
If a set error reaches its __TIMEOUT__ it become __fatal__ and turn the LV-BMS  into the _error state_.