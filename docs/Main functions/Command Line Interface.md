# Command Line Interface

The command line interface of the __LV-BMS__ is based on the _cli.h_ library and is used to communicate via __UART__ peripheral.

- __Parameters:__ baudrate at 115200 bit/s 

Pressing `?` followed by `Enter` or just `Enter` return the command list 

!!! Command list
    ```c
    - volts
    - radiator
    - pumps
    - temps
    - feedbacks
    - dmesg
    - reset
    - can
    - inv
    - cooling
    - errors
    - ?
    ```


## Commands

### volts
Prints all cell voltages

#### Parameters:

1. `status`: append to the output also voltage status

Possible outputs of voltage status:

- VOLT OK
- VOLT UNDER VOLTAGE
- VOLT OVER VOLTAGE
- VOLT ERROR

Typical output without `status`parameter
!!! example
    ```c
        Cell 1: 4.142V
        Cell 2: 4.159V
        Cell 3: 4.159V
        Total voltage on board: 12.460V
    ```
### radiator
Used to set manually the duty cycle of a radiator, if cooling routine it's disabled, or just to see the status of the radiators and the internal fan inside the case.

#### Parameters:

1. `L`/`R`/`B`/`info`: select whether to set Left, Right, Both radiator value or just obtain the info about them
2. `duty_cycle`/`off`: select a duty cycle from 0 to 1.0 (1.0  corresponds to 100%, and off to 0%)

Typical output with `info` parameter
!!! example
    ```c 
    Radiators status:
            Left Duty Cycle: 0.35
            Right Duty Cycle: 0.35
            Is right on: 1
            Is left on: 1
    Automatic mode:true
    Internal fan dt: 12.32%
    ```
!!! note
    Automatic mode tells to the user if the radiators and the internal fan values are set by the inverter temperatures, so they are spinning under the BMS control, or it they are piloted by the steering wheel

### pumps
Used to set manually the pumps power, if cooling routine it's disabled, or just to see the status of the pumps.

#### Parameters:
1. `info`/`p`/`v`: choose whether to see the status of the pumps or set a specific value 
!!! note
    `p` stands for proportional and it would be followed by a value from 0 to 1.0, in fact the value is multiplied with `MAX_OPAMP_OUT` <br>
    `v` stands for volt and it would be followed by a value from 0 to `MAX_OPAMP_OUT`, in fact the value represents the analog voltage level at the _opamp_ output

2. `value`: proportional or analog value choosen if the first parameter choosen is `p` or `v`

Typical output with `info` parameter
!!! example
    ```c 
    Pumps status
        Pumps L:1.00 [4.95V] R:1.00 [4.95V]
    Automatic mode:false
    ```
!!! note
    When displayed, the first value of the pumps represents the proportionality with the output (1.0 = _Max power_), and the second one enclosed in the square brackets is the analog value.

!!! note
    Automatic mode tells to the user if the pumps value are set by the inverter temperatures, so they are under the BMS control, or it they are piloted by the steering wheel

### temps
Returns the values of the various ADC sensors
!!! example
    ```c 
    ADC sensors:
        Current: 2100.950623 [mA]
        Batt1: 21.113552 [°C]
        Batt2: 20.952381 [°C]
        DCDC 12V: 21.032967[°C]
        DCDC 24V: 20.952381[°C]
    ```

### feedbacks
Returns the values of the MCP23017 chip, which holds all of the feedbacks signals
!!! example
    ```c 
    FB_RELAY: 0 [GPA0]
    FB_INVERTERS: 0 [GPA1]
    GPIOA VAL: 0 [GPA2] // useless feedback, held for consistency
    FB_24: 0 [GPA3]
    FB_PUMPS: 0 [GPA4]                      
    FB_SHUTDOWN: 0 [GPA5]                   
    FB_RADIATORS: 0 [GPA6] 
    FB_FAN: 0 [GPA7]                                       
    FB_MAIN: 1 [GPB0]
    FB_PCBS: 0 [GPB1]
    FB_12: 0 [GPB2]
    ```
### dmesg
Use it if you want to show a debug message
### reset
Invokes `HAL_NVIC_SystemReset()` and reset the MCU
### can
Trigger some can messages, just for debug:
#### Parameters:
1. `volts`/`cooling`/`total`/`current`/`temp`: to send the relative can msg
### inv
Optionally close or open the relays connected to the inverters and prints `RFE` and `FRG` values (relays values).
With no parameters it just print the status of the pins.
#### Parameters:
1. `on`/`off`: close or open the inverter relays
!!! example
    ```c 
    [RFE]: 1
    [FRG]: 1
    ```
### cooling
Returns the cooling info (fancy way to do `radiator info` + `pumps info`)
!!! example
    ```c
    Radiators status:
         Left Duty Cycle: 0.35
         Right Duty Cycle: 0.35
         Is right on: 1
         Is left on: 1
        Radiator automatic mode:true
    Internal fan dt: 12.51%
    Pumps status
    Pumps L:1.00 [4.95V] R:1.00 [4.95V]
    Pumps automatic mode:false  
    ```
### errors
Returns the error count and, for every set error, the relative id with 

!!! example when LVMS is OFF
    ```c
    total 5

    id..........20 (dcdc12 off)
    timestamp...T+2286 (169801ms ago)
    offset......0
    state.......warning
    timeout delta 2147313845

    id..........16 (radiator)
    timestamp...T+2298 (169789ms ago)
    offset......1
    state.......warning
    timeout delta 2147313857

    id..........17 (fan)
    timestamp...T+2298 (169789ms ago)
    offset......0
    state.......warning
    timeout delta 2147313857

    id..........21 (dcdc24 off)
    timestamp...T+2298 (169789ms ago)
    offset......0
    state.......warning
    timeout delta 2147313857

    id..........12 (open relay)
    timestamp...T+2299 (169788ms ago)
    offset......0
    state.......warning
    timeout delta 2147313858
    ```