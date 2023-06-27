# Command Line Interface <!-- omit in toc -->

The command line interface of the **LV-BMS** is based on the _cli.h_ library and is used to communicate via **UART** peripheral.
**Parameters:** baudrate at 115200 bit/s

# Table of Contents <!-- omit in toc -->
- [Commands](#commands)
  - [volts](#volts)
  - [radiator](#radiator)
  - [pumps](#pumps)
  - [temps](#temps)
  - [adc](#adc)
  - [feedbacks](#feedbacks)
  - [dmesg](#dmesg)
  - [reset](#reset)
  - [can](#can)
  - [inv](#inv)
  - [cooling](#cooling)
  - [errors](#errors)
  - [health](#health)
  - [inject](#inject)


Pressing `?` followed by `Enter` or just `Enter` return the command list


!!! Commands

    ```c
    - volt
    - radiator
    - pumps
    - temps
    - adc
    - feedbacks
    - dmesg
    - reset
    - can
    - inv
    - cooling
    - errors
    - health
    - inject
    - ?
    ```

## Commands

### volts

Prints all cell voltages
<!-- omit in toc -->
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
    Cell 0: 3.751V M
    Cell 1: 3.921V
    Cell 2: 3.855V
    Cell 3: 3.890V
    Cell 4: 3.922V
    Cell 5: 3.803V
    Total voltage on board: 23.143V
    ```

### radiator

Used to set manually the duty cycle of a radiator, if cooling routine it's disabled, or just to see the status of the radiators and the internal fan inside the case.

#### Parameters: <!-- omit in toc -->

1. `L`/`R`/`B`/`info`: select whether to set Left, Right, Both radiator value or just obtain the info about them
2. `duty_cycle`/`off`: select a duty cycle from 0 to 1.0 (1.0 corresponds to 100%, and off to 0%)

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

#### Parameters: <!-- omit in toc -->

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

Returns the values of the cells tempratures
!!! example

    ```c
    Cell 0: 30.0000[°C]
    Cell 1: 30.0000[°C]
    Cell 2: 30.0000[°C]
    Cell 3: 30.0000[°C]
    Cell 4: 30.0000[°C]
    Cell 5: 30.0000[°C]
    Cell 6: 30.0000[°C]
    Cell 7: 30.0000[°C]
    Cell 8: 30.0000[°C]
    Cell 9: 30.0000[°C]
    Cell 10: 30.0000[°C]
    Cell 11: 30.0000[°C]
    ```

### adc
Returns the values of the three different ADCs of the BMS. 

#### Parameters: <!-- omit in toc -->
1. `hall/fb/ind`: 
   1. If **hall** is selected the current values of AS_Battery, LV_Battery and Charger and the respectively OCD (Over Current Detection) values are returned. Note that when OCD is detected the voltage should drop to ground.
   2. If **fb** is selected the values of the feedbacks are returned. 
   3. If **ind** is selected the values of the other signals that are directly connected to the third ADC.


!!! example
    ```c
    > adc ind
    As computer fb: 14.44 mV
    Relay out: 23103.30 mV
    Lvms out: 21.66 mV
    Batt out: 23161.06 mV
    
    > adc hall
    Current values
    OCD (Over Current Detection) and S (shunt current value)
    Hall ocd 0 (AS_Battery): 3300.00 mV
    Hall ocd 1 (LV_Battery): 3300.00 mV
    Hall ocd 2 (Charger): 3300.00 mV
    S hall 0 (AS_Battery): 2000.00 mA
    S hall 1 (LV_Battery): 2500.00 mA
    S hall 2 (Charger): 500.00 mA
    
    > adc fb
    SD_END: 0.80 mV
    BSPD_FB: 0.80 mV
    IMD_FB: 1.60 mV
    LVMS_FB: 1.60 mV
    RES_FB: 0.80 mV
    TSMS_FB: 0.80 mV
    LV_ENCL_1_FB: 0.80 mV
    LV_ENCL_2_FB: 0.80 mV
    HV_ENCL_1_FB: 0.80 mV
    HV_ENCL_2_FB: 0.80 mV
    BACK_PLATE_FB: 0.80 mV
    HVD_FB: 0.80 mV
    AMS_FB: 1.60 mV
    ASMS_FB: 1.60 mV
    INTERLOCK_IMD_FB: 0.80 mV
    SD_START: 1.60 mV
    ```


### feedbacks

Returns the values of the MCP23017 chip, which holds all of the feedbacks signals
!!! example

    ```c
    FB_INVERTERS: 1 [GPA0]
    FB_PCBS: 1 [GPA1]
    FB_PUMPS: 1 [GPA2]
    FB_SHUTDOWN: 1 [GPA3]
    FB_RADIATORS: 1 [GPA4]
    FB_FAN: 1 [GPA5]
    FB_AS_ACTUATION: 0 [GPA6]
    LED_R: 1 [GPB0]
    LED_G: 0 [GPB1]
    LED_B: 0 [GPB2]
    FRG_EN: 0 [GPB3]
    RFE_EN: 0 [GPB4]
    STP_ENABLE: 0 [GPB5]
    DISCHARGE: 0 [GPB6]
    STP_SLEEP: 0 [GPB7]
    ```

### dmesg

Use it if you want to show a debug message

### reset

Invokes `HAL_NVIC_SystemReset()` and reset the MCU

### can

Trigger some can messages, just for debug:

#### Parameters: <!-- omit in toc -->

1. `volts`/`cooling`/`total`/`current`/`temp`: to send the relative can msg

### inv

Optionally close or open the relays connected to the inverters and prints `RFE` and `FRG` values (relays values).
With no parameters it just print the status of the pins.

#### Parameters: <!-- omit in toc -->

1. `on`/`off`: close or open the inverter relays
   !!! example
   `c
[RFE]: 1
[FRG]: 1
`

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

### health

Returns the health signal bitset of the car. Health signals are signals used to detect **tricky** car's faults or **dangerous** scenarios

!!! Example

    ```c
    Health signals bitset

    Health code: 55

    LVMS_OUT: 1 (1 if V_LVMS - V_RELAY < 2000.000)

    RELAY_OUT: 1 (1 if V_RELAY - V_BATTERY < 2000.000)

    BATTERY_VOLTAGE_OUT: 1 (1 if > 19800.000)

    CHARGER_CURRENT_OUT: 0 (1 if > 50.000)

    BATTERY_CURRENT_OUT: 1 (1 if > 4000.000)

    SIGN_BATTERY_CURRENT_OUT: 1 (1 if positive)

    ```

### inject

Signal injection is a feature that allows to internally set some signals (like currents and temps) to trigger a specific fault or scenario. This feature should be used while the flag `NON_CRITICAL_SAFETY_CHECKS_BYPASS` is defined in `fenice-config.h`, otherwise every injected signal will be immediately overwritten by the real one sampled during the _measurement routine_.

#### Parameters: <!-- omit in toc -->

1. `current`/`temp`: to inject a current or a temp
2. `ìndex`: index of the signal in the array that will be stored
3. `value`: the value to inject

!!! example

    ```c
    inject temp 2 -50.0
    ```
