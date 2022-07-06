# Timing and CAN Messages

Following the [rules](https://www.formulastudent.de/fileadmin/user_upload/all/2022/rules/FS-Rules_2022_v1.0.pdf) is mandatory for the __LV-BMS__ and, to accomplish the required tasks, the system need to stick to a specific timing for each measurement or message sending.<br>
For this reason a specific timer with more compare threshold has been chosen [(see TIM2 config)](IOC file and pinout.md).

For the 90% of the cases, when the timer is triggered to make a measure, it also send the relative payload via CAN. 

## Intervals
Here's the following periodic intervals

|                Interval name       |Interval (ms)
-------------------------------------|------------
|OPEN_WIRE_MEASURE                   |50
|VOLT_MEASURE                        |100
|CURRENT_AND_INVERTER_STATUS_MEASURE |500
|TEMPERATURE_MEASURE                 |100
|COOLING_STATUS                      |1000
|LV_VERSION                          |1000

## Flags setting
When the measurement module is initialized the timer threshold are set and the flags are cleared.

!!! help
    TIM2 is not the only timer which handles the measurements, in fact there's also TIM4 that it's dedicated to the __OPEN WIRE__ check

When an interrupts occurs a flag is set and the timer compare threshold is moved forward using for another period, in this way:  
    ```c
    // Set new threshold
    __HAL_TIM_SetCompare(
                    timer_handle,
                    timer_channel,
                    actual_counter + TIM_MS_TO_TICKS(timer_handle, TRIGGERED_MEASURE_INTERVAL));
    // Set the flag
    flags |= TRIGGERED_MEASURE_FLAG
    
    ```

To understand if an event has occured a flag is set

```c
    uint8_t flags;

    // Like a bitset
    enum {
    MEAS_OPEN_WIRE                              = 1,
    MEAS_VOLTS_AND_TEMPS_READ_FLAG              = 2,
    MEAS_COOLING_AND_LV_VERSION_READ_FLAG       = 4,
    MEAS_CURRENT_AND_INVERTERS_STATUS_READ_FLAG = 8
    };
```

## Flags check
During the __Running stage__ of the __LV-BMS__ every flags checked with this basic logic: 
```c 
if(flags & SELECTED_FLAG){
    //do stuffs
    ...
    //remove the flags
    flags &= ~SELECTED_FLAGS; 
}
```

## Example of a measure and can send
```c
if (flags & MEAS_VOLTS_AND_TEMPS_READ_FLAG) {
        // measure with the function sample and read
        if (volt_sample_and_read() != VOLT_ERR) {
            // if no errors are returned the data are sent via CAN
            can_primary_send(primary_ID_LV_VOLTAGE);
            can_primary_send(primary_ID_LV_TOTAL_VOLTAGE);
        }
        can_primary_send(primary_ID_LV_TEMPERATURE);
        flags &= ~MEAS_VOLTS_AND_TEMPS_READ_FLAG;
    }
```

