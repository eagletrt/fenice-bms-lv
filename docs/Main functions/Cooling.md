# Cooling

One of the main purpouse of the __LV-BMS__ is to control the cooling circuit of the car (composed of radiators and pumps).
The operation is performed by changing the duty cycle of the radiators and the pumps power based on the average mean of the two inverter temperatures.
The fan inside the case has to cool down the DCDCs, so its speed follows the average mean of them temperatures.

## Cooling routine
It's a function wich takes as input the average mean of the two inverters temperature and set the speed of radiators and the power of the pumps based on a simple curve: $y=mx+q$

## Radiators
The m factor as follows: 

$m = \frac{(MAX\_RADIATOR\_DT-MIN\_RADIATOR\_DT)}{(MAX\_INVERTER\_TEMP-MIN\_INVERTER\_TEMP)}$

Where: 

- `MAX_RADIATOR_DT` is the max duty cycle allowed for the radiator
- `MIN_RADIATOR_DT` is the min duty cycle allowed for the radiator
- `MAX_INVERTER_TEMP` is the inverter temperature associated with the `MAX_RADIATOR_DT` value (basically when the radiators need to go to max speed and cool as much as possible)
- `MIN_INVERTER_TEMP` is the inverter min temperature associated with `MIN_RADIATOR_DT` value (basically when radiators need to start cooling at them min duty cycle)

The q factor as follows: 

$q = (MIN\_RADIATOR\_DT-(MIN\_INVERTER\_TEMP*m))$

Given:

Costant           |Value
------------------|------
MAX_RADIATOR_DT   | 0.9
MIX_RADIATOR_DT   | 0.8
MAX_INVERTER_TEMP | 60
MIN_INVERTER_TEMP | 40

We can obtain:

Inverter temperature °C    |Radiator Duty Cycle
---------------------------|------
0                          | -0.3
10                         | -0.1
20                         | 0.1
30                         | 0.3
40                         | 0.5
50                         | 0.7
60                         | 0.9

!!! note
    Negative value are carried to zero via sw


## Pumps
The m factor as follows: 

$m = \frac{(MAX\_OPAMP\_OUT-MIN\_OPAMP\_OUT)}{(MAX\_INVERTER\_TEMP-MIN\_INVERTER\_TEMP)}$

Where: 

- `MAX_OPAMP_OUT` is the max allowed output voltage given by the opamp wich feeds the pumps
- `MAX_OPAMP_OUT` is the min allowed output voltage given by the opamp wich feeds the pumps
- `MAX_INVERTER_TEMP` is the inverter temperature associated with the `MAX_OPAMP_OUT` value (basically when the pumps need to go to max power and cool as much as possible)
- `MIN_INVERTER_TEMP` is the inverter min temperature associated with `MIN_OPAMP_OUT` value (basically when pumps need to start cooling at them min power value)

The q factor as follows: $q = MIN\_OPAMP\_OUT-(MIN\_INVERTER\_TEMP*m)$

Given:

Costant           |Value
------------------|------
MAX_OPAMP_OUT     | 4.95
MIN_OPAMP_OUT     | 0
MAX_INVERTER_TEMP | 60
MIN_INVERTER_TEMP | 40

We can obtain:

Inverter temperature °C    |Analog Pumps value
---------------------------|------
0                          | -9.9
10                         | -7.425
20                         | -4.95
30                         | -2.475
40                         | 0
50                         | 2.475
60                         | 4.95

!!! note
    Negative value are carried to zero via sw