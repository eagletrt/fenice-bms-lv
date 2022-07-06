# Low Voltage Battery Management System

The aim of the __LV-BMS__ is to provide the necessary voltage supply to makes all low voltages components working, such as: _DAS_, _Steering wheel_, _Telemetry_ boards, lights on board, pumps and radiators. <br>

Inside its case a (3S1P <s>4S1P</s>) LiPo Battery is present, which, when it's completely charged has a capacity of __16.5 Ah__ and a voltage of __12.45 V__. 
Because of some components need fixed voltages of 12V and 24V, on the _PCB_ are also present two _DCDC_ converter (boost), which can deliver those voltages even with an input of 9V. <br>

The main job of the __LV-BMS__ is to turn on or off the _LVMS_ switch, in order to give or not power to the board. Monitoring the voltages and the temperature of the cells, the temperatures of the _DCDCs_ converters, the battery overcurrent and other more parameters, the board can decide autonomously whether open or close the _relay_ connected to the _LVMS_ switch. Another important feature of the board it's the cooling routine, which consists in the pumps and radiator piloting to keep the inverters and the motors in a specific range of temperatures.

The currently working branch of the repo is: __dev-baremetal__.<br>
All the main functions are mostly in the directory _Lib_

!!! important
    It's advised to check [BMS LV State Machine](Main functions/State machine.md) before reading other chapters.

!!! tip
    Taking a look to the [hardware](https://github.com/eagletrt/fenice-bms-lv-hw) never hurts 🥰
