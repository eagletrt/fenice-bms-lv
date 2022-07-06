# State Machine

Here you can find the firmware flowchart for the __LV-BMS__, don't be scared, there isn't a real state machine at all!

The flowchart can be splitted into three stages:

1. __Warm up stage__: Nothing in the car is already powered, the BMS is still initializing the peripherals and checking if there's a minimum amount of voltage from the battery to close the relay.
2. __Run stage__: All the time critical tasks are executed
3. __Error stage__: The system can still communicate via CLI and CAN Bus, however the relays is switched off and the car is not powered
```diagram
graph TD;
    subgraph Warm up stage
        A[Peripherals init] -->B[Accumulator total  voltage measurement]
        B-->C{Measure attempt <br> <= <br> VOLT_MAX_ATTEMPTS}
        C--YES-->D{Total voltage measured <br> > <br> MIN_POWER_ON_VOLTAGE?}
        D--YES-->F[Close relay + Play buzzer]
        end
        C--NO-->E[Error state]
        D--NO-->B
    subgraph Run stage
        F-->F1[CLI Loop]
        F1-->F2[Do periodic measurements and send can messages]
        F2-->F2.1[Check feedback values]
        F2.1-->F3[Run cooling routine]
        F3-->F4{Errors or SCS?}
        F4--NO-->F1
        F4--YES-->F5{Has error <br> reached its TIMEOUT?}
        F5--NO-->F1
    end
    subgraph Error stage
        F5--YES-->E
        E-->E1[Open relay]
        E1-->E2[Loop CLI + Led Blink]
        E2-->E3[Do periodic measurements and send can messages]
        E3-->E2
    end
```
