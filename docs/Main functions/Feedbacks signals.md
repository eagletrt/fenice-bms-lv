# Feedbacks signals

The IC _MCP23017_ is a _GPIO Expander_, which uses I2C bus to monitor the status of some feedbacks on the board.

![mcp23017 feedbacks](../media/mcp23017%20feedbacks.png)

List of the feedbacks on board:

- __FB_Main__: indicate if LVMS is closed 
- __FB_12__: indicate if there are 12V on the _DCDC12V_ output
- __FB_PCBS__: depends on FB_12
- __FB_Relay__: indicate if the relay is closed or not
- __FB_24__: indicate if there are 24V on the _DCDC24V_ output
- __FB_Inverters__: depends on FB_24 
- __FB_PUMPS__: depends on FB_24
- __FB_SHUTDOWN__: depends on FB_12
- __FB_RADIATORS__: depends on FB_12
- __FB_FAN__: depends on FB_12

!!! warning
    With the latest configuration of the board FB_Main is always 1
    !!! note
        Latest configuration = BMS wiring inside its case

!!! faq
    You can obtain a specific Feedback value using a specific functions declared in `main.h`
    !!! example
        ```c 
        //Check if there are 12V on board, needed to supply other boards
        bool feedback_state = FDBK_12V_PCBS_get_state();
        ```