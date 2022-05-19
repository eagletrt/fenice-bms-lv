/**
 * @file			error_list_ref.h
 * @brief			This file contains variables that reference node_t in error_list: error list reference/s
 * @deatails	This file is a "database" of all variables concerning errors.
* 						<b>ERRORS ARE NOT CONTAINED IN THESE VARIABLES</b>
 * 						These variables reference positions in the error_list DatStruct.
 * 						Their main purpose is to give access in O(1) complexity to errors they are
 * 						representing. The way these error_list_ref_X variables are called is by the use of
 * 						the error_list_reference array, indexed by the error type (error_type_t)
 * 						and an offset (for error types that can have multiple instances).
 * 
 * @date			March 12, 2019
 *
 * @author		Matteo Bonora [matteo.bonora@studenti.unitn.it]
 * @author		Simone Ruffini [simone.ruffini@studenti.unitn.it]
 */

#include "error_list_ref.h"

#include "fenice-config.h"
#include "ltc_config.h"

#include <stdio.h>

/*  
    This file can contain a variable if and only if:
        1) A peripheral/device can generate data that could have errors
        1.1) This peripheral/device data must be defiend in data.c
        2) The peripheral/device data has an error_type_t descriptor
        2.1) NB two ore error_type_t descriptors can be associated to the same error_list_ref
    then:
    the name of the peripheral/device error list reference is defined as follows:
    node_t* error_list_ref_<insert the relative data name in data.c>;
*/
llist_node error_list_ref_relay[1]                        = {NULL};
llist_node error_list_ref_ltc6810[1]                      = {NULL};
llist_node error_list_ref_low_voltages[LV_CELLS_COUNT]    = {NULL};
llist_node error_list_ref_high_voltages[LV_CELLS_COUNT]   = {NULL};
llist_node error_list_ref_mcp23017[1]                     = {NULL};
llist_node error_list_ref_can[TOTAL_CAN_PHERIPERALS]      = {NULL};
llist_node error_list_ref_radiators[2]                    = {NULL};
llist_node error_list_ref_fan[1]                          = {NULL};
llist_node error_list_ref_pumps[2]                        = {NULL};
llist_node error_list_ref_adc_init[1]                     = {NULL};
llist_node error_list_ref_adc_timeout[1]                  = {NULL};
llist_node error_list_ref_dcdc12_low_temp[1]              = {NULL};
llist_node error_list_ref_dcdc12_high_temp[1]             = {NULL};
llist_node error_list_ref_dcdc24_low_temp[1]              = {NULL};
llist_node error_list_ref_dcdc24_high_temp[1]             = {NULL};
llist_node error_list_ref_cells_low_temp[LV_CELLS_COUNT]  = {NULL};
llist_node error_list_ref_cells_high_temp[LV_CELLS_COUNT] = {NULL};
llist_node error_list_ref_over_current[1]                 = {NULL};
llist_node error_list_ref_dcdc12[1]                       = {NULL};
llist_node error_list_ref_dcdc24[1]                       = {NULL};

llist_node *const error_list_ref_array[ERROR_NUM_ERRORS] = {
    [ERROR_RELAY]                    = error_list_ref_relay,
    [ERROR_LTC6810]                  = error_list_ref_ltc6810,
    [ERROR_CELL_UNDERVOLTAGE]        = error_list_ref_low_voltages,
    [ERROR_CELL_OVERVOLTAGE]         = error_list_ref_high_voltages,
    [ERROR_MCP23017]                 = error_list_ref_mcp23017,
    [ERROR_CAN]                      = error_list_ref_can,
    [ERROR_RADIATOR]                 = error_list_ref_radiators,
    [ERROR_FAN]                      = error_list_ref_fan,
    [ERROR_PUMP]                     = error_list_ref_pumps,
    [ERROR_ADC_INIT]                 = error_list_ref_adc_init,
    [ERROR_ADC_TIMEOUT]              = error_list_ref_adc_timeout,
    [ERROR_DCDC12_UNDER_TEMPERATURE] = error_list_ref_dcdc12_low_temp,
    [ERROR_DCDC12_OVER_TEMPERATURE]  = error_list_ref_dcdc12_high_temp,
    [ERROR_DCDC24_UNDER_TEMPERATURE] = error_list_ref_dcdc24_low_temp,
    [ERROR_DCDC24_OVER_TEMPERATURE]  = error_list_ref_dcdc24_high_temp,
    [ERROR_CELL_UNDER_TEMPERATURE]   = error_list_ref_cells_low_temp,
    [ERROR_CELL_OVER_TEMPERATURE]    = error_list_ref_cells_high_temp,
    [ERROR_OVER_CURRENT]             = error_list_ref_over_current,
    [ERROR_DCDC12]                   = error_list_ref_dcdc12,
    [ERROR_DCDC24]                   = error_list_ref_dcdc24,

};

/**
 * @brief Returns the cell memory position of error_list_ref_array
 * 
 * @details error_list_ref_array is an array of llist_node arrays (not a matrix). After first dereference
 * 			by @param id we get the memory addres of the relative error_<id> error variable. 
 * 			With that memory addres we can offset by @param offset (in case the error_<id> is an array) 
 * 			and get a memory addres: &error_<id>[offset].
 * @param 	id 			The error id
 * @param	offset		The &error_<id>[offset]
 * 
 * @returns a 
 */
llist_node *error_list_ref_array_element(uint16_t id, uint16_t offset) {
    llist_node *tmp = *(error_list_ref_array + id);
    return tmp + offset;
}