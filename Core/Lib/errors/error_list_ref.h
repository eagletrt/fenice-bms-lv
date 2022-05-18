/**
 * @file		error_list_ref.h
 * @brief		This file contains variables that reference node_t in error_list: error list reference/s
 * @details     This file is a "database" of all variables concerning errors.
* 				<b>ERRORS ARE NOT CONTAINED IN THESE VARIABLES</b>
 * 				These variables reference positions in the error_list DatStruct.
 * 				Their main purpose is to give access in O(1) complexity to errors they are
 * 				representing. The way these error_list_ref_X variables are called is by the use of
 * 				the error_list_reference array, indexed by the error type (error_type_t)
 * 				and an offset (for error types that can have multiple instances).
 * 
 * @date		March 12, 2019
 *
 * @author		Matteo Bonora [matteo.bonora@studenti.unitn.it]
 * @author		Simone Ruffini [simone.ruffini@studenti.unitn.it]
 */

#ifndef ERROR_LIST_REF_H
#define ERROR_LIST_REF_H

#include "error.h"
#include "llist.h"
//#include "mainboard_config.h"

#include <stdio.h>

//#define ERROR_GET_REF(__ID__, __OFFSET__)
//	(*((error_list_ref_array[__ID__]) + __OFFSET__))

llist_node *error_list_ref_array_element(uint16_t id, uint16_t offset);
//llist_node *error_get_array_element(uint16_t id, uint16_t offset);
/**                                                                                                             
 * @brief	this array contains the references to error_list_ref_XXX variables     
 * @details	this array is indexed by using error_type_t enum, an element is contained in this array if:         \
 *          is defined prevoiusly in this file as an node_t variable                                            \
 *          The position that this variable will take in this array dependes on the descriptor                  \
 *          error_type_t value associated to it, example:                                                       \
 *          error_list_ref_voltages is associated to both ERROR_CELL_UNDER_VOLTAGE and ERROR_CELL_OVER_VOLTAGE, \
 *          so error_list_ref_voltages must be placed in position valueof(ERROR_CELL_UNDER_VOLTAGE)             \
 *          and valueof(ERROR_CELL_OVER_VOLTAGE)                                                                \
 *                                                                                                              \
 */
extern llist_node *const error_list_ref_array[ERROR_NUM_ERRORS];
#endif