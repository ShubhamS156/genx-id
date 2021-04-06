#include "esp_err.h"
#ifndef DATA_DEFS_H
#define DATA_DEFS_H


/*
 * @brief: 
 *  Function to parse the data recieved by application, store it in genx_uuid_t structs
 *  and return a pointer to the created array.
 * @params: none
 * @return: pointer to the array of genx_uuid_t structs.
 */
void genx_uuid_data_parse(const uint8_t **value,uint8_t *stud_arr,uint16_t length);


/*
 * @brief:
 *  Function to take in the array containing uuid for the current class/session and call
 *  the esp_ble_mesh_provisioner_set_dev_uuid_match() function for the received uuids.
 * @params: genx_uuid_t* data
 * @return: 0 if SUCCESS
 *          1 if error
 */
#endif
