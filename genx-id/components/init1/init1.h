#ifndef _INIT1_H
#define _INIT1_H

#include "esp_err.h"

void ble_get_dev_uuid(uint8_t *dev_uuid);

esp_err_t bluetooth_init(void);

#endif 

