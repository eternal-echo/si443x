#ifndef __SI443X_IO_H__
#define __SI443X_IO_H__

#include <stdint.h>
#include "si443x.h"

#include "../../mcc_generated_files/system/system.h"

#define SI443X_DEBUG
#define SI443X_DEBUG_PRINT
#define SI443X_DEBUG_LOG
#define SI443X_DELAY_MS

// SPI接口函数
void si443x_spi_init(si443x_t *si443x);
uint8_t si443x_spi_tx_rx(si443x_t *si443x, uint8_t tx_data);
void si443x_spi_tx(si443x_t *si443x, uint8_t data);
uint8_t si443x_spi_rx(si443x_t *si443x);

// GPIO接口函数
void si443x_gpio_init(si443x_t *si443x);
void si443x_gpio_set_cs(si443x_t *si443x);
void si443x_gpio_reset_cs(si443x_t *si443x);
void si443x_gpio_set_sdn(si443x_t *si443x);
void si443x_gpio_reset_sdn(si443x_t *si443x);
uint8_t si443x_gpio_get_nirq(si443x_t *si443x);

#endif /* __SI443X_IO_H__ */