#include "si443x_io.h"

__attribute__((weak)) void si443x_spi_init(si443x_t *si443x) {
    SPI1_Open(MSSP1_DEFAULT);
}

__attribute__((weak)) uint8_t si443x_spi_tx_rx(si443x_t *si443x, uint8_t tx_data) {
    // transmit data and receive data
    return SPI1_ByteExchange(tx_data);
}

__attribute__((weak)) void si443x_gpio_init(si443x_t *si443x) {
    // init CS, SDN, nIRQ
}

__attribute__((weak)) void si443x_gpio_set_cs(si443x_t *si443x) {
    SPI_CS_SetHigh();
}

__attribute__((weak)) void si443x_gpio_reset_cs(si443x_t *si443x) {
    SPI_CS_SetLow();
}

__attribute__((weak)) void si443x_gpio_set_sdn(si443x_t *si443x) {
    SI4431_SDN_SetHigh();
}

__attribute__((weak)) void si443x_gpio_reset_sdn(si443x_t *si443x) {
    SI4431_SDN_SetLow();
}

__attribute__((weak)) uint8_t si443x_gpio_get_nirq(si443x_t *si443x) {
    return SI4431_IRQ_GetValue();
}

void si443x_spi_tx(si443x_t *si443x, uint8_t data) {
    // transmit data
    si443x_spi_tx_rx(si443x, data);
}

uint8_t si443x_spi_rx(si443x_t *si443x) {
    return si443x_spi_tx_rx(si443x, 0x00);
}