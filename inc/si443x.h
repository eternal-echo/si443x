#ifndef SI443x_H_
#define SI443x_H_

#include <stdio.h>
#include <stdint.h>

#define SI443X_ERR_OK 0
#define SI443X_ERR_INVALID_PARAM -1
#define SI443X_ERR_FAILED -2
#define SI443X_ERR_TIMEOUT -3
#define SI443X_ERR_CRC -4
#define SI443X_ERR_NO_PACKET -5

typedef struct {
    /* hardware */

    /* attributes */
    uint16_t freq_carrier;
    uint8_t freq_channel;
    uint16_t kbps;
    uint16_t package_sign;
} si443x_t;

typedef enum {
    REG_DEV_TYPE = 0x00,
    REG_DEV_VERSION = 0x01,
    REG_DEV_STATUS = 0x02,

    REG_INT_STATUS1 = 0x03,
    REG_INT_STATUS2 = 0x04,
    REG_INT_ENABLE1 = 0x05,
    REG_INT_ENABLE2 = 0x06,
    REG_STATE = 0x07,
    REG_OPERATION_CONTROL = 0x08,

    REG_GPIO0_CONF = 0x0B,
    REG_GPIO1_CONF = 0x0C,
    REG_GPIO2_CONF = 0x0D,
    REG_IOPORT_CONF = 0x0E,

    REG_IF_FILTER_BW = 0x1C,
    REG_AFC_LOOP_GEARSHIFT_OVERRIDE = 0x1D,
    REG_AFC_TIMING_CONTROL = 0x1E,
    REG_CLOCK_RECOVERY_GEARSHIFT = 0x1F,
    REG_CLOCK_RECOVERY_OVERSAMPLING = 0x20,
    REG_CLOCK_RECOVERY_OFFSET2 = 0x21,
    REG_CLOCK_RECOVERY_OFFSET1 = 0x22,
    REG_CLOCK_RECOVERY_OFFSET0 = 0x23,
    REG_CLOCK_RECOVERY_TIMING_GAIN1 = 0x24,
    REG_CLOCK_RECOVERY_TIMING_GAIN0 = 0x25,
    REG_RSSI = 0x26,
    REG_RSSI_THRESHOLD = 0x27,

    REG_AFC_LIMITER = 0x2A,
    REG_AFC_CORRECTION_READ = 0x2B,

    REG_DATAACCESS_CONTROL = 0x30,
    REG_EZMAC_STATUS = 0x31,
    REG_HEADER_CONTROL1 = 0x32,
    REG_HEADER_CONTROL2 = 0x33,
    REG_PREAMBLE_LENGTH = 0x34,
    REG_PREAMBLE_DETECTION = 0x35,
    REG_SYNC_WORD3 = 0x36,
    REG_SYNC_WORD2 = 0x37,
    REG_SYNC_WORD1 = 0x38,
    REG_SYNC_WORD0 = 0x39,
    REG_TRANSMIT_HEADER3 = 0x3A,
    REG_TRANSMIT_HEADER2 = 0x3B,
    REG_TRANSMIT_HEADER1 = 0x3C,
    REG_TRANSMIT_HEADER0 = 0x3D,

    REG_PKG_LEN = 0x3E,

    REG_CHECK_HEADER3 = 0x3F,
    REG_CHECK_HEADER2 = 0x40,
    REG_CHECK_HEADER1 = 0x41,
    REG_CHECK_HEADER0 = 0x42,

    REG_RECEIVED_HEADER3 = 0x47,
    REG_RECEIVED_HEADER2 = 0x48,
    REG_RECEIVED_HEADER1 = 0x49,
    REG_RECEIVED_HEADER0 = 0x4A,

    REG_RECEIVED_LENGTH = 0x4B,

    REG_CHARGEPUMP_OVERRIDE = 0x58,
    REG_DIVIDER_CURRENT_TRIM = 0x59,
    REG_VCO_CURRENT_TRIM = 0x5A,

    REG_AGC_OVERRIDE = 0x69,

    REG_TX_POWER = 0x6D,
    REG_TX_DATARATE1 = 0x6E,
    REG_TX_DATARATE0 = 0x6F,

    REG_MODULATION_MODE1 = 0x70,
    REG_MODULATION_MODE2 = 0x71,

    REG_FREQ_DEVIATION = 0x72,
    REG_FREQ_OFFSET1 = 0x73,
    REG_FREQ_OFFSET2 = 0x74,
    REG_FREQBAND = 0x75,
    REG_FREQCARRIER_H = 0x76,
    REG_FREQCARRIER_L = 0x77,

    REG_FREQCHANNEL = 0x79,
    REG_CHANNEL_STEPSIZE = 0x7A,

    REG_FIFO = 0x7F,
}si443x_registers_t;

typedef enum {
    SI443X_MODE_READY = 0x01,
    SI443X_MODE_TUNE = 0x02,
    SI443X_MODE_RX = 0x04,
    SI443X_MODE_TX = 0x08,
} si443x_antenna_mode_t;

int8_t si443x_init(si443x_t *si443x);
void si443x_hardware_reset(si443x_t *si443x);
void si443x_software_reset(si443x_t *si443x);
void si443x_turn_on(si443x_t *si443x);
void si443x_turn_off(si443x_t *si443x);
// void si443x_set_frequency(si443x_t *si443x, uint16_t base_freq_MHz);
// void si443x_set_baud_rate(si443x_t *si443x, uint16_t kbps);
// void si443x_set_channel(si443x_t *si443x, uint8_t channel);
// void si443x_set_comms_signature(si443x_t *si443x, uint16_t signature);
void si443x_read_all(si443x_t *si443x);
int8_t si443x_send_packet(si443x_t *si443x, const uint8_t* data, uint8_t length);
uint8_t si443x_get_packet_received(si443x_t *si443x, uint8_t* readData);
void si443x_clear_txFIFO(si443x_t *si443x);
void si443x_clear_rxFIFO(si443x_t *si443x);
void si443x_clear_FIFO(si443x_t *si443x);
void si443x_start_listening(si443x_t *si443x);
int8_t si443x_is_packetReceived(si443x_t *si443x);
#endif /* SI443x_H_ */
