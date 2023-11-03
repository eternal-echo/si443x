#include "si443x.h"
#include "si443x_io.h"
#include <math.h>

#define MAX_TRANSMIT_TIMEOUT_MS 200

#define SI443X_DEFAULT_FREQ 433
#define SI443X_DEFAULT_CHANNEL 0
#define SI443X_DEFAULT_BAUD_RATE 100
#define SI443X_DEFAULT_PACKAGE_SIGN 0xDEAD

//values here are kept in khz x 10 format (for not to deal with decimals) - look at AN440 page 26 for whole table
const uint16_t IFFilterTable[][2] = { { 322, 0x26 }, { 3355, 0x88 }, { 3618, 0x89 }, { 4202, 0x8A }, { 4684, 0x8B }, {
		5188, 0x8C }, { 5770, 0x8D }, { 6207, 0x8E } };

/**
 * @brief Reads a register from the radio.
 *
 * When reading a single register of the radio, a 16-bit value must be sent to the
 * radio (the 8-bit address of the register followed by a dummy 8-bit value).
 * The radio provides the value of the register during the second byte of the SPI transaction.
 * Note that it is crucial to clear the MSB of the register address to indicate a read cycle.
 * [AN415:EZRADIOPRO® PROGRAMMING GUIDE]
 *
 * This function reads a register value from the radio module.
 * The read operation involves sending the register address (with MSB cleared) followed by
 * a dummy byte to the radio module over the SPI interface. The radio will respond with the
 * register value during the second byte of the SPI transaction.
 *
 * @param si443x Pointer to the si443x_t structure representing the radio module.
 * @param reg The 8-bit address of the register to be read.
 * @param data Pointer to the data buffer to store the read value.
 * @param len Length of the data buffer to store the read value.
 * @return An 8-bit integer indicating the status of the read operation.
 */
static void burst_read(si443x_t *si443x, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t reg_val = reg & 0x7F; // clear the MSB (indicating a read cycle)
    uint8_t i = 0;

    if(si443x == NULL || data == NULL || len == 0) {
        return ;
    }
    // Start SPI transaction and select the radio by pulling the nSEL pin low
    si443x_gpio_reset_cs(si443x);
    // Write the address of the register into the SPI buffer of the MCU (with MSB cleared)
    si443x_spi_tx(si443x, reg_val);
    // Write a dummy data byte into the SPI buffer of the MCU to initiate the read cycle
    // During sending this byte the MCU will read the value of the radio register and save it
    for (i = 0; i < len; ++i) {
        data[i] = si443x_spi_tx_rx(si443x, 0x00);
    }
    // Deselect the radio by pulling high the nSEL pin
    si443x_gpio_set_cs(si443x);

    // DEBUG
    SI443X_DEBUG_PRINT("burst_read: reg: %02x, data: ", reg);
    for (i = 0; i < len; ++i) {
        SI443X_DEBUG_PRINT("%02x ", data[i]);
    }
}

/**
 * @brief Writes a new value into a register on the radio.
 *
 * If only one register of the radio is written, a 16-bit value must be sent to the
 * radio (the 8-bit address of the register followed by the new 8-bit value of the register).
 * To write a register, the MSB of the address is set to 1 to indicate a device write.
 * [AN415:EZRADIOPRO® PROGRAMMING GUIDE]
 *
 * @param si443x Pointer to the si443x_t structure representing the radio module.
 * @param reg The 8-bit address of the register to be written.
 * @param data Pointer to the data buffer containing the new value to be written.
 * @param len Length of the data buffer to be written.
 * @return An 8-bit integer indicating the status of the write operation.
 */
static void burst_write(si443x_t *si443x, uint8_t reg, const uint8_t *data, uint8_t len)
{
    // important to set the MSB bit
    uint8_t reg_val = reg | 0x80; // set the MSB

    if(si443x == NULL || data == NULL || len == 0) {
        return ;
    }
    // Select the radio by pulling the nSEL pin to low
    si443x_gpio_reset_cs(si443x);
    // Write the address of the register into the SPI buffer of the MCU (with MSB set)
    si443x_spi_tx(si443x, reg_val);
    // Write the new value of the register into the SPI buffer of the MCU
    for (uint8_t i = 0; i < len; ++i) {
        si443x_spi_tx(si443x, data[i]);
    }
    // Deselect the radio by pulling high the nSEL pin
    si443x_gpio_set_cs(si443x);
}

static void change_register(si443x_t *si443x, uint8_t reg, uint8_t value)
{
    burst_write(si443x, reg, &value, 1);

#ifdef SI443X_DEBUG
	// reg = 0x07 and value = 0x80 is Software Register Reset Bit.
	// This bit will be automatically cleared.
    if(reg != 0x07 || value != 0x80) 
    {
        uint8_t _value = 0;
        burst_read(si443x, reg, &_value, 1);
        if(_value != value) 
        {
            SI443X_DEBUG_PRINT("change register failed!\r\n");
            SI443X_DEBUG_PRINT("reg: %02x, value: %02x, _value: %02x\r\n", reg, value, _value);
        }
    }
#endif
}

static uint8_t read_register(si443x_t *si443x, uint8_t reg)
{
    uint8_t value = 0;
    burst_read(si443x, reg, &value, 1);
    return value;
}

/**
 * @brief Switch to the given mode
 * 
 * @param si443x 
 * @param mode 
 * @return int8_t 
 */
static void si443x_switch_mode(si443x_t *si443x, si443x_antenna_mode_t mode)
{
    SI443X_DEBUG_PRINT("switching to mode: %02x\r\n", mode);
    change_register(si443x, REG_STATE, mode);

#ifdef SI443X_DEBUG
    uint8_t state = read_register(si443x, REG_STATE);

    if(state != mode) {
        SI443X_DEBUG_PRINT("switching to mode failed!\r\n");
        SI443X_DEBUG_PRINT("state: %02x, mode: %02x\r\n", state, mode);
    }
#endif
}

static void boot(si443x_t *si443x)
{
    change_register(si443x, REG_AFC_TIMING_CONTROL, 0x02); // refer to AN440 for reasons
	change_register(si443x, REG_AFC_TIMING_CONTROL, 0x02); // refer to AN440 for reasons
	change_register(si443x, REG_AFC_LIMITER, 0xFF); // write max value - excel file did that.
	change_register(si443x, REG_AGC_OVERRIDE, 0x60); // max gain control
	change_register(si443x, REG_AFC_LOOP_GEARSHIFT_OVERRIDE, 0x3C); // turn off AFC

    /* Configure the receive packet handler */
	change_register(si443x, REG_DATAACCESS_CONTROL, 0xAD); // enable rx packet handling, enable tx packet handling, enable CRC, use CRC-IBM
	change_register(si443x, REG_HEADER_CONTROL1, 0x0C); // no broadcast address control, enable check headers for bytes 3 & 2
	change_register(si443x, REG_HEADER_CONTROL2, 0x22);  // enable headers uint8_t 3 & 2, no fixed package length, sync word 3 & 2
	change_register(si443x, REG_PREAMBLE_LENGTH, 0x08); // 8 * 4 bits = 32 bits (4 bytes) preamble length
	change_register(si443x, REG_PREAMBLE_DETECTION, 0x3A); // validate 7 * 4 bits of preamble  in a package
	change_register(si443x, REG_SYNC_WORD3, 0x2D); // sync uint8_t 3 val
	change_register(si443x, REG_SYNC_WORD2, 0xD4); // sync uint8_t 2 val

	change_register(si443x, REG_TX_POWER, 0x1F); // max power

	change_register(si443x, REG_CHANNEL_STEPSIZE, 0x64); // each channel is of 1 Mhz interval

    /*set the physical signal parameters*/
    si443x_set_frequency(si443x, si443x->freq_carrier); // default freq
    /*set the modem parameters according to the exel calculator(parameters: 100kbps, deviation: 25 kHz, channel filter BW: 112.1 kHz*/
	si443x_set_baud_rate(si443x, si443x->kbps); // default baud rate is 100kbps
	si443x_set_channel(si443x, si443x->freq_channel); // default channel is 0
	si443x_set_comms_signature(si443x, si443x->package_sign); // default signature

	si443x_switch_mode(si443x, SI443X_MODE_READY);
}

int8_t si443x_init(si443x_t *si443x)
{
    int8_t ret = SI443X_ERR_OK;
    if(si443x == NULL) {
        return SI443X_ERR_INVALID_PARAM;
    }

    // set default values
    si443x->freq_carrier = SI443X_DEFAULT_FREQ;
    si443x->freq_channel = SI443X_DEFAULT_CHANNEL;
    si443x->kbps = SI443X_DEFAULT_BAUD_RATE;
    si443x->package_sign = SI443X_DEFAULT_PACKAGE_SIGN;

    // init gpio and spi
    si443x_gpio_init(si443x);
    // turn off
    si443x_turn_off(si443x);

    // init spi and set cs pin to high, so that the chip is not selected
    // Max SPI clock frequency is 10MHz
    si443x_spi_init(si443x);

    SI443X_DEBUG_LOG("spi is initialized now");

    si443x_hardware_reset(si443x);

    // check sync word
    uint8_t sync_word[4] = {0};
    burst_read(si443x, (uint8_t)REG_SYNC_WORD3, sync_word, 4);

    SI443X_DEBUG_PRINT("sync word3: %02x\n", sync_word[0]);
    SI443X_DEBUG_PRINT("sync word2: %02x\n", sync_word[1]);
    SI443X_DEBUG_PRINT("sync word1: %02x\n", sync_word[2]);
    SI443X_DEBUG_PRINT("sync word0: %02x\n", sync_word[3]);

    if (sync_word[0] != 0x2D || sync_word[1] != 0xD4)
    {
        ret = SI443X_ERR_FAILED;
    }
    return ret;
}

void si443x_hardware_reset(si443x_t *si443x)
{

    uint8_t reg = 0, it_status1 = 0, it_status2 = 0;

    // Driving this pin low enables the radio and performs a power on reset 
    // cycle, which takes about 15 ms. [AN415:EZRADIOPRO® PROGRAMMING GUIDE#2.2.2]
    si443x_turn_off(si443x);
    si443x_turn_on(si443x);
    //read interrupt status registers to clear the interrupt flags and release NIRQ pin
    it_status1 = read_register(si443x, REG_INT_STATUS1);
    it_status2 = read_register(si443x, REG_INT_STATUS2);

    // SW reset
    change_register(si443x, REG_STATE, 0x80);

    // wait for POR interrupt from the radio (while the nIRQ pin is high)
    while(si443x_gpio_get_nirq(si443x) != 0);

    // ichiprdy Chip SI443X_MODE_READY (XTAL).
    // When a chip ready event has been detected this bit will be set to 1
    do {
        it_status2 = read_register(si443x, REG_INT_STATUS2);
        SI443X_DELAY_MS(1);
    } while((reg & 0x02) != 0x02);

    boot(si443x);
}

void si443x_software_reset(si443x_t *si443x)
{


    change_register(si443x, REG_STATE, SI443X_MODE_READY);

    // swres Software Reset.
    change_register(si443x, REG_STATE, 0x80);

    // wait for chip ready
    uint8_t reg = 0;
    do {
        reg = read_register(si443x, REG_INT_STATUS2);
        SI443X_DELAY_MS(1);
    } while((reg & 0x02) != 0x02);

    boot(si443x);
    return SI443X_ERR_OK;
}

/**
 * @brief Turn on the chip
 * 
 * @param si443x 
 * @return int 
 */
void si443x_turn_on(si443x_t *si443x)
{
    si443x_gpio_reset_sdn(si443x);
    SI443X_DELAY_MS(20);
}

/**
 * @brief Turn off the chip
 * 
 * @param si443x 
 * @return int 
 */
void si443x_turn_off(si443x_t *si443x)
{
    si443x_gpio_set_sdn(si443x);
    SI443X_DELAY_MS(20);
}

/**
 * @brief Set the frequency of the chip. Call before boot.
 * 
 * @param si443x 
 * @param base_freq_MHz 
 * @return int8_t 
 */
void si443x_set_frequency(si443x_t *si443x, uint16_t base_freq_MHz)
{
    if(base_freq_MHz < 240 || base_freq_MHz > 930) {
        return ;
    }

    si443x->freq_carrier = base_freq_MHz;
    uint8_t high_band = 0;
    if(base_freq_MHz >= 480) {
        high_band = 1;
    }

    double f_part = ((double)base_freq_MHz / (10 * (high_band + 1))) - 24;

    uint8_t freq_band = (uint8_t)f_part; // truncate the integer part
    uint16_t freq_carr = (uint16_t)((f_part - freq_band) * 64000); // get the fractional part

    // sideband is always on (0x40)
    uint8_t vals[3] = { 0x40 | (high_band << 5) | (freq_band & 0x3F), (uint8_t)(freq_carr >> 8), (uint8_t)(freq_carr & 0xFF) };

    burst_write(si443x, REG_FREQBAND, vals, 3);
}

/**
 * @brief Set the baud rate of the chip. Call before switching to tx or rx mode. - min:1, max: 256
 * 
 * @param si443x
 * @param kbps
 * 
 * @return int8_t 
 */
void si443x_set_baud_rate(si443x_t *si443x, uint8_t kbps)
{
	// chip normally supports very low bps values, but they are cumbersome to implement - so I just didn't implement lower bps values
    if ((kbps > 256) || (kbps < 1))
    {
        return SI443X_ERR_INVALID_PARAM;
    }
    si443x->kbps = kbps;

    uint8_t freq_dev = kbps <= 10 ? 15 : 150; // 15khz / 150 khz
//  uint8_t modulation_value = kbps < 30 ? 0x4c : 0x0c; // use FIFO Mode, GFSK, low baud mode on / off
    uint8_t modulation_value = kbps < 30 ? 0x2c : 0x0c; // use FIFO Mode, GFSK, low baud mode on / off

    uint8_t modulation_vals[] = { modulation_value, 0x23, round((freq_dev * 1000.0) / 625.0) }; // msb of the kpbs to 3rd bit of register
    burst_write(si443x, REG_MODULATION_MODE1, modulation_vals, 3); // 0x70
    // set data rate
    uint16_t bps_reg_val = round((kbps * 65536.0) / 1000.0);
    uint8_t datarate_vals[] = { bps_reg_val >> 8, bps_reg_val & 0xFF };

    burst_write(si443x, REG_TX_DATARATE1, datarate_vals, 2); // 0x6E

    // now set the timings
    uint16_t min_bandwidth = (2 * (uint32_t) freq_dev) + kbps;

    SI443X_DEBUG_PRINT("min Bandwidth value: 0x%02x\r\n", min_bandwidth);

    uint8_t IFValue = 0x1D; // 0x1D is the default value
    // since the table is ordered (from low to high), just find the 'minimum bandwidth which is greater than required'
    for(uint8_t i = 0; i < 8; ++i) {
        if(IFFilterTable[i][0] >= (min_bandwidth * 10)) {
            IFValue = IFFilterTable[i][1];
            break;
        }
    }
    SI443X_DEBUG_PRINT("Selected IF value: 0x%02x\r\n", IFValue);

    change_register(si443x, REG_IF_FILTER_BW, IFValue);

    uint8_t dwn3_bypass = (IFValue & 0x80) ? 1 : 0; // if msb is set
    uint8_t ndec_exp = (IFValue >> 4) & 0x07; // only 3 bits

    uint16_t rx_oversampling = round((500.0 * (1 + 2 * dwn3_bypass)) / ((pow(2, ndec_exp - 3)) * (double ) kbps));

    uint16_t nc_offset = round(((double) kbps * (pow(2, ndec_exp + 20))) / (500.0 * (1 + 2 * dwn3_bypass)));

    uint16_t cr_gain = 2 + round((65535.0 * (double) kbps) / ((double) rx_oversampling * freq_dev));
    uint8_t cr_multiplier = 0x00;
	if (cr_gain > 0x7FF) {
		cr_gain = 0x7FF;
	}

    SI443X_DEBUG_PRINT("dw3_bypass: %02x, ndec_exp: %02x, rx_oversampling: %04x, nc_offset: %04x, cr_gain: %04x, cr_multiplier: %02x\r\n",
        dwn3_bypass, ndec_exp, rx_oversampling, nc_offset, cr_gain, cr_multiplier);

    uint8_t timingVals[] = { rx_oversampling & 0x00FF, ((rx_oversampling & 0x0700) >> 3) | ((nc_offset >> 16) & 0x0F),
            (nc_offset >> 8) & 0xFF, nc_offset & 0xFF, ((cr_gain & 0x0700) >> 8) | cr_multiplier, cr_gain & 0xFF };

    burst_write(si443x, REG_CLOCK_RECOVERY_OVERSAMPLING, timingVals, 6);

    return SI443X_ERR_OK;
}

/**
 * @brief Set the channel of the chip. Call before switching to tx or rx mode.
 * 
 * @param si443x 
 * @param channel 
 * @return int8_t 
 */
void si443x_set_channel(si443x_t *si443x, uint8_t channel)
{
    si443x->freq_channel = channel;
    change_register(si443x, REG_FREQCHANNEL, channel);
}

/**
 * @brief used to 'sign' packets with a predetermined signature - call before boot
 * 
 * @param si443x 
 * @param signature 
 * @return int8_t 
 */
void si443x_set_comms_signature(si443x_t *si443x, uint16_t signature)
{
    si443x->package_sign = signature;
    change_register(si443x, REG_TRANSMIT_HEADER3, (uint8_t)(signature >> 8));
    change_register(si443x, REG_TRANSMIT_HEADER2, (uint8_t)(signature & 0xFF));
    change_register(si443x, REG_CHECK_HEADER3, (uint8_t)(signature >> 8));
    change_register(si443x, REG_CHECK_HEADER2, (uint8_t)(signature & 0xFF));
    SI443X_DEBUG_LOG("Package signature is set");
    return SI443X_ERR_OK;
}

void si443x_read_all(si443x_t *si443x)
{
    uint8_t reg;

    for(uint8_t i = 0; i < 0x7F; ++i) {
        SI443X_DEBUG_PRINT("reg(%02x): %02x\r\n", i, read_register(si443x, i));
    }
}

int8_t si443x_send_packet(si443x_t *si443x, const uint8_t* data, uint8_t length)
{
    // disable the receiver chain (but keep the XTAL running to have shorter TX on time!)
    si443x_switch_mode(si443x, SI443X_MODE_READY);
	si443x_clear_txFIFO(si443x);
    // set the length of the payload to 'length'	
	change_register(si443x, REG_PKG_LEN, length);
    // fill the payload into the transmit FIFO
	burst_write(si443x, REG_FIFO, data, length);

    //Disable all other interrupts and enable the packet sent interrupt only.
    //This will be used for indicating the successfull packet transmission for the MCU
	change_register(si443x, REG_INT_ENABLE1, 0x04); // set interrupts on for package sent
	change_register(si443x, REG_INT_ENABLE2, 0x00); // set interrupts off for anything else
	//read interrupt registers to clean them
	read_register(si443x, REG_INT_STATUS1);
	read_register(si443x, REG_INT_STATUS2);

    /*enable transmitter*/
    //The radio forms the packet and send it automatically.
	si443x_switch_mode(si443x, SI443X_MODE_TX | SI443X_MODE_READY);

    /*wait for the packet sent interrupt*/
    //The MCU just needs to wait for the 'ipksent' interrupt.
	uint32_t timeout = 0;
    do {
        uint8_t int_status = read_register(si443x, REG_INT_STATUS1);
        if(int_status & 0x04) { // Packet Sent Interrupt
            si443x_switch_mode(si443x, SI443X_MODE_READY | SI443X_MODE_TUNE);
            return SI443X_ERR_OK;
        }
        if(si443x_gpio_get_nirq(si443x) == 0) {
            return SI443X_ERR_OK;
        }
    } while(timeout++ < MAX_TRANSMIT_TIMEOUT_MS);

	SI443X_DEBUG_PRINT("Timeout in Transit -- ");
    si443x_hardware_reset(si443x);

	return SI443X_ERR_TIMEOUT;
}

void si443x_get_packet_received(si443x_t *si443x, uint8_t* readData, uint8_t* length)
{
	*length = read_register(si443x, REG_RECEIVED_LENGTH);

    burst_read(si443x, REG_FIFO, readData, *length);

	si443x_clear_rxFIFO(si443x); // which will also clear the interrupts
}


void si443x_clear_txFIFO(si443x_t *si443x) 
{

	change_register(si443x, REG_OPERATION_CONTROL, 0x01);
	change_register(si443x, REG_OPERATION_CONTROL, 0x00);
}

void si443x_clear_rxFIFO(si443x_t *si443x) 
{
    change_register(si443x, REG_OPERATION_CONTROL, 0x02);
    change_register(si443x, REG_OPERATION_CONTROL, 0x00);
}

void si443x_clear_FIFO(si443x_t *si443x) 
{

    change_register(si443x, REG_OPERATION_CONTROL, 0x03);
    change_register(si443x, REG_OPERATION_CONTROL, 0x00);
}


void si443x_start_listening(si443x_t *si443x) 
{


	si443x_clear_rxFIFO(si443x); // clear first, so it doesn't overflow if packet is big

	change_register(si443x, REG_INT_ENABLE1, 0x03); // set interrupts on for package received and CRC error

#ifdef SI443X_DEBUG
	change_register(si443x, REG_INT_ENABLE2, 0xC0);
#else
	change_register(si443x, REG_INT_ENABLE2, 0x00); // set other interrupts off
#endif
	//read interrupt registers to clean them
	read_register(si443x, REG_INT_STATUS1);
	read_register(si443x, REG_INT_STATUS2);

    si443x_switch_mode(si443x, SI443X_MODE_RX | SI443X_MODE_READY);

    return SI443X_ERR_OK;
}


void si443x_is_packetReceived(si443x_t *si443x) 
{

    // if no interrupt occurred, no packet received is assumed (since startListening will be called prior, this assumption is enough)
	if (si443x_gpio_get_nirq(si443x) == 1) {
        return SI443X_ERR_NO_PACKET;
    }
	// check for package received status interrupt register
	uint8_t int_stat1 = read_register(si443x, REG_INT_STATUS1);
    SI443X_DEBUG_PRINT("isPacketReceived REG_INT_STATUS1=%02x\r\n", int_stat1);

	uint8_t int_stat2 = read_register(si443x, REG_INT_STATUS2);
    SI443X_DEBUG_PRINT("isPacketReceived REG_INT_STATUS2=%02x\r\n", int_stat2);

	if (int_stat2 & 0x40) { //interrupt occurred, check it && read the Interrupt Status1 register for 'preamble '
        SI443X_DEBUG_PRINT("Valid Preamble detected -- %02x\r\n", int_stat2);
	}

	if (int_stat2 & 0x80) { //interrupt occurred, check it && read the Interrupt Status1 register for 'preamble '
        SI443X_DEBUG_PRINT("SYNC WORD detected -- %02x\r\n", int_stat2);

	}

	if (int_stat1 & 0x02) { //interrupt occurred, check it && read the Interrupt Status1 register for 'valid packet'
		si443x_switch_mode(si443x, SI443X_MODE_READY | SI443X_MODE_TUNE); // if packet came, get out of Rx mode till the packet is read out. Keep PLL on for fast reaction
        SI443X_DEBUG_PRINT("Packet detected -- %02x\r\n", int_stat1);
		return SI443X_ERR_OK;
	} else if (int_stat1 & 0x01) { // packet crc error
		si443x_switch_mode(si443x, SI443X_MODE_READY); // get out of Rx mode till buffers are cleared
        SI443X_DEBUG_PRINT("CRC Error in Packet detected -- %02x\r\n", int_stat1);
		si443x_clear_rxFIFO(si443x);
		si443x_switch_mode(si443x, SI443X_MODE_RX | SI443X_MODE_READY); // get back to work
		return SI443X_ERR_CRC;
	}

	//no relevant interrupt? no packet!

	return SI443X_ERR_NO_PACKET;
}
