/*
    Copyright 2017 fishpepper <AT> gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.

   author: fishpepper <AT> gmail.com
*/

#ifndef HAL_CC25XX_H__
#define HAL_CC25XX_H__

#include <stdint.h>

void hal_cc25xx_init(void);
void hal_cc25xx_set_register(uint8_t reg, uint8_t val);
uint8_t hal_cc25xx_get_register(uint8_t address);
void hal_cc25xx_strobe(uint8_t val);

void hal_cc25xx_enable_receive(void);
void hal_cc25xx_enable_transmit(void);
void hal_cc25xx_enter_rxmode(void);
void hal_cc25xx_enter_txmode(void);

#define hal_cc25xx_rx_sleep() { delay_us(1352); }
#define hal_cc25xx_tx_sleep() { delay_us(1250); }

// not used on d4rii
#define hal_cc25xx_disable_rf_interrupt() {}
#define hal_cc25xx_setup_rf_dma(mode) {}
#define hal_cc25xx_partnum_valid(p, v) ((p == 0x80) && (v = 0x03))

uint8_t hal_cc25xx_get_status(void);
static void hal_cc25xx_init_gpio(void);
uint32_t hal_cc25xx_set_antenna(uint8_t id);
void hal_cc25xx_set_gdo_mode(void);
uint8_t hal_cc25xx_get_gdo_status(void);
void hal_cc25xx_process_packet(volatile uint8_t *packet_received,
                               volatile uint8_t *buffer, uint8_t maxlen);
void hal_cc25xx_transmit_packet(volatile uint8_t *buffer, uint8_t len);

void hal_cc25xx_read_fifo(uint8_t *buf, uint8_t len);
void hal_cc25xx_register_read_multi(uint8_t address, uint8_t *buffer, uint8_t len);
uint8_t hal_cc25xx_transmission_completed(void);

// adress checks
#define CC2500_PKTCTRL1_FLAG_ADR_CHECK_00 ((0<<1) | (0<<0))
#define CC2500_PKTCTRL1_FLAG_ADR_CHECK_01 ((0<<1) | (1<<0))
#define CC2500_PKTCTRL1_FLAG_ADR_CHECK_10 ((1<<1) | (0<<0))
#define CC2500_PKTCTRL1_FLAG_ADR_CHECK_11 ((1<<1) | (1<<0))
// append status bytes?
#define CC2500_PKTCTRL1_APPEND_STATUS     (1<<2)
// crc autoflush
#define CC2500_PKTCTRL1_CRC_AUTOFLUSH     (1<<3)

// Flags
#define BURST_FLAG   0b01000000
#define WRITE_FLAG   0b00000000
#define READ_FLAG    0b10000000

// Definitions for burst/single access to registers
#define CC2500_WRITE_SINGLE     0x00
#define CC2500_WRITE_BURST      0x40
#define CC2500_READ_SINGLE      0x80
#define CC2500_READ_BURST       0xC0

#define CC2500_STATUS_STATE_IDLE   (0<<4)
#define CC2500_STATUS_STATE_RX     (1<<4)
#define CC2500_STATUS_STATE_TX     (2<<4)
#define CC2500_STATUS_STATE_FSTXON (3<<4)
#define CC2500_STATUS_STATE_CALIBRATE  (4<<4)
#define CC2500_STATUS_STATE_SETTLING   (5<<4)
#define CC2500_STATUS_STATE_RXFIFO_OVF (6<<4)
#define CC2500_STATUS_STATE_TXFIFO_OVF (7<<4)

#define hal_cc25xx_get_register_burst(x)  hal_cc25xx_get_register(x | READ_FLAG | BURST_FLAG)


// strobes
#define RFST_SRES     0x30
#define RFST_SFSTXON  0x31
#define RFST_SXOFF    0x32
#define RFST_SCAL     0x33
#define RFST_SRX      0x34
#define RFST_STX      0x35
#define RFST_SIDLE    0x36
#define RFST_SWOR     0x38
#define RFST_SPWD     0x39
#define RFST_SFRX     0x3A
#define RFST_SFTX     0x3B
#define RFST_SWORRST  0x3C
#define RFST_SNOP     0x3D

// Status registers
#define PARTNUM        0x30|BURST_FLAG
#define VERSION        0x31|BURST_FLAG
#define FREQEST        0x32|BURST_FLAG
#define LQI            0x33|BURST_FLAG
#define RSSI           0x34|BURST_FLAG
#define MARCSTATE      0x35|BURST_FLAG
#define WORTIME1       0x36|BURST_FLAG
#define WORTIME0       0x37|BURST_FLAG
#define PKTSTATUS      0x38|BURST_FLAG
#define VCO_VC_DAC     0x39|BURST_FLAG
#define TXBYTES        0x3A|BURST_FLAG
#define RXBYTES        0x3B|BURST_FLAG
#define RCCTRL1_STATUS 0x3C|BURST_FLAG
#define RCCTRL0_STATUS 0x3D|BURST_FLAG

// Status byte states
#define STB_IDLE         0x00
#define STB_RX           0x10
#define STB_TX           0x20
#define STB_FSTXON       0x30
#define STB_CALIBRATE    0x40
#define STB_SETTLING     0x50
#define STB_RX_OVF       0x60
#define STB_TX_UNDF      0x70

// Config registers addresses
#define IOCFG2   0x00
#define IOCFG1   0x01
#define IOCFG0   0x02
#define FIFOTHR  0x03
#define SYNC1    0x04
#define SYNC0    0x05
#define PKTLEN   0x06
#define PKTCTRL1 0x07
#define PKTCTRL0 0x08
#define ADDR     0x09
#define CHANNR   0x0A
#define FSCTRL1  0x0B
#define FSCTRL0  0x0C
#define FREQ2    0x0D
#define FREQ1    0x0E
#define FREQ0    0x0F
#define MDMCFG4  0x10
#define MDMCFG3  0x11
#define MDMCFG2  0x12
#define MDMCFG1  0x13
#define MDMCFG0  0x14
#define DEVIATN  0x15
#define MCSM2    0x16
#define MCSM1    0x17
#define MCSM0    0x18
#define FOCCFG   0x19
#define BSCFG    0x1A
#define AGCCTRL2 0x1B
#define AGCCTRL1 0x1C
#define AGCCTRL0 0x1D
#define WOREVT1  0x1E
#define WOREVT0  0x1F
#define WORCTRL  0x20
#define FREND1   0x21
#define FREND0   0x22
#define FSCAL3   0x23
#define FSCAL2   0x24
#define FSCAL1   0x25
#define FSCAL0   0x26
#define RCCTRL1  0x27
#define RCCTRL0  0x28
#define FSTEST   0x29
#define PTEST    0x2A
#define AGCTEST  0x2B
#define TEST2    0x2C
#define TEST1    0x2D
#define TEST0    0x2E

#define PA_TABLE0  0x3E

// FIFO
#define CC25XX_FIFO     0x3F

enum {
    CC2500_00_IOCFG2 = 0x00,   // GDO2 output pin configuration
    CC2500_01_IOCFG1 = 0x01,   // GDO1 output pin configuration
    CC2500_02_IOCFG0 = 0x02,   // GDO0 output pin configuration
    CC2500_03_FIFOTHR = 0x03,  // RX FIFO and TX FIFO thresholds
    CC2500_04_SYNC1 = 0x04,    // Sync word, high byte
    CC2500_05_SYNC0 = 0x05,    // Sync word, low byte
    CC2500_06_PKTLEN = 0x06,   // Packet length
    CC2500_07_PKTCTRL1 = 0x07, // Packet automation control
    CC2500_08_PKTCTRL0 = 0x08, // Packet automation control
    CC2500_09_ADDR = 0x09,     // Device address
    CC2500_0A_CHANNR = 0x0A,   // Channel number
    CC2500_0B_FSCTRL1 = 0x0B,  // Frequency synthesizer control
    CC2500_0C_FSCTRL0 = 0x0C,  // Frequency synthesizer control
    CC2500_0D_FREQ2 = 0x0D,    // Frequency control word, high byte
    CC2500_0E_FREQ1 = 0x0E,    // Frequency control word, middle byte
    CC2500_0F_FREQ0 = 0x0F,    // Frequency control word, low byte
    CC2500_10_MDMCFG4 = 0x10,  // Modem configuration
    CC2500_11_MDMCFG3 = 0x11,  // Modem configuration
    CC2500_12_MDMCFG2 = 0x12,  // Modem configuration
    CC2500_13_MDMCFG1 = 0x13,  // Modem configuration
    CC2500_14_MDMCFG0 = 0x14,  // Modem configuration
    CC2500_15_DEVIATN = 0x15,  // Modem deviation setting
    CC2500_16_MCSM2 = 0x16,    // Main Radio Cntrl State Machine config
    CC2500_17_MCSM1 = 0x17,    // Main Radio Cntrl State Machine config
    CC2500_18_MCSM0 = 0x18,    // Main Radio Cntrl State Machine config
    CC2500_19_FOCCFG = 0x19,   // Frequency Offset Compensation config
    CC2500_1A_BSCFG = 0x1A,    // Bit Synchronization configuration
    CC2500_1B_AGCCTRL2 = 0x1B, // AGC control
    CC2500_1C_AGCCTRL1 = 0x1C, // AGC control
    CC2500_1D_AGCCTRL0 = 0x1D, // AGC control
    CC2500_1E_WOREVT1 = 0x1E,  // High byte Event 0 timeout
    CC2500_1F_WOREVT0 = 0x1F,  // Low byte Event 0 timeout
    CC2500_20_WORCTRL = 0x20,  // Wake On Radio control
    CC2500_21_FREND1 = 0x21,   // Front end RX configuration
    CC2500_22_FREND0 = 0x22,   // Front end TX configuration
    CC2500_23_FSCAL3 = 0x23,   // Frequency synthesizer calibration
    CC2500_24_FSCAL2 = 0x24,   // Frequency synthesizer calibration
    CC2500_25_FSCAL1 = 0x25,   // Frequency synthesizer calibration
    CC2500_26_FSCAL0 = 0x26,   // Frequency synthesizer calibration
    CC2500_27_RCCTRL1 = 0x27,  // RC oscillator configuration
    CC2500_28_RCCTRL0 = 0x28,  // RC oscillator configuration
    CC2500_29_FSTEST = 0x29,   // Frequency synthesizer cal control
    CC2500_2A_PTEST = 0x2A,    // Production test
    CC2500_2B_AGCTEST = 0x2B,  // AGC test
    CC2500_2C_TEST2 = 0x2C,    // Various test settings
    CC2500_2D_TEST1 = 0x2D,    // Various test settings
    CC2500_2E_TEST0 = 0x2E,    // Various test settings

    // Status registers
    CC2500_30_PARTNUM = 0x30,    // Part number
    CC2500_31_VERSION = 0x31,    // Current version number
    CC2500_32_FREQEST = 0x32,    // Frequency offset estimate
    CC2500_33_LQI = 0x33,        // Demodulator estimate for link quality
    CC2500_34_RSSI = 0x34,       // Received signal strength indication
    CC2500_35_MARCSTATE = 0x35,  // Control state machine state
    CC2500_36_WORTIME1 = 0x36,   // High byte of WOR timer
    CC2500_37_WORTIME0 = 0x37,   // Low byte of WOR timer
    CC2500_38_PKTSTATUS = 0x38,  // Current GDOx status and packet status
    CC2500_39_VCO_VC_DAC = 0x39, // Current setting from PLL cal module
    CC2500_3A_TXBYTES = 0x3A,    // Underflow and # of bytes in TXFIFO
    CC2500_3B_RXBYTES = 0x3B,    // Overflow and # of bytes in RXFIFO

    // Multi byte memory locations
    CC2500_3E_PATABLE = 0x3E,
    CC2500_3F_TXFIFO = 0x3F,
    CC2500_3F_RXFIFO = 0x3F
};

// Definitions for burst/single access to registers
#define CC2500_WRITE_SINGLE 0x00
#define CC2500_WRITE_BURST 0x40
#define CC2500_READ_SINGLE 0x80
#define CC2500_READ_BURST 0xC0

// Strobe commands
#define CC2500_SRES 0x30 // Reset chip.
#define CC2500_SFSTXON                                                         \
    0x31 // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
         // If in RX/TX: Go to a wait state where only the synthesizer is
         // running (for quick RX / TX turnaround).
#define CC2500_SXOFF 0x32 // Turn off crystal oscillator.
#define CC2500_SCAL 0x33  // Calibrate frequency synthesizer and turn it off
                          // (enables quick start).
#define CC2500_SRX                                                             \
    0x34 // Enable RX. Perform calibration first if coming from IDLE and
         // MCSM0.FS_AUTOCAL=1.
#define CC2500_STX                                                             \
    0x35 // In IDLE state: Enable TX. Perform calibration first if
         // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
         // Only go to TX if channel is clear.
#define CC2500_SIDLE                                                           \
    0x36 // Exit RX / TX, turn off frequency synthesizer and exit
         // Wake-On-Radio mode if applicable.
#define CC2500_SAFC 0x37 // Perform AFC adjustment of the frequency synthesizer
#define CC2500_SWOR 0x38 // Start automatic RX polling sequence (Wake-on-Radio)
#define CC2500_SPWD 0x39 // Enter power down mode when CSn goes high.
#define CC2500_SFRX 0x3A // Flush the RX FIFO buffer.
#define CC2500_SFTX 0x3B // Flush the TX FIFO buffer.
#define CC2500_SWORRST 0x3C // Reset real time clock.
#define CC2500_SNOP                                                            \
    0x3D // No operation. May be used to pad strobe commands to two
         // bytes for simpler software.
//----------------------------------------------------------------------------------
// Chip Status Byte
//----------------------------------------------------------------------------------

// Bit fields in the chip status byte
#define CC2500_STATUS_CHIP_RDYn_BM 0x80
#define CC2500_STATUS_STATE_BM 0x70
#define CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM 0x0F

// Chip states
#define CC2500_STATE_IDLE 0x00
#define CC2500_STATE_RX 0x10
#define CC2500_STATE_TX 0x20
#define CC2500_STATE_FSTXON 0x30
#define CC2500_STATE_CALIBRATE 0x40
#define CC2500_STATE_SETTLING 0x50
#define CC2500_STATE_RX_OVERFLOW 0x60
#define CC2500_STATE_TX_UNDERFLOW 0x70

//----------------------------------------------------------------------------------
// Other register bit fields
//----------------------------------------------------------------------------------
#define CC2500_LQI_CRC_OK_BM 0x80
#define CC2500_LQI_EST_BM 0x7F

#endif  // HAL_CC25XX_H_

