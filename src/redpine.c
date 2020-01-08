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

#include "main.h"
#include <string.h>
#include <stdio.h>
#include "redpine.h"
#include "telemetry.h"
#include "debug.h"
#include "timeout.h"
#include "led.h"
#include "delay.h"
#include "wdt.h"
#include "cc25xx.h"
#include "io.h"
#include "storage.h"
#include "adc.h"
#include "ppm.h"
#include "failsafe.h"
#include "sbus.h"

// this will make binding not very reliable, use for debugging only!
#define REDPINE_DEBUG_BIND_DATA 0
#define REDPINE_DEBUG_HOPTABLE 1

//Functions to look more like betaflight
#define cc2500Strobe(val) hal_cc25xx_strobe(val);

// hop data & config
EXTERNAL_MEMORY uint8_t redpine_current_ch_idx;

// diversity counter
EXTERNAL_MEMORY uint8_t redpine_diversity_count;

// rssi
EXTERNAL_MEMORY uint8_t rssi;
EXTERNAL_MEMORY uint8_t redpine_link_quality;

// pll calibration
EXTERNAL_MEMORY uint8_t redpine_calib_fscal1_table[REDPINE_HOPTABLE_SIZE];
EXTERNAL_MEMORY uint8_t redpine_calib_fscal2;
EXTERNAL_MEMORY uint8_t redpine_calib_fscal3;

// rf rxtx buffer
EXTERNAL_MEMORY volatile uint8_t packet[REDPINE_PACKET_BUFFER_SIZE];
EXTERNAL_MEMORY volatile uint8_t redpine_packet_received;

#define DEFAULT_PACKET_TIME_US 50000
#define SCALE_REDPINE(channelValue) ((2 * channelValue + 2452) / 3)
#define SCALE_REDPINE_TO_SBUS(channelValue) ((321 * channelValue)/200 - 1418)
#define SCALE_ALL(channelValue) (((107 * channelValue - 10618)/100))
#define CHANNEL_START 3
#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
EXTERNAL_MEMORY int32_t looptime = DEFAULT_PACKET_TIME_US;


void redpine_init(void) {
    debug("redpine: init\n"); debug_flush();

    cc25xx_init();

    redpine_link_quality = 0;
    redpine_diversity_count = 0;
    redpine_packet_received = 0;

    rssi = 100;

    // check if spi is working properly
    redpine_show_partinfo();

    // init redpine registersttings for cc2500
    redpine_configure();

    if (redpine_bind_jumper_set()) {
        // do binding
        redpine_do_bind();
        // binding will never return/continue
    }

    // show info:
    debug("redpine: using txid 0x"); debug_flush();
    debug_put_hex8(storage.txid[0]);
    debug_put_hex8(storage.txid[1]);
    debug_put_newline();

    // init txid matching
    //redpine_configure_address();

    // tune cc2500 pll and save the values to ram
    redpine_calib_pll();

    debug("redpine: init done\n"); debug_flush();
}


void redpine_show_partinfo(void) {
    uint8_t partnum, version;
    // start idle
    cc25xx_strobe(RFST_SIDLE);

    // check version:
    debug("redpine: cc25xx partnum 0x");
    partnum = cc25xx_get_register_burst(PARTNUM);
    debug_put_hex8(partnum);

    debug(" version 0x");
    version = cc25xx_get_register_burst(VERSION);
    debug_put_hex8(version);
    debug_put_newline();

    if (cc25xx_partnum_valid(partnum, version)) {
        debug("redpine: got valid part and version info\n");
    } else {
        debug("redpine: got INVALID part and version info?!\n");
    }
    debug_flush();
}



void redpine_configure(void) {
    debug("redpine: configure\n"); debug_flush();

    cc25xx_strobe(0x30);
    delay_ms(1);

    // IOCFG0,1,2 is set in hal code (it is specific to the board used)
    cc25xx_set_gdo_mode();

    cc2500WriteReg(CC2500_03_FIFOTHR,  0x07);       
    cc2500WriteReg(CC2500_06_PKTLEN,   REDPINE_PACKET_LENGTH);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x0C);
    cc2500WriteReg(CC2500_08_PKTCTRL0, 0x05);
    cc2500WriteReg(CC2500_09_ADDR,     0x00); 
    cc2500WriteReg(CC2500_0B_FSCTRL1,  0x0A);
    cc2500WriteReg(CC2500_0C_FSCTRL0,  0x00);        
    cc2500WriteReg(CC2500_0D_FREQ2,    0x5D);
    cc2500WriteReg(CC2500_0E_FREQ1,    0x93);
    cc2500WriteReg(CC2500_0F_FREQ0,    0xB1);        
    cc2500WriteReg(CC2500_10_MDMCFG4,  0x2D);
    cc2500WriteReg(CC2500_11_MDMCFG3,  0x3B);
    cc2500WriteReg(CC2500_12_MDMCFG2,  0x73);
    cc2500WriteReg(CC2500_13_MDMCFG1,  0x23);
    cc2500WriteReg(CC2500_14_MDMCFG0,  0x56);        
    cc2500WriteReg(CC2500_15_DEVIATN,  0x00); 
    cc2500WriteReg(CC2500_17_MCSM1,    0x0C);
    cc2500WriteReg(CC2500_18_MCSM0,    0x18);     
    cc2500WriteReg(CC2500_19_FOCCFG,   0x1D);
    cc2500WriteReg(CC2500_1A_BSCFG,    0x1C);                
    cc2500WriteReg(CC2500_1B_AGCCTRL2, 0xC7);
    cc2500WriteReg(CC2500_1C_AGCCTRL1, 0x00);
    cc2500WriteReg(CC2500_1D_AGCCTRL0, 0xB0);   
    cc2500WriteReg(CC2500_21_FREND1,   0xB6);   
    cc2500WriteReg(CC2500_22_FREND0,   0x10);
    cc2500WriteReg(CC2500_23_FSCAL3,   0xA9);
    cc2500WriteReg(CC2500_24_FSCAL2,   0x0A);
    cc2500WriteReg(CC2500_25_FSCAL1,   0x00);
    cc2500WriteReg(CC2500_26_FSCAL0,   0x11);        
    cc2500WriteReg(CC2500_29_FSTEST,   0x59);                  
    cc2500WriteReg(CC2500_2C_TEST2,    0x88);
    cc2500WriteReg(CC2500_2D_TEST1,    0x31);
    cc2500WriteReg(CC2500_2E_TEST0,    0x0B);
    cc2500WriteReg(CC2500_3E_PATABLE,  0xFF); 

    debug("redpine: configure done\n"); debug_flush();
}

uint8_t redpine_bind_jumper_set(void) {
    debug("redpine: BIND jumper set = "); debug_flush();
    if (io_bind_request()) {
        debug("YES -> binding\n");
        return 1;
    } else {
        debug("NO -> no binding\n");
        return 0;
    }
}

void redpine_do_bind(void) {
    debug("redpine: do bind\n"); debug_flush();

    // set txid to bind channel
    storage.txid[0] = 0x03;

    // frequency offset to zero (will do auto tune later on)
    storage.freq_offset = 0;

    // init txid matching
    //redpine_configure_address();

    // set up leds:txid
    led_red_off();
    led_green_on();

    // start autotune:
    redpine_autotune();

    // now run the actual binding:
    redpine_fetch_txid_and_hoptable();

    // important: stop RF interrupts:
    cc25xx_disable_rf_interrupt();

    // save to persistant storage:
    storage_write_to_flash();

    // done, end up in fancy blink code
    debug("redpine: finished binding. please reset\n");
    led_green_on();

    while (1) {
        led_red_on();
        delay_ms(500);
        wdt_reset();

        led_red_off();
        delay_ms(500);
        wdt_reset();
    }
}


void redpine_autotune(void) {
    uint8_t done = 0;
    uint8_t received_packet = 0;
    uint8_t state = 0;
    int8_t fscal0_min = 127;
    int8_t fscal0_max = -127;
    int16_t fscal0_calc;

    debug("redpine: autotune\n"); debug_flush();

    // find best offset:
    storage.freq_offset = 0;

    cc2500WriteReg(CC2500_19_FOCCFG, 0x14);

    cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)storage.freq_offset);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x0C);
    cc2500WriteReg(CC2500_18_MCSM0, 0x8);

    // enter RX mode
    redpine_enter_rxmode(0);
        

    debug("redpine: entering bind loop\n"); debug_flush();

    led_red_off();

    // search for best fscal 0 match
    while (state != 5) {
        // reset wdt
        wdt_reset();

        // handle any ovf conditions
        redpine_handle_overflows();

        // search full range quickly using binary search
        switch (state) {
        default:
        case(0):
            // init left search:
            storage.freq_offset = -127;
            state = 1;
            break;

        case(1):
            // first search quickly through the full range:
            if (storage.freq_offset < 127-10) {
                storage.freq_offset += 9;
            } else {
                // done one search, did we receive anything?
                if (received_packet) {
                    // finished, go to slow search
                    storage.freq_offset = fscal0_min - 9;
                    state = 2;
                } else {
                    // no success, lets try again
                    state = 0;
                }
            }
            break;

        case(2):
            if (storage.freq_offset < fscal0_max+9) {
                storage.freq_offset++;
            } else {
                // done!
                state = 5;
            }
            break;
        }

        // go to idle
        cc25xx_strobe(RFST_SIDLE);

        // set freq offset
        cc25xx_set_register(FSCTRL0, storage.freq_offset);

        led_red_off();

        // go back to RX:
        delay_ms(1);
        cc25xx_strobe(RFST_SRX);

        // set timeout
        timeout_set(50);
        done = 0;

        led_green_on();
        led_red_off();

        //debug("tune "); debug_put_int8(storage.freq_offset);
        //debug_put_newline(); debug_flush();

        while ((!timeout_timed_out()) && (!done)) {
            // handle any ovf conditions
            redpine_handle_overflows();

            cc25xx_process_packet(&redpine_packet_received,
                                  (volatile uint8_t *)&packet,
                                  REDPINE_PACKET_BUFFER_SIZE);

            if (redpine_packet_received) {
                // prepare for next packet:
                redpine_packet_received = 0;
                cc25xx_enable_receive();
                cc25xx_strobe(RFST_SRX);

                // valid packet?
                if (REDPINE_VALID_PACKET_BIND(packet)) {
                    // bind packet!
                    debug_putc('B');

                    // packet received
                    received_packet = 1;

                    // this fscal value is done
                    done = 1;

                    // update min/max
                    fscal0_min = min(fscal0_min, storage.freq_offset);
                    fscal0_max = max(fscal0_max, storage.freq_offset);

                    // make sure we never read the same packet twice by invalidating packet
                    packet[0] = 0x00;
                }
/*
                debug("[");debug_flush();
                uint8_t cnt;
                for (cnt=0; cnt<REDPINE_PACKET_BUFFER_SIZE; cnt++) {
                    debug_put_hex8(packet[cnt]);
                    debug_putc(' ');
                    debug_flush();
                }
                debug("]\n"); debug_flush();
                */
            }
        }
        if (!done) {
            debug_putc('-');
            debug_flush();               
        }
    }

    // set offset to what we found out to be the best:
    fscal0_calc = (fscal0_max + fscal0_min)/2;

    debug("\nredpine: fscal0 ");
    debug_put_int8(fscal0_min);
    debug(" - ");
    debug_put_int8(fscal0_max);
    debug_put_newline();
    debug_flush();

    // store new value
    storage.freq_offset = fscal0_calc;

    cc25xx_strobe(RFST_SIDLE);

    // set freq offset
    cc25xx_set_register(FSCTRL0, storage.freq_offset);

    // go back to RX:
    delay_ms(1);
    cc25xx_strobe(RFST_SRX);

    debug("redpine: autotune done. offset=");
    debug_put_int8(storage.freq_offset);
    debug_put_newline();
    debug_flush();
}


void redpine_enter_rxmode(uint8_t channel) {
    cc25xx_strobe(RFST_SIDLE);

    cc25xx_enter_rxmode();

    // set & do a manual tuning for the given channel
    redpine_tune_channel(channel);

    cc25xx_enable_receive();

    // go back to rx mode
    cc25xx_strobe(RFST_SRX);
}



void redpine_tune_channel(uint8_t ch) {
    // start idle
    cc25xx_strobe(RFST_SIDLE);

    // set channel number
    cc25xx_set_register(CHANNR, ch);

    // start Self calib:
    cc25xx_strobe(RFST_SCAL);

    // wait for scal end
    // either delay_us(800) or check MARCSTATE:
    while (cc25xx_get_register(MARCSTATE) != 0x01) {}

    // now FSCAL3..1 shold be set up correctly! yay!
}

void redpine_handle_overflows(void) {
    uint8_t marc_state;

    // fetch marc status
    marc_state = cc25xx_get_register(MARCSTATE) & 0x1F;
    if (marc_state == 0x11) {
        debug("redpine: RXOVF\n");
        // flush rx buf
        cc25xx_strobe(RFST_SFRX);
        debug_put_int8(cc25xx_get_register(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F);
        debug_put_newline(); debug_flush();        
        // cc25xx_strobe(RFST_SIDLE);
    } else if (marc_state == 0x16) {
        debug("redpine: TXOVF\n");
        // flush tx buf
        cc25xx_strobe(RFST_SFTX);
        // cc25xx_strobe(RFST_SIDLE);
    }
}


void redpine_fetch_txid_and_hoptable(void) {
    uint16_t hopdata_received = 0;
    uint8_t index;
    uint8_t i;

    // enter RX mode
    redpine_enter_rxmode(0);

#define MAX_BIND_PACKET_COUNT 10
    // DONE when n times a one:
#define HOPDATA_RECEIVE_DONE ((1 << (MAX_BIND_PACKET_COUNT))-1)

    // clear txid:
    storage.txid[0] = 0;
    storage.txid[1] = 0;

    // timeout to wait for packets
    timeout_set(9*3+1);

    // fetch hopdata array
    while (hopdata_received != HOPDATA_RECEIVE_DONE) {
        // reset wdt
        wdt_reset();

        // handle any ovf conditions
        redpine_handle_overflows();

        // FIXME: this should be handled in a cleaner way.
        // as this is just for binding, stay with this fix for now...
        if (timeout_timed_out()) {
            // do diversity
            cc25xx_switch_antenna();

            debug_putc('m');

            // next packet should be there ein 9ms
            // if no packet for 3*9ms -> reset rx chain:
            timeout_set(3*9+1);

            // re-prepare for next packet:
            cc25xx_strobe(RFST_SIDLE);
            // TESTME: moved to rx_sleep....
            // delay_ms(1);
            redpine_packet_received = 0;
            cc25xx_rx_sleep();
            cc25xx_enable_receive();
            cc25xx_strobe(RFST_SRX);
        }

        // process incoming data
        cc25xx_process_packet(&redpine_packet_received,
                              (volatile uint8_t *)&packet,
                              REDPINE_PACKET_BUFFER_SIZE);

        if (redpine_packet_received) {
            debug_putc('p');

            // prepare for next packet:
            redpine_packet_received = 0;
            cc25xx_enable_receive();
            cc25xx_strobe(RFST_SRX);


#if REDPINE_DEBUG_BIND_DATA
            if (REDPINE_VALID_FRAMELENGTH(packet)) {
                debug("redpine: RX ");
                debug_flush();
                for (i = 0; i < REDPINE_PACKET_BUFFER_SIZE; i++) {
                    debug_put_hex8(packet[i]);
                    debug_putc(' ');
                }
                debug_put_newline();
            }
#endif  // REDPINE_DEBUG_BIND_DATA


            // do we know our txid yet?
            if (REDPINE_VALID_PACKET_BIND(packet)) {
                // next packet should be ther ein 9ms
                // if no packet for 3*9ms -> reset rx chain:
                timeout_set(3*9+1);

                debug_putc('B');
                if ((storage.txid[0] == 0) && (storage.txid[1] == 0)) {
                    // no! extract this
                    storage.txid[0] = packet[3];
                    storage.txid[1] = packet[4];
                    // debug
                    debug("redpine: got txid 0x");
                    debug_put_hex8(storage.txid[0]);
                    debug_put_hex8(storage.txid[1]);
                    debug_put_newline();
                }

                // this is actually for us
                index = packet[5];

                // valid bind index?
                if (index/5 < MAX_BIND_PACKET_COUNT) {
                    // copy data to our hop list:
                    for (i = 0; i < 5; i++) {
                        if ((index+i) < REDPINE_HOPTABLE_SIZE) {
                            storage.hop_table[index+i] = packet[6+i];
                        }
                    }
                    // mark as done: set bit flag for index
                    hopdata_received |= (1 << (index / 5));
                } else {
                    debug("redpine: invalid bind idx");
                    debug_put_uint8(index/5);
                    debug_put_newline();
                }

                // make sure we never read the same packet twice by crc flag
                packet[REDPINE_PACKET_BUFFER_SIZE-1] = 0x00;
            }
        }
    }

#if REDPINE_DEBUG_BIND_DATA
    debug("redpine: hop[] = ");
    for (i = 0; i < REDPINE_HOPTABLE_SIZE; i++) {
        debug_put_hex8(storage.hop_table[i]);
        debug_putc(' ');
        debug_flush();
    }
    debug_putc('\n');
#endif  // REDPINE_DEBUG_BIND_DATA

    // idle
    cc25xx_strobe(RFST_SIDLE);
}

void redpine_calib_pll(void) {
    uint8_t i;
    uint8_t ch;

    debug("redpine: calib pll\n");

    // fine tune offset
    cc25xx_set_register(FSCTRL0, storage.freq_offset);

    debug("redpine: tuning hop[] =");

    // calibrate pll for all channels
    for (i = 0; i < REDPINE_HOPTABLE_SIZE; i++) {
        // reset wdt
        wdt_reset();

        // fetch channel from hop_table:
        ch = storage.hop_table[i];

        // debug info
        debug_putc(' ');
        debug_put_hex8(ch);

        // set channel number
        redpine_tune_channel(ch);

        // store pll calibration:
        redpine_calib_fscal1_table[i] = cc25xx_get_register(FSCAL1);
    }
    debug_put_newline();

    // only needed once:
    redpine_calib_fscal3 = cc25xx_get_register(FSCAL3);
    redpine_calib_fscal2 = cc25xx_get_register(FSCAL2);

    // return to idle
    cc25xx_strobe(RFST_SIDLE);

    debug("redpine: calib fscal0 = ");
    debug_put_int8(storage.freq_offset);
    debug("\nredpine: calib fscal1 = ");
    for (i = 0; i < REDPINE_HOPTABLE_SIZE; i++) {
        debug_put_hex8(redpine_calib_fscal1_table[i]);
        debug_putc(' ');
        debug_flush();
    }
    debug("\nredpine: calib fscal2 = 0x");
    debug_put_hex8(redpine_calib_fscal2);
    debug("\nredpine: calib fscal3 = 0x");
    debug_put_hex8(redpine_calib_fscal3);
    debug_put_newline();
    debug_flush();

    debug("redpine: calib pll done\n");
}

void redpine_main(void) {
    uint8_t send_telemetry = 0;
    uint8_t missing = 0;
    uint16_t hopcount = 0;
    uint16_t stat_rxcount = 0;
    // uint8_t badrx_test = 0;
    uint8_t conn_lost = 1;
    uint8_t packet_received = 0;
    // uint8_t i;

    debug("redpine: starting main loop\n");

    // start with any channel:
    redpine_current_ch_idx = 0;

    // first set channel uses enter rxmode, this will set up dma etc
    redpine_enter_rxmode(storage.hop_table[redpine_current_ch_idx]);
    cc25xx_strobe(RFST_SRX);  // D4R-II addition!

    // wait 500ms on the current ch on powerup
    timeout_set(500);

    // start with conn lost (allow full sync)
    conn_lost = 1;

    // reset wdt once in order to have at least one second waiting for a packet:
    wdt_reset();

    // start main loop
    while (1) {
        if (timeout_timed_out()) {
            // next hop in 9ms
            if (!conn_lost) {
                if (packet_received) {
                    
                    timeout_set_100us(looptime+looptime/8); //Add 1/8 looptime jitter for packets
                } else {
                    timeout_set_100us(looptime); //If you missed the last packet don't add the jitter
                }
            } else {
                timeout_set_100us(5000);
            }

            // wdt reset
            wdt_reset();

            redpine_increment_channel(1);

            // diversity toggle on missing frame
            if (!packet_received) {
                led_red_on();
                cc25xx_switch_antenna();
            }

            // go back to rx mode
            cc25xx_enable_receive();
            // cc25xx_enter_rxmode(); THIS BREAKS VD5M!
            cc25xx_strobe(RFST_SRX);

            // if enabled, send a sbus frame in case we lost that frame:
            if (!packet_received) {
                // frame was lost, so there was no channel value update
                // and no transmission for the last frame slot.
                // therefore we will do a transmission now
                // (frame lost packet flag will be set)
                sbus_start_transmission(SBUS_FRAME_LOST);
            }

            if (send_telemetry) {
                // last packet was outbound telemetry, send last rx packet on sbus
                sbus_start_transmission(SBUS_FRAME_NOT_LOST);
                send_telemetry = 0;
            }

            // check for packets
            if (packet_received > 0) {
                debug_putc('0' + cc25xx_get_current_antenna());
            } else {
                debug_putc('!');
                missing++;
            }
            packet_received = 0;

            if (hopcount++ >= REDPINE_COUNT_RXSTATS) {
                uint16_t percent_ok;
                debug(" STATS: ");
                percent_ok = (((uint32_t)stat_rxcount) * 100) / REDPINE_COUNT_RXSTATS;
                debug_put_uint8(percent_ok);
                debug_putc('%');
                debug_put_newline();

                // for testing
                // debug_put_uint16((uint16_t)(((
                //                  packet[11] & 0x0F) << 8 |
                //                  packet[8])));
                // debug_put_uint16(rssi);
                // debug_putc(' ');

                // link quality
                redpine_link_quality = stat_rxcount;

                if (stat_rxcount == 0) {
                    conn_lost = 1;
                    // enter failsafe mode
                    failsafe_enter();
                    debug("\nCONN LOST!\n");
                }

                // statistics
                hopcount = 1;
                stat_rxcount = 0;
            }

            // led_red_off();

            // handle ovfs
            redpine_handle_overflows();
        }

        // process incoming data
        cc25xx_process_packet(&redpine_packet_received,
                              (volatile uint8_t *)&packet,
                              REDPINE_PACKET_BUFFER_SIZE);

        if (redpine_packet_received) {
            led_red_off();
            redpine_packet_received = 0;

            // valid packet?
            if (REDPINE_VALID_PACKET(packet)) {
                // ok, valid packet for us
                led_green_on();
                looptime = packet[CHANNEL_START + 7];

                // we hop to the next channel in 0.5ms
                // afterwards hops are in 9ms grid again
                // this way we can have up to +/-1ms jitter on our 9ms timebase
                // without missing packets
                // 500us delay:
                
                timeout_set_100us(0);

                // reset wdt
                wdt_reset();

                // reset missing packet counter
                missing = 0;

                // stats
                stat_rxcount++;
                packet_received = 1;
                conn_lost = 0;

                // extract rssi in redpine format
                rssi = redpine_extract_rssi(packet[REDPINE_PACKET_BUFFER_SIZE-2]);

                // extract channel data:
                redpine_update_ppm();

                led_green_off();
            }
        } else {
            // invalid packet -> mark as not received
            packet_received = 0;
        }
    }
}



void redpine_set_channel(uint8_t hop_index) {
    uint8_t ch = storage.hop_table[hop_index];
    // debug_putc('S'); debug_put_hex8(ch);

    // go to idle
    cc25xx_strobe(RFST_SIDLE);

    // fetch and set our stored pll calib data:
    cc25xx_set_register(FSCAL3, redpine_calib_fscal3);
    cc25xx_set_register(FSCAL2, redpine_calib_fscal2);
    cc25xx_set_register(FSCAL1, redpine_calib_fscal1_table[hop_index]);

    // set channel
    cc25xx_set_register(CHANNR, ch);
}



void redpine_increment_channel(int8_t cnt) {
    int8_t next = redpine_current_ch_idx;
    // add increment
    next+=cnt;
    // convert to a safe unsigned number:
    if (next < 0) {
        next += REDPINE_HOPTABLE_SIZE;
    }
    if (next >= REDPINE_HOPTABLE_SIZE) {
        next -= REDPINE_HOPTABLE_SIZE;
    }

    redpine_current_ch_idx = next;
    redpine_set_channel(redpine_current_ch_idx);
}


uint8_t redpine_extract_rssi(uint8_t rssi_raw) {
#define REDPINE_RSSI_OFFSET 70
    if (rssi_raw >= 128) {
        // adapted to fit better to the original values... FIXME: find real formula
        // return (rssi_raw * 18)/32 - 82;
        return ((((uint16_t)rssi_raw) * 18)>>5) - 82;
    } else {
        return ((((uint16_t)rssi_raw) * 18)>>5) + 65;
    }
}


void redpine_update_ppm(void) {
    // build uint16_t array from data:
    EXTERNAL_MEMORY uint16_t channel_data[8];

    /*debug("[");debug_flush();
    for (cnt=0; cnt<REDPINE_PACKET_BUFFER_SIZE; cnt++) {
        debug_put_hex8(packet[cnt]);
        debug_putc(' ');
        debug_flush();
    }
    debug("]\n"); debug_flush();
    */
    uint16_t channelValue;



    // extract channel data from packet:
    channelValue = (uint16_t)((packet[CHANNEL_START+1] << 8) & 0x700) | packet[CHANNEL_START];
    channel_data[0] = SCALE_ALL(channelValue);

    channelValue = (uint16_t)((packet[CHANNEL_START+2] << 4) & 0x7F0) | ((packet[CHANNEL_START+1] >> 4) & 0xF);
    channel_data[1] = SCALE_ALL(channelValue);
    
    channelValue = (uint16_t)((packet[CHANNEL_START+4] << 8) & 0x700) | packet[CHANNEL_START+3];
    channel_data[2] = SCALE_ALL(channelValue);
    
    channelValue = (uint16_t)((packet[CHANNEL_START+5] << 4) & 0x7F0) | ((packet[CHANNEL_START+4] >> 4) & 0xF);
    channel_data[3] = SCALE_ALL(channelValue);

    //12 1-bit aux channels - first 4 are interleaved with stick channels
    channel_data[4]= (packet[CHANNEL_START + 1] & 0x08) ? 1792  : 192 ; 
    channel_data[5]= (packet[CHANNEL_START + 2] & 0x80) ? 1792  : 192 ; 
    channel_data[6]= (packet[CHANNEL_START + 4] & 0x08) ? 1792  : 192 ; 
    channel_data[7]= (packet[CHANNEL_START + 5] & 0x80) ? 1792  : 192 ; 


    // exit failsafe mode
    failsafe_exit();

    // copy to output modules:
    sbus_update(channel_data);
    sbus_start_transmission(SBUS_FRAME_NOT_LOST);
    // and to ppm
    ppm_update(channel_data);
}


// useful for debugging/sniffing packets from anothe tx or rx
// make sure to bind this rx before using this...
void redpine_frame_sniffer(void) {
    uint8_t send_telemetry = 0;
    uint8_t missing = 0;
    uint8_t hopcount = 0;
    uint8_t stat_rxcount = 0;
    uint8_t badrx_test = 0;
    uint8_t conn_lost = 1;
    uint8_t packet_received = 0;
    uint8_t i;

    debug("redpine: entering sniffer mode\n");

    // start with any channel:
    redpine_current_ch_idx = 0;
    // first set channel uses enter rxmode, this will set up dma etc
    redpine_enter_rxmode(storage.hop_table[redpine_current_ch_idx]);

    // wait 500ms on the current ch on powerup
    timeout_set(500);

    // start with conn lost (allow full sync)
    conn_lost = 1;
    redpine_set_channel(0);

    // reset wdt once in order to have at least one second waiting for a packet:
    wdt_reset();

    // start main loop
    while (1) {
        wdt_reset();

        if (timeout_timed_out()) {
            led_red_on();

            // next hop in 9ms
            if (!conn_lost) {
                timeout_set(9);
                redpine_increment_channel(1);
            } else {
                timeout_set(500);
                redpine_set_channel(0);
            }


            // strange delay
            // _delay_us(1000);
            cc25xx_rx_sleep();

            // go back to rx mode
            redpine_packet_received = 0;
            cc25xx_enable_receive();

            cc25xx_strobe(RFST_SRX);

            // check for packets
            if (!packet_received) {
                if (send_telemetry) {
                    debug("< MISSING\n");
                    send_telemetry = 0;
                } else {
                    debug("> MISSING\n");
                }
                send_telemetry = 0;
                missing++;
            }
            packet_received = 0;

            if (hopcount++ >= 100) {
                if (stat_rxcount == 0) {
                    conn_lost = 1;
                    debug("\nCONN LOST!\n");
                }

                // statistics
                hopcount = 1;
                stat_rxcount = 0;
            }

            led_red_off();
        }

        // handle ovfs
        redpine_handle_overflows();

        // process incoming data
        cc25xx_process_packet(&redpine_packet_received,
                              (volatile uint8_t *)&packet,
                              REDPINE_PACKET_BUFFER_SIZE);


        if (redpine_packet_received) {
            cc25xx_enable_receive();
            cc25xx_strobe(RFST_SRX);

            if (REDPINE_VALID_PACKET(packet)) {
                // ok, valid packet for us
                led_green_on();

                // we hop to the next channel in 0.5ms
                // afterwards hops are in 9ms grid again
                // this way we can have up to +/-1ms jitter on our 9ms timebase
                // without missing packets
                delay_us(500);
                timeout_set(0);

                // dump all packets!
                if (send_telemetry) {
                    debug("< ");
                    send_telemetry = 0;
                } else {
                    debug("> ");
                }

                for (i = 0; i < REDPINE_PACKET_BUFFER_SIZE; i++) {
                    debug_put_hex8(packet[i]);
                    debug_putc(' ');
                }
                debug("\n");

                // reset wdt
                wdt_reset();

                // reset missing packet counter
                missing = 0;

                // every 4th frame is a telemetry frame (transmits every 36ms)
                if ((packet[3] % 4) == 2) {
                    send_telemetry = 1;
                }

                // stats
                stat_rxcount++;
                packet_received = 1;
                conn_lost = 0;

                // make sure we never read the same packet twice by crc flag
                packet[REDPINE_PACKET_BUFFER_SIZE-1] = 0x00;

                led_green_off();
            } else {
                if (packet[0] != 0) {
                    debug("RX NON REDPINE PACKET: ");
                    for (i = 0; i < REDPINE_PACKET_BUFFER_SIZE; i++) {
                        debug_put_hex8(packet[i]);
                        debug_putc(' ');
                    }
                    debug_put_newline();
                    redpine_packet_received = 0;
                }
            }
        }
    }
}


#if 0




uint8_t redpine_append_hub_data(uint8_t sensor_id, uint16_t value, uint8_t *buf) {
    uint8_t index = 0;
    uint8_t val8;

    // add header:
    buf[index++] = REDPINE_HUB_TELEMETRY_HEADER;
    // add sensor id
    buf[index++] = sensor_id;
    // add data, take care of byte stuffing
    // low byte
    val8 = LO(value);
    if (val8 == 0x5E) {
        buf[index++] = 0x5D;
        buf[index++] = 0x3E;
    } else if (val8 == 0x5D) {
        buf[index++] = 0x5D;
        buf[index++] = 0x3D;
    } else {
        buf[index++] = val8;
    }
    // high byte
    val8 = HI(value);
    if (val8 == 0x5E) {
        buf[index++] = 0x5D;
        buf[index++] = 0x3E;
    } else if (val8 == 0x5D) {
        buf[index++] = 0x5D;
        buf[index++] = 0x3D;
    } else {
        buf[index++] = val8;
    }
    // add footer:
    buf[index] = REDPINE_HUB_TELEMETRY_HEADER;

    // return index:
    return index;
}
#endif  // 0
