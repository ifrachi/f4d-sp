/*
 * Copyright (c) 2025, Professor Menachem Moshelion,
 * Field4D Research Framework, The Hebrew University of Jerusalem, Israel
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
/**
 * \file
 *         Sensors Platform (SP) Version 1 implementation.
 * \author
 *         Idan Ifrach <idan.ifrach@mail.huji.ac.il>
 *
 * \ingroup simplelink sensortag-cc2650
 *
 * \brief
 *         This file contains the implementation of the Sensors Platform (SP) child node.
 *         The SP node is a low-power environmental sensing unit that collects data such as
 *         temperature, humidity and light intensity. It communicates with
 *         the Base Edge Router (BER) using UDP over RPL-TSCH within the Field4D (F4D) Outdoor
 *         Plant Phenotyping iIoT framework.
 * @{
 */

#include "contiki.h"
#include "board-peripherals.h"
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "dev/watchdog.h"
#include "dev/button-hal.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "random.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/*-------------------------------------------------*/
#define LEFT_KEY BOARD_BUTTON_HAL_INDEX_KEY_LEFT
#define RIGHT_KEY BOARD_BUTTON_HAL_INDEX_KEY_RIGHT

/*-------------------------------------------------*/
/* Declare the custom event globally */
static process_event_t cycle_event;
/*-------------------------------------------------*/
#define BMP_COMPLETED  (1 << 0)  // Bit 0
#define OPT_COMPLETED  (1 << 1)  // Bit 1
#define HDC_COMPLETED  (1 << 2)  // Bit 2

static uint8_t sensor_cycle_status = 0;  // Bitmask to track sensor completion
/*-------------------------------------------------*/
#include "sys/log.h"
#define LOG_MODULE "F4D"
#define LOG_LEVEL LOG_LEVEL_INFO
/*-------------------------------------------------*/
#define UDP_SERVER_PORT 1234
#define UDP_CLIENT_PORT 4321
/*-------------------------------------------------*/
// #define RESEND_INTERVAL ( CLOCK_SECOND * 5 )
#define SENSOR_READING_PERIOD (CLOCK_SECOND * 175)
#define SENSOR_READING_RANDOM (CLOCK_SECOND << 4)
/*-------------------------------------------------*/
#define PAYLOAD_BUFFER_SIZE 17  // Adjusted for remaining payload elements
#define BYTES_PER_INT   4          // Number of bytes for each int
/*-------------------------------------------------*/
bool is_registered = false;
static int payload_buffer[PAYLOAD_BUFFER_SIZE];
static uint8_t bytes_payload[PAYLOAD_BUFFER_SIZE * BYTES_PER_INT];
static uint16_t ping[1] = { 16 };
/*-------------------------------------------------*/
static int package_number = 0;
/*-------------------------------------------------*/
uip_ipaddr_t dest_ipaddr;
/*-------------------------------------------------*/
static struct ctimer bmp_timer, opt_timer, batt_timer, hdc_timer;
static struct etimer transmission;
/*-------------------------------------------------*/
static void init_bmp_reading(void* not_used);
static void init_opt_reading(void* not_used);
static void init_batt_reading(void* not_used);
static void init_hdc_reading(void* not_used);
/*-------------------------------------------------*/
static struct simple_udp_connection udp_conn;
/*-------------------------------------------------*/
void int_to_bytes(int value, uint8_t *bytes) {
    // Assuming the system is little-endian. If the system is big-endian, byte order needs to be reversed.
    bytes[0] = (uint8_t)(value & 0xFF);
    bytes[1] = (uint8_t)((value >> 8) & 0xFF);
    bytes[2] = (uint8_t)((value >> 16) & 0xFF);
    bytes[3] = (uint8_t)((value >> 24) & 0xFF);
}
/*-------------------------------------------------*/
void convertPayloadToBytes(int *payload, uint8_t *bytes_payload) {
    int byte_index = 0;
    for(int i = 0; i < PAYLOAD_BUFFER_SIZE; ++i) {
        int_to_bytes(payload[i], &bytes_payload[byte_index]);
        byte_index += BYTES_PER_INT;
    }
}
/*-------------------------------------------------*/
void printPayloadAsJSON(int *payload) {
    printf("\r\n{\r\n");
    printf("    \"opt_data\": [%d, %d],\r\n", payload[0], payload[1]);
    printf("    \"batmon_data\": [%d, %d],\r\n", payload[2], payload[3]);
    printf("    \"bmp_data\": [%d, %d, %d, %d],\r\n", payload[4], payload[5], payload[6], payload[7]);
    printf("    \"hdc_data\": [%d, %d, %d, %d],\r\n", payload[8], payload[9], payload[10], payload[11]);
    printf("    \"package_number\": %d\r\n", payload[12]);
    printf("}\r\n");
}

/*-------------------------------------------------*/
// Helper function to convert 4 bytes back into an int
int bytes_to_int(uint8_t *bytes) {
    return (int)(bytes[0] | bytes[1] << 8 | bytes[2] << 16 | bytes[3] << 24);
}
/*-------------------------------------------------*/
void printBytesPayloadAsJSON(uint8_t *bytes_payload) {
    printf("\r\n{\r\n");
    printf("    \"opt_data\": [%d, %d],\r\n", bytes_to_int(&bytes_payload[0]), bytes_to_int(&bytes_payload[4]));
    printf("    \"batmon_data\": [%d, %d],\r\n", bytes_to_int(&bytes_payload[8]), bytes_to_int(&bytes_payload[12]));
    printf("    \"bmp_data\": [%d, %d, %d, %d],\r\n", bytes_to_int(&bytes_payload[16]), bytes_to_int(&bytes_payload[20]), bytes_to_int(&bytes_payload[24]), bytes_to_int(&bytes_payload[28]));
    printf("    \"hdc_data\": [%d, %d, %d, %d]\r\n", bytes_to_int(&bytes_payload[32]), bytes_to_int(&bytes_payload[36]), bytes_to_int(&bytes_payload[40]), bytes_to_int(&bytes_payload[44]));
    printf("    \"package_number\": %d\r\n", bytes_to_int(&bytes_payload[48]));
    printf("}\r\n");
}
/*-------------------------------------------------*/
static void get_battery_reading() {
    int value;
    value = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);
    payload_buffer[2] = value;
    value = batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);
    payload_buffer[3] = (value * 125) >> 5;
}


/*-------------------------------------------------*/
static void get_hdc_reading()
{
    int value;
     clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);
        value = hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_TEMP);
        if (value != CC26XX_SENSOR_READING_ERROR)
        {
              payload_buffer[8] = value / 100;
              payload_buffer[9] = value % 100;
        }
        else {
            payload_buffer[8] = 999;
            payload_buffer[9] = 999;
        }
        value = hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_HUMID);
        if (value != CC26XX_SENSOR_READING_ERROR)
        {
            payload_buffer[10] = value / 100;
            payload_buffer[11] = value % 100;
        }
        else {
            payload_buffer[10] = 999;
            payload_buffer[11] = 999;
        }
    ctimer_set(&hdc_timer, next, init_hdc_reading, NULL);
}
/*-------------------------------------------------*/
static void get_bmp_reading()
{
    int value;
         clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);
        value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_PRESS);
        if (value != CC26XX_SENSOR_READING_ERROR)
        {
              payload_buffer[4] = value / 100;
              payload_buffer[5] = value % 100;
        }
        else
        {
            payload_buffer[4] = 999;
            payload_buffer[5] = 999;
        }
        value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_TEMP);
        if (value != CC26XX_SENSOR_READING_ERROR)
        {
              payload_buffer[6] = value / 100;
              payload_buffer[7] = value % 100;
        }
        else
        {
            payload_buffer[6] = 999;
            payload_buffer[7] = 999;
        }
    SENSORS_DEACTIVATE(bmp_280_sensor);
    ctimer_set(&bmp_timer, next, init_bmp_reading, NULL);
}
/*-------------------------------------------------*/
static void get_light_reading()
{
    int value;
         clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);
        value = opt_3001_sensor.value(0);
        if (value != CC26XX_SENSOR_READING_ERROR)
        {
              payload_buffer[0] = value / 100;
              payload_buffer[1] = value % 100;
        }
        else
        {
              payload_buffer[0] = 999;
              payload_buffer[1] = 999;
        }
    ctimer_set(&opt_timer, next, init_opt_reading, NULL);
}
/*-------------------------------------------------*/
static void init_bmp_reading(void* not_used)
{
    SENSORS_ACTIVATE(bmp_280_sensor);
}
static void init_opt_reading(void* not_used)
{
    SENSORS_ACTIVATE(opt_3001_sensor);
}
static void init_hdc_reading(void* not_used)
{
    SENSORS_ACTIVATE(hdc_1000_sensor);
}
static void init_batt_reading(void* not_used)
{
    SENSORS_ACTIVATE(batmon_sensor);
}
/*-------------------------------------------------*/
static void init_sensor_readings(void)
{
    SENSORS_ACTIVATE(hdc_1000_sensor);
    SENSORS_ACTIVATE(opt_3001_sensor);
    SENSORS_ACTIVATE(bmp_280_sensor);
    SENSORS_ACTIVATE(batmon_sensor);
}
/*-------------------------------------------------*/
PROCESS(fieldarray_sp_process, "SENSORS");
PROCESS(udp_send_process, "UDP");
/*-------------------------------------------------*/
AUTOSTART_PROCESSES(&fieldarray_sp_process, &udp_send_process);
/*-------------------------------------------------*/
PROCESS_THREAD(udp_send_process, ev, data)
{

    static struct etimer et;

    PROCESS_BEGIN();


    while (1) {

        PROCESS_WAIT_EVENT_UNTIL(ev == cycle_event);

        package_number++;
        get_battery_reading();
        payload_buffer[12] = package_number;

        convertPayloadToBytes(payload_buffer, bytes_payload);

        if (NETSTACK_ROUTING.node_is_reachable() &&
        NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) {

            // printPayloadAsJSON(payload_buffer);
            // printBytesPayloadAsJSON(bytes_payload);
            uint8_t sensors_payload[68];  // 17 elements * 4 bytes each
            for (int i = 0; i < sizeof(sensors_payload); i++) {
                sensors_payload[i] = bytes_payload[i];
            }

            simple_udp_sendto(&udp_conn, sensors_payload, sizeof(sensors_payload), &dest_ipaddr);
            LOG_INFO("\n\rSending a Payload Size of: %d bytes\n\r", sizeof(sensors_payload));

        } else {
            LOG_INFO("\n\rRoot node NOT reachable...\n\r");
        }
        // /* Set a timer to trigger every 10 seconds */
        // etimer_set(&et,RESEND_INTERVAL );
        // PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

        // if (NETSTACK_ROUTING.node_is_reachable() &&
        //         NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) {

        //     uint8_t sensors_payload[68];  // 17 elements * 4 bytes each
        //     for (int i = 0; i < sizeof(sensors_payload); i++) {
        //         sensors_payload[i] = bytes_payload[i];
        //     }
        //     simple_udp_sendto(&udp_conn, sensors_payload, sizeof(sensors_payload), &dest_ipaddr);
        //     LOG_INFO("Resending a Payload Size of: %d bytes\n\r", sizeof(sensors_payload));

        // } else {
        //     LOG_INFO("Root node NOT reachable...\n\r");
        // }
    }

    PROCESS_END();
}


/*-------------------------------------------------*/
PROCESS_THREAD(fieldarray_sp_process, ev, data)
{
    PROCESS_BEGIN();

    init_sensor_readings();

    if (!is_registered) {
        simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL, UDP_SERVER_PORT, NULL);
        is_registered = true;
    }

    NETSTACK_MAC.on();

    while (1) {

        PROCESS_YIELD();

        if (ev == button_hal_press_event) {
            button_hal_button_t* btn = (button_hal_button_t*)data;
            if (btn->unique_id == LEFT_KEY){
                if (NETSTACK_ROUTING.node_is_reachable() &&
                        NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr))
                {
                    simple_udp_sendto(&udp_conn, ping, sizeof(ping), &dest_ipaddr);
                    LOG_INFO("\n\rPINGv6: %d", sizeof(ping));
                }
            }
            if (btn->unique_id == RIGHT_KEY)
            {
                LOG_INFO("\n\rRebooting...");
                watchdog_reboot();
            }
        }
        if (ev == sensors_event) {
            if (data == &bmp_280_sensor) {
                get_bmp_reading();
                sensor_cycle_status |= BMP_COMPLETED;
            } else if (data == &opt_3001_sensor) {
                get_light_reading();
                sensor_cycle_status |= OPT_COMPLETED;
            } else if (data == &hdc_1000_sensor) {
                get_hdc_reading();
                sensor_cycle_status |= HDC_COMPLETED;
            }

            // Check if all sensors have completed their cycle
            if (sensor_cycle_status == (BMP_COMPLETED | OPT_COMPLETED | HDC_COMPLETED)) {
                process_post(&udp_send_process, cycle_event, NULL);
                // printf("fieldarray_sp_process: Event posted.\n");
                sensor_cycle_status = 0;  // Reset for the next cycle
            }
        }


    }
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/** @} */
