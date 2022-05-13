/*
 * flow.c
 *
 *  Created on: May 13, 2022
 *      Author: Fabian Glutz
 */

#include "flow.h"
#include "main.h"
#include "../../Core/Inc/stm32-hal-rfm95/rfm95.h"

#define LORAINTERVALL 5
extern SPI_HandleTypeDef hspi2;

rfm95_handle_t rfm95_handle = { .spi_handle = &hspi2, .nss_port = GPIOB,
		.nss_pin = GPIO_PIN_6, .nrst_port = Reset_LoRa_GPIO_Port, .nrst_pin =
				Reset_LoRa_Pin, .irq_port = LORA_DIO0_GPIO_Port, .irq_pin =
				LORA_DIO0_Pin, .dio5_port = LORA_DIO5_GPIO_Port, .dio5_pin =
				LORA_DIO5_Pin, .device_address = { 0x26, 0x0B, 0x3F, 0x79 },
		.application_session_key = { 0xCB, 0xB2, 0xE0, 0x78, 0xDB, 0x50, 0x85,
				0xC5, 0x0F, 0x73, 0x62, 0x2B, 0x12, 0xE6, 0x43, 0x8B },
		.network_session_key = { 0x8E, 0x05, 0x50, 0x9C, 0x74, 0x30, 0x4F, 0x8D,
				0x33, 0x8B, 0x9B, 0x2D, 0xC2, 0x76, 0x3C, 0x56 },
		.reload_frame_counter = NULL, .save_frame_counter = NULL, .longitude =
				8220250, .longitude_or = { 'E' }, .latitude = 47478520,
		.latitude_or[0] = { 'N' }, .altitude = 2461, .indent = 0 };

FlowInit() {

}

Flow() {
	static uint8_t loraCounter;
	uint8_t data_packet[] = { 0x01, 0x02, 0x03, 0x4 };
	uint8_t data_packet_ground[3];

	if (loraCounter == LORAINTERVALL) {
		if (!rfm95_send_data(&rfm95_handle, data_packet, sizeof(data_packet))) {
			printf("RFM95 send failed\n\r");
		} else {
			printf("RFM95 send success\n\r");

			if (!rfm95_send_data_groundstation(&rfm95_handle)) {
				printf("RFM95 send failed\n\r");
			} else {
				printf("RFM95 send success\n\r");
			}
		}

	}

	loraCounter++;
}
