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

rfm95_handle_t rfm95_handle = { .spi_handle = &hspi2, .nss_port = LORA_NSS_GPIO_Port,
		.nss_pin = LORA_NSS_Pin, .nrst_port = Reset_LoRa_GPIO_Port, .nrst_pin =
				Reset_LoRa_Pin, .irq_port = LORA_DIO0_GPIO_Port, .irq_pin =
				LORA_DIO0_Pin, .dio5_port = LORA_DIO5_GPIO_Port, .dio5_pin =
				LORA_DIO5_Pin, .device_address = { 0x26, 0x0B, 0x9B, 0xB7 },
		.application_session_key = { 0x36, 0x99, 0xBA, 0xEE, 0x6F, 0x2C, 0xFC, 0xB1, 0x20, 0xB6, 0xF8, 0xB0, 0x99, 0xFC, 0xED, 0xE3 },
		.network_session_key = { 0x0C, 0x78, 0xCE, 0x6F, 0xC4, 0x15, 0x29, 0x37, 0x2A, 0xE6, 0x52, 0x8B, 0x7E, 0x6F, 0xC6, 0x45 },
		.reload_frame_counter = NULL, .save_frame_counter = NULL, .longitude =
				8220250, .longitude_or = { 'E' }, .latitude = 47478520,
		.latitude_or[0] = { 'N' }, .altitude = 2461, .indent = 0 };

FlowInit() {
	// Initialise RFM95 module.
	if (!rfm95_init(&rfm95_handle)) {
		printf("RFM95 init failed\n\r");
	}else {
		printf("RFM95 init sucess\n\r");
	}
}

Flow() {
	static uint8_t loraCounter;
	uint8_t data_packet[] = { 0x01, 0x02, 0x03, 0x4 };
	uint8_t data_packet_ground[3];

//	if (loraCounter == LORAINTERVALL) {
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

//	}

	loraCounter++;
}

