/*
 * flow.c
 *
 *  Created on: May 13, 2022
 *      Author: Fabian Glutz
 */

#include "flow.h"
#include "main.h"
#include <stdio.h>

#define LORAINTERVALL 5
extern SPI_HandleTypeDef hspi2;

FlowInit(rfm95_handle_t *handle) {
	// Initialise RFM95 module.
	if (!rfm95_init(handle)) {
		printf("RFM95 init failed\n\r");
	} else {
		printf("RFM95 init sucess\n\r");
	}
}

Flow(rfm95_handle_t *handle) {
	static uint8_t loraCounter;
//	uint8_t data_packet[] = { 0x01, 0x02, 0x03, 0x4 };
//	char data_packet1[] = { "$GPGGA,093108.00,4728.76159,N,00812.80882," };
//	char data_packet2[] = { "E,1,09,1.53,384.6,M,47.3,M,,*5C" };
//	int length = sizeof(handle->device_address)/sizeof(handle->device_address[0]);
//	uint8_t data_packet[length];
	uint8_t data_packet_ground[3];

//    for (int i = 0; i < length; i++) {
//    	data_packet[i] = handle->device_address[i];
//    }
	char data_packet[51];
	//data_packet[0] = handle->longitude;

	int cx;
	//cx = snprintf ( data_packet, 51, "The half of %d is %d", 60, 60/2 );
//	cx = snprintf(data_packet, 51, "{ %d, %c, %d, %c, %d}", handle->longitude,
//			handle->longitude_or[0], handle->latitude, handle->latitude_or[0],
//			handle->altitude);
//	for (uint8_t i = 0; i <= 51 - (cx + 1); ++i) {
//		data_packet[51 - i] = 0;
//	}

//	if (loraCounter == LORAINTERVALL) {
//	if (!rfm95_send_data(handle, data_packet, sizeof(data_packet))) {
//		printf("RFM95 send failed\n\r");
//	} else {
//		printf("RFM95 send success\n\r");
//	}

	size_t length = LENGHT_GROUNDSTATION;
	uint8_t ident = IDENT;
	uint8_t rfm_data[LENGHT_GROUNDSTATION];
	uint8_t rfm_package_length = LENGHT_GROUNDSTATION;
	unsigned long latitude = handle->latitude;
	unsigned long longitude = handle->longitude;
	unsigned altitude = handle->altitude;



	// set identifier to 1 for bits
	rfm_data[0] = ((ident << 4) & 0xf0);
	// set the next for bits with the orientation
	if (handle->latitude_or[0] == 'N' & handle->longitude_or[0] == 'E') {
		rfm_data[0] = (rfm_data[0] | 0x01);
	} else if (handle->latitude_or[0] == 'N' & handle->longitude_or[0] == 'W') {
		rfm_data[0] = (rfm_data[0] | 0x02);
	} else if (handle->latitude_or[0] == 'S' & handle->longitude_or[0] == 'E') {
		rfm_data[0] = (rfm_data[0] | 0x03);
	} else if (handle->latitude_or[0] == 'S' & handle->longitude_or[0] == 'W') {
		rfm_data[0] = (rfm_data[0] | 0x04);
	} else {
		rfm_data[0] = (rfm_data[0] | 0x00);
	}

	// set latitude
	for (int i = 0; i < 4; i++) {
		rfm_data[4 - i] = ((latitude >> (i * 8)) & 0xFF);
	}
	for (int i = 0; i < 4; i++) {
		rfm_data[8 - i] = ((longitude >> (i * 8)) & 0xFF);
	}

	for (int i = 0; i < 2; i++) {
		rfm_data[10 - i] = ((altitude >> (i * 8)) & 0xFF);
	}


	if (!rfm95_send_data(handle, rfm_data, sizeof(rfm_data))) {
		printf("RFM95 send failed\n\r");
	} else {
		printf("RFM95 send success\n\r");
	}

	if (!rfm95_send_data_groundstation(handle)) {
		printf("RFM95 send failed\n\r");
	} else {
		printf("RFM95 send success\n\r");
	}

//	}

	loraCounter++;
}

