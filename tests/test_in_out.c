/*
 * Simple input-output comparison test
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "modbus.h"

uint8_t in_frame[][MODBUS_MAX_RTU_FRAME_SIZE] = {
	/* read holding registers, starting address 108 (number 40108), 3 registers */
	{0x11, 0x003, 0x00, 0x6B, 0x00, 0x03, 0x76, 0x87},
};

uint8_t out_frame[][MODBUS_MAX_RTU_FRAME_SIZE] = {
	{0x11, 0x03, 0x06, 0xAE, 0x41, 0x56, 0x52, 0x43, 0x40, 0x49, 0xAD},
};

int in_frame_len[] = {
	sizeof(in_frame[0])
};

int out_frame_len[] = {
	sizeof(out_frame[0])
};

#define N 32
uint8_t actual_out_frame[N][MODBUS_MAX_RTU_FRAME_SIZE];
uint8_t actual_out_frame_len[N];

static int test_number = 0;
const int test_count = sizeof(in_frame) / sizeof(uint16_t*);

int8_t modbus_callback_function(modbus_transaction_t *transaction)
{
	uint16_t dummy_holding_registers[] = { 0xAE41, 0x5652, 0x4340 };
	if (transaction->function_code == MODBUS_READ_HOLDING_REGISTERS) {
		if (transaction->register_number == 40108) {
			if (transaction->register_count > sizeof(dummy_holding_registers)/sizeof(uint16_t)) {
				fprintf(stderr, "Error: register count too high");
				exit(-1);
			}
			for (int i = 0; i < transaction->register_count; i++) {
				transaction->holding_registers[i] = dummy_holding_registers[i];
			}
			return MODBUS_OK;
		} else {
			return MODBUS_ERROR_OUT_OF_BOUNDS; // modbus slave should return exception code 2
		}
	} else {
		return MODBUS_ERROR_NOT_IMPLEMENTED;
	}

}

int8_t modbus_transmit_function(uint8_t *buffer, int data_len)
{
	actual_out_frame_len[test_number] = 0;
	for (int i = 0; i < data_len; i++) {
		actual_out_frame[test_number][i] = buffer[i];
		actual_out_frame_len[test_number]++;
	}
}

int main(void)
{
	printf("In-out frame test (%d tests total)\n", test_count);
	while (test_number < test_count) {
		printf("Starting test %d\n\tIn frame (master request):\t[ ", test_number);
		for (int i = 0; i < in_frame_len[test_number]; i++) {
			printf("0x%x, ", in_frame[test_number][i]);
		}
		printf("\b\b ]\n\tDesired out frame (slave response):\t[ ");
		for (int i = 0; i < out_frame_len[test_number]; i++) {
			printf("0x%x, ", out_frame[test_number][i]);
		}
		modbus_process_msg(in_frame[test_number], in_frame_len[test_number]);
		printf("\b\b ]\n\tActual out frame (slave response):\t[ ");
		for (int i = 0; i < actual_out_frame_len[test_number]; i++) {
			printf("0x%x, ", actual_out_frame[test_number][i]);
		}
		test_number++;
	}
}
