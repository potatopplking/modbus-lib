/*
 * modbus.c
 *
 *  Created on: Jul 18, 2021
 *      Author: user
 */

#include "modbus.h"

/*
 * Global variables
 */

/* Modbus TX buffer; can be also used for RX in memory constrained systems (e.g. in main.c);
 * NOTE if shared buffer is used for TX/RX, care must be taken to prevent writing into buffer
 * during execution of modbus_process_message() */
uint8_t modbus_buffer[MODBUS_MAX_RTU_FRAME_SIZE];

/* device address: declared  */
uint8_t modbus_slave_address = MODBUS_DEFAULT_SLAVE_ADDRESS;

/*
 * CRC16 functions
 * see https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 * section 6.2.2
 */

/* CRC16 (without memory mapped values)
 * taken from https://ctlsys.com/support/how_to_compute_the_modbus_rtu_message_crc/ */
uint16_t modbus_CRC16(const uint8_t *buf, int len)
{
	uint16_t crc = 0xFFFF;

	for (int pos = 0; pos < len; pos++) {
		crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
       
		for (int i = 8; i != 0; i--) {    // Loop over each bit
			if ((crc & 0x0001) != 0) {      // If the LSB is set
				crc >>= 1;                    // Shift right and XOR 0xA001
				crc ^= 0xA001;
			} else {                            // Else LSB is not set
				crc >>= 1;                    // Just shift right
			}
		}
	}
	// Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
	return crc;  
}

/*
 * Private functions
 */

/* here we assume buffer has minimal size of MODBUS_MAX_RTU_FRAME_SIZE;
 * this function is private, so hopefully it's going to be ok */
int8_t modbus_copy_reply_to_buffer(uint8_t *buffer, uint8_t *msg_len, modbus_transaction_t *transaction)
{
	uint16_t crc16;
	uint8_t byte_count;

	buffer[0] = modbus_slave_address;
	buffer[1] = transaction->function_code;
	*msg_len = 5;

	if (transaction->function_code | MODBUS_ERROR_FLAG) {
		/* sending error reply */
		buffer[2] = transaction->exception.exception_code;
	}
	switch (transaction->function_code) {
		case MODBUS_READ_HOLDING_REGISTERS:
		case MODBUS_READ_INPUT_REGISTERS:
			byte_count = transaction->register_count * 2;
			buffer[2] = byte_count;
			*msg_len = byte_count + 5;
			for (int i = 0; i < transaction->register_count; i++) {
				// TODO endianness handling
				/* buffer16b is alias for both holding and input register buffers */
				buffer[3 + 2*i] = transaction->buffer16b[i] >> 8;
				buffer[4 + 2*i] = transaction->buffer16b[i] & 0xff;
			}
			break;
	}
	crc16 = modbus_CRC16(buffer, *msg_len - 2); /* last two bytes is the checksum itself */
	buffer[*msg_len - 2] = crc16 & 0xff;
	buffer[*msg_len - 1] = crc16 >> 8;
}

/*
 * Public function definitions
 */

int8_t modbus_slave_set_address(uint8_t address)
{
	if (address == 0) {
		/* address 0 is broadcast address */
		return MODBUS_ERROR;
	}
	modbus_slave_address = address;
	return MODBUS_OK;
}

int8_t modbus_slave_process_msg(const uint8_t *buffer, int len)
{
	/* transaction holds message context and content:
	 * it wraps all necessary buffers and variables */
	modbus_transaction_t transaction;
	int8_t callback_result;
	uint8_t buffer_pos = 0;

	if (len < MODBUS_MINIMAL_FRAME_LEN) {
		/* frame too short; return error */
		return MODBUS_ERROR_FRAME_INVALID;
	}
	/* check CRC first */
	uint16_t crc_received = (buffer[len - 1] << 8) | buffer[len - 2];
	uint16_t crc_calculated = modbus_CRC16(buffer, len - 2);
	if (crc_received != crc_calculated) {
		/* CRC mismatch, return error */
		printf("crc mismatch: received 0x%x, calculated 0x%x\n", crc_received, crc_calculated);
		return MODBUS_ERROR_CRC;
	}
	/* check if address matches ours */
	uint8_t address = buffer[buffer_pos++];
	if (address != modbus_slave_address && address != MODBUS_BROADCAST_ADDR) {
		/* Message is not for us */
		return MODBUS_OK;
	}
	/* get function code */
	transaction.function_code = buffer[buffer_pos++];
	transaction.exception.exception_code = 0;

	if (transaction.function_code == MODBUS_READ_DEVICE_IDENTIFICATION) {
		// TODO
		goto modbus_send;
	}

	/* set starting register number */
	switch (transaction.function_code) {
	/* coils */
	case MODBUS_READ_DO:
	case MODBUS_WRITE_SINGLE_DO:
	case MODBUS_WRITE_MULTIPLE_DO:
		transaction.register_number = MODBUS_DO_START_NUMBER;
		break;
	/* discrete inputs */
	case MODBUS_READ_DI:
		transaction.register_number = MODBUS_DI_START_NUMBER;
		break;
	/* input registers */
	case MODBUS_READ_AI:
		transaction.register_number = MODBUS_AI_START_NUMBER;
		break;
	/* holding registers */
	case MODBUS_READ_AO:
	case MODBUS_WRITE_SINGLE_AO:
	case MODBUS_WRITE_MULTIPLE_AO:
	case MODBUS_READ_WRITE_MULTIPLE_REGISTERS:
		transaction.register_number = MODBUS_AO_START_NUMBER;
		break;	
	}

	#define MODBUS_FLAG_WRITE  0x01
	#define MODBUS_FLAG_SINGLE 0x02
	uint8_t flags = 0x00;

	/* process message */
	switch (transaction.function_code) {
	case MODBUS_WRITE_SINGLE_COIL:
	case MODBUS_WRITE_SINGLE_REGISTER: /* holding register */
		flags |= MODBUS_FLAG_SINGLE;
	case MODBUS_WRITE_MULTIPLE_COILS:
	case MODBUS_WRITE_MULTIPLE_REGISTERS:
		flags |= MODBUS_FLAG_WRITE;
	case MODBUS_READ_DISCRETE_INPUTS:
	case MODBUS_READ_COILS:
	case MODBUS_READ_INPUT_REGISTERS:
	case MODBUS_READ_HOLDING_REGISTERS:
		if (len < (MODBUS_MINIMAL_FRAME_LEN + 4)) {
			/* buffer too short to contain everything we need */
			return MODBUS_ERROR;
		}
		transaction.register_address = (buffer[buffer_pos++] << 8) | buffer[buffer_pos++];
		transaction.register_count = (buffer[buffer_pos++] << 8) | buffer[buffer_pos++];
		if (
				transaction.register_count < 1 ||
				transaction.register_count > MODBUS_MAX_REGISTERS
		   ) {
			transaction.exception.exception_code = MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
		}
		// add offset to register number
		transaction.register_number += transaction.register_address;
		break;
	default:
		/* function code not known / not implemented, reply with
		 * ExceptionCode 1 */
		transaction.exception.exception_code = MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
		break;
	}
	/* data in modbus_buffer have been processed and buffer can be re-used for TX */
	/* handle reply */
	if (transaction.exception.exception_code != 0) {
		/* indicate error */
		transaction.function_code |= MODBUS_ERROR_FLAG;
	} else {
		callback_result = modbus_slave_callback(&transaction);
		/* error handling */
		if (callback_result != MODBUS_OK) {
			transaction.function_code |= MODBUS_ERROR_FLAG;
			if (callback_result == MODBUS_ERROR_FUNCTION_NOT_IMPLEMENTED) {
				transaction.exception.exception_code = MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
			} else if (callback_result == MODBUS_ERROR_REGISTER_NOT_IMPLEMENTED) {
				transaction.exception.exception_code = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
			}
		}
	}
	uint8_t msg_len = 0;
modbus_send:
	if (address != MODBUS_BROADCAST_ADDR) {
		/* send only if master request was not broadcast */
		modbus_copy_reply_to_buffer(modbus_buffer, &msg_len, &transaction);
		modbus_transmit_function(modbus_buffer, msg_len);
	}
}
