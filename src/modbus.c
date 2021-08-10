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
 */

#ifdef CRC16_MEMORY_MAPPED

/* Table of CRC values for highorder byte */
static uint8_t auchCRCHi[] = {
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for loworder byte */
static uint8_t auchCRCLo[] = {
		0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
		0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
		0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
		0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
		0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
		0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
		0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
		0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
		0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
		0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
		0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
		0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
		0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
		0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
		0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
		0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};


/* taken from "Modbus over serial line specification and implementation guide", Appendix B */
uint16_t modbus_CRC16(const uint8_t *puchMsg, int usDataLen )
{
	if (usDataLen < 0) {
		return 0xFFFF;
	}
	uint8_t uchCRCHi = 0xFF;
	uint8_t uchCRCLo = 0xFF;
	/* low byte of CRC initialized  */
	unsigned uIndex;
	/* will index into CRC lookup table  */
	/* pass through message buffer  */
	while (usDataLen--)  {
		uIndex = uchCRCLo ^ *puchMsg++ ;
		/* calculate the CRC  */
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo);
}

#else

/* alternative CRC16 (without memory mapped values)
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

#endif /* CRC16_MEMORY_MAPPED */

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
		//printf("crc mismatch: received 0x%x, calculated 0x%x\n", crc_received, crc_calculated);
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
