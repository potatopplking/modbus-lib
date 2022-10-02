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

/* MODBUS device address */
uint8_t modbus_slave_address = MODBUS_DEFAULT_SLAVE_ADDRESS;

/* Device ID struct */
modbus_device_id_t *modbus_device_id = NULL;

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

static uint8_t modbus_fill_device_id_objects(uint8_t *buffer, modbus_transaction_t *transaction)
{
	/* we assume buffer is 256 - MODBUS_READ_DEVICE_ID_RESPONSE_HEADER_LEN = 252 bytes long */
	/* find out how many objects we copy to buffer */
	int len;
	uint8_t object_index = transaction->object_id;
	uint8_t object_count;
	uint8_t more_follows = MODBUS_NO_MORE_FOLLOWS;
	uint8_t next_object_id;
	uint8_t last_object;
	const uint8_t max_len = 256 - MODBUS_READ_DEVICE_ID_RESPONSE_HEADER_LEN;

	/* last object index */
	if (transaction->read_device_id_code == MODBUS_CONFORMITY_BASIC) {
		last_object = MODBUS_BASIC_OBJECT_COUNT;
	} else if (transaction->read_device_id_code == MODBUS_CONFORMITY_REGULAR) {
		last_object = MODBUS_REGULAR_OBJECT_COUNT;
	/* extended not implemented */
//	} else if (transaction->read_device_id_code == MODBUS_CONFORMITY_EXTENDED){
//		last_object = MODBUS_EXTENDED_OBJECT_COUNT;
	} else {
		/* fallback: regular */
		last_object = MODBUS_REGULAR_OBJECT_COUNT;
	}
	last_object--; // we need index
	/* copy as many objects as possible */
	do {
		/* copy object */
		int object_len = strlen(modbus_device_id->object_id[object_index]);
		if (len + object_len + 2 > max_len) {
			more_follows = MODBUS_MORE_FOLLOWS;
			next_object_id = object_index;
			break;
		}
		/* offset is for "more follows", "next object id", "object count" */
		buffer[MODBUS_READ_DEVICE_ID_RESPONSE_OFFSET + len++] = object_index;
		buffer[MODBUS_READ_DEVICE_ID_RESPONSE_OFFSET + len++] = object_len;
		/* note that string copied to buffer is not null-terminated */
		strncpy((char*)(buffer + len), (char*)modbus_device_id->object_id[object_index++], object_len);
		len += object_len;
		object_count++;
	} while (object_index < last_object);
	buffer[0] = more_follows;
	buffer[1] = next_object_id;
	buffer[2] = object_count;
	return MODBUS_READ_DEVICE_ID_RESPONSE_OFFSET + len;
}

/* here we assume buffer has minimal size of MODBUS_MAX_RTU_FRAME_SIZE;
 * this function is private, so hopefully it's going to be ok */
static int8_t modbus_transaction_to_buffer(uint8_t *buffer, uint8_t *msg_len, modbus_transaction_t *transaction)
{
	uint16_t crc16;
	uint8_t byte_count;
	uint8_t buffer_pos = 0;

	// TODO use relative indices (increments) instead of absolute
	buffer[buffer_pos++] = modbus_slave_address;
	buffer[buffer_pos++] = transaction->function_code;
	*msg_len = 5;

	if (transaction->function_code & MODBUS_ERROR_FLAG) {
		/* sending error reply */
		buffer[buffer_pos++] = transaction->exception;
	} else {
		switch (transaction->function_code) {
			case MODBUS_READ_HOLDING_REGISTERS:
			case MODBUS_READ_INPUT_REGISTERS:
				byte_count = transaction->register_count * 2;
				buffer[buffer_pos++] = byte_count;
				*msg_len = byte_count + 5;
				for (int i = 0; i < transaction->register_count; i++) {
					// TODO endianness handling
					/* buffer16b is alias for both holding and input register buffers */
					buffer[buffer_pos++] = transaction->buffer16b[i] >> 8;
					buffer[buffer_pos++] = transaction->buffer16b[i] & 0xff;
				}
				break;
			case MODBUS_WRITE_SINGLE_REGISTER:
				buffer[buffer_pos++] = (uint8_t) (transaction->register_address >> 8);
				buffer[buffer_pos++] = (uint8_t) transaction->register_address;
				buffer[buffer_pos++] = (uint8_t) (transaction->holding_registers[0] >> 8);
				buffer[buffer_pos++] = (uint8_t) transaction->holding_registers[0];
				*msg_len = 8; /* includes 2 bytes for CRC */
				break;
			case MODBUS_WRITE_MULTIPLE_REGISTERS:
				buffer[buffer_pos++] = (uint8_t) (transaction->register_address >> 8);
				buffer[buffer_pos++] = (uint8_t) transaction->register_address;
				buffer[buffer_pos++] = (uint8_t) (transaction->register_count >> 8);
				buffer[buffer_pos++] = (uint8_t) transaction->register_count;
				*msg_len = 8; /* includes 2 bytes for CRC */
				break;
			case MODBUS_READ_DEVICE_IDENTIFICATION:
				/* MEI type */
				buffer[buffer_pos++] = MODBUS_MEI;
				/* read device id */
				buffer[buffer_pos++] = transaction->read_device_id_code;
				/* conformity level */
				buffer[buffer_pos++] = modbus_device_id->conformity_level;
				/* fill buffer with as many objects as possible  */
				*msg_len = modbus_fill_device_id_objects(buffer+buffer_pos, transaction);
				*msg_len += 7; /* includes 2 bytes for CRC */
				break;
			default:
				break;
		}
	}
	crc16 = modbus_CRC16(buffer, buffer_pos); /* last two bytes is the checksum itself */
	buffer[buffer_pos++] = crc16 & 0xff;
	buffer[buffer_pos++] = crc16 >> 8;
	return MODBUS_OK;
}

static int8_t modbus_process_device_id_request(const uint8_t *buffer, int len, modbus_transaction_t *transaction)
{
	uint8_t MEI_type;
	uint8_t read_device_id_code;
	uint8_t object_id;
	uint8_t buffer_pos = 0;

	if (transaction->broadcast == 1) {
		/* Read device ID broadcast - invalid; ignore (master will get timeout) */
		return MODBUS_ERROR;
	}
	if (modbus_device_id == NULL) {
		/* modbus_device_id not initialized; user should use modbus_slave_init_device_id() first */
		transaction->exception = MODBUS_EXCEPTION_ILLEGAL_DEVICE_ID_CODE;
		return MODBUS_OK;
	}
	if (len < MODBUS_READ_DEVICE_ID_REQUEST_LEN) {
		/* frame too short, ignore */
		return MODBUS_ERROR;
	}
	/* next byte should be MEI = 0x0E */
	MEI_type = buffer[buffer_pos++];
	if (MEI_type != MODBUS_MEI) {
		/* invalid MEI, ignore. I have no idea what MEI does, but it should always be 0x0E */
		return MODBUS_ERROR;
	}
	/* next byte is read device id code */
	read_device_id_code = buffer[buffer_pos++];
	/* read device id code can only have values 1,2,3,4 */
	if (read_device_id_code < 1 || read_device_id_code > 4) {
		transaction->exception = MODBUS_EXCEPTION_ILLEGAL_DEVICE_ID_CODE;
		return MODBUS_OK;
	}
	transaction->read_device_id_code = read_device_id_code;
	/* next byte is object id */
	object_id = buffer[buffer_pos++];
	transaction->object_id = object_id;
	if (object_id > MODBUS_DEVICE_ID_OBJECT_NUM) {
		/* illegal object ID */
		transaction->exception = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
		return MODBUS_OK;
	}
	/* Message processed */
	return MODBUS_OK;
}

/* returns ERROR only when no response to master is needed */
static int8_t modbus_process_read_write_request(const uint8_t *buffer, int len, modbus_transaction_t *transaction)
{
	uint8_t byte_count;
	int8_t callback_result;
	uint8_t buffer_pos = 0;

	/* set starting register number */
	switch (transaction->function_code) {
	/* coils */
	case MODBUS_READ_DO:
	case MODBUS_WRITE_SINGLE_DO:
	case MODBUS_WRITE_MULTIPLE_DO:
		transaction->register_number = MODBUS_DO_START_NUMBER;
		break;
	/* discrete inputs */
	case MODBUS_READ_DI:
		transaction->register_number = MODBUS_DI_START_NUMBER;
		break;
	/* input registers */
	case MODBUS_READ_AI:
		transaction->register_number = MODBUS_AI_START_NUMBER;
		break;
	/* holding registers */
	case MODBUS_READ_AO:
	case MODBUS_WRITE_SINGLE_AO:
	case MODBUS_WRITE_MULTIPLE_AO:
	case MODBUS_READ_WRITE_MULTIPLE_REGISTERS:
		transaction->register_number = MODBUS_AO_START_NUMBER;
		break;
	default:
		break;
	}

	#define MODBUS_FLAG_WRITE  0x01
	#define MODBUS_FLAG_SINGLE 0x02
	uint8_t flags = 0x00;

	/* process message */
	switch (transaction->function_code) {
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
		if (len < MODBUS_MINIMAL_READWRITE_LEN) {
			/* buffer too short to contain everything we need */
			return MODBUS_ERROR;
		}
		transaction->register_address = (buffer[buffer_pos] << 8) | buffer[buffer_pos + 1];
		buffer += 2;
		// TODO check length!
		if (flags & MODBUS_FLAG_WRITE) {
			if (flags & MODBUS_FLAG_SINGLE) {
				transaction->holding_registers[0] = (buffer[buffer_pos] << 8) | buffer[buffer_pos + 1];
				buffer_pos += 2;
			} else {
				/* Write multiple registers */
				transaction->register_count = (buffer[buffer_pos] << 8) | buffer[buffer_pos + 1];
				buffer_pos += 2;
				if (len < MODBUS_MINIMAL_WRITE_MULTIPLE_LEN) {
					return MODBUS_ERROR;
				}
				byte_count = buffer[buffer_pos++];
				if (transaction->register_count > 123 || 2*transaction->register_count != byte_count) {
					/* Max number of register is defined by Modbus_Application_Protocol_V1_1b, section 6.12 */
					transaction->exception = MODBUS_EXCEPTION_ILLEGAL_REGISTER_QUANTITY;
				} else {
					if (len < MODBUS_MINIMAL_WRITE_MULTIPLE_LEN + byte_count) {
						return MODBUS_ERROR;
					}
					for (uint8_t i = 0; i < transaction->register_count; i++) {
						transaction->holding_registers[i] = (buffer[buffer_pos] << 8) | buffer[buffer_pos + 1];
						buffer_pos += 2;
					}
				}
			}
		} else {
			transaction->register_count = (buffer[buffer_pos] << 8) | buffer[buffer_pos + 1];
			buffer_pos += 2;
			if (
					transaction->register_count < 1 ||
					transaction->register_count > MODBUS_MAX_REGISTERS
			   ) {
				transaction->exception = MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
			}
		}
		// add offset to register number
		transaction->register_number += transaction->register_address;
		break;
	default:
		/* function code not known / not implemented, reply with
		 * ExceptionCode 1 */
		transaction->exception = MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
		break;
	}
	/* data in modbus_buffer have been processed and buffer can be re-used for TX */
	/* handle reply */
	if (transaction->exception != 0) {
		/* indicate error */
		transaction->function_code |= MODBUS_ERROR_FLAG;
	} else {
		callback_result = modbus_slave_callback(transaction);
		/* error handling */
		if (callback_result != MODBUS_OK) {
			transaction->function_code |= MODBUS_ERROR_FLAG;
			if (callback_result == MODBUS_ERROR_FUNCTION_NOT_IMPLEMENTED) {
				transaction->exception = MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
			} else if (callback_result == MODBUS_ERROR_REGISTER_NOT_IMPLEMENTED) {
				transaction->exception = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
			}
		}
	}
	return MODBUS_OK;
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


	/*
	 * TODO list:
	 *
	 * 1) check that errors and exceptions are handled according to Modbus_Application_Protocol_V1_1b.pdf
	 * 2) buffer overflow prevention: for each function code, check that buffer is long enough
	 */


	/* transaction holds message context and content:
	 * it wraps all necessary buffers and variables */
	modbus_transaction_t transaction;
	uint8_t buffer_pos = 0;

	if (len < MODBUS_MINIMAL_FRAME_LEN) {
		/* frame too short; return error (no reply needed) */
		return MODBUS_ERROR_FRAME_INVALID;
	}
	/* check CRC first */
	uint16_t crc_received = (buffer[len - 1] << 8) | buffer[len - 2];
	uint16_t crc_calculated = modbus_CRC16(buffer, len - 2);
	if (crc_received != crc_calculated) {
		/* CRC mismatch, return error (no reply needed) */
		return MODBUS_ERROR_CRC;
	}
	/* check if address matches ours */
	uint8_t address = buffer[buffer_pos++];
	transaction.broadcast = (address == MODBUS_BROADCAST_ADDR);
	if (address != modbus_slave_address && transaction.broadcast != 1) {
		/* Message is not for us (no reply needed) */
		return MODBUS_OK;
	}
	/* get function code */
	transaction.function_code = buffer[buffer_pos++];
	transaction.exception = 0;
	uint8_t request_processing_result;
	if (transaction.function_code == MODBUS_READ_DEVICE_IDENTIFICATION) {
		/* Read device ID request is quite complicated, therefore it has its own processing function */
		request_processing_result = modbus_process_device_id_request(buffer + buffer_pos, len - buffer_pos, &transaction);
	} else {
		/* process other requests: input register read, holding register read/write */
		request_processing_result = modbus_process_read_write_request(buffer + buffer_pos, len - buffer_pos, &transaction);
	}
	uint8_t msg_len;
	/* reply only if request was processed successfully and message was not broadcast */
	if (request_processing_result == MODBUS_OK && transaction.broadcast == 0) {
		modbus_transaction_to_buffer(modbus_buffer, &msg_len, &transaction);
		/* send reply */
		modbus_transmit_function(modbus_buffer, msg_len);
	}
	return MODBUS_OK;
}

int8_t modbus_slave_init_device_id(modbus_device_id_t *device_id)
{
	if (device_id == NULL) {
		return MODBUS_ERROR;
	}
	/* at least basic category objects have to be implemented */
	if (	device_id->object_name.VendorName == NULL  ||
			device_id->object_name.ProductCode == NULL ||
			device_id->object_name.MajorMinorRevision == NULL
	) {
		return MODBUS_ERROR;
	}
	/* set conformity level: currently only "basic" and "regular" is implemented */
	if ( 	device_id->object_id[3] != NULL &&
			device_id->object_id[4] != NULL &&
			device_id->object_id[5] != NULL

	) {
		/* strings are present in regular category (optional) */
		device_id->conformity_level = MODBUS_CONFORMITY_REGULAR;
	} else {
		device_id->conformity_level = MODBUS_CONFORMITY_BASIC;
	}
	/* we support both stream and individual access to objects */
	device_id->conformity_level |= MODBUS_DEVICE_ID_INDIVIDUAL_ACCESS_FLAG;
	modbus_device_id = device_id;
	return MODBUS_OK;
}
