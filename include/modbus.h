/*
 * modbus.h
 *
 *  Created on: Jul 18, 2021
 *      Author: user
 *
 *  Modbus slave RTU library (does NOT support ASCII and TCP)
 *
 *  Useful links:
 *  https://www.picotech.com/library/oscilloscopes/modbus-serial-protocol-decoding
 *  https://ipc2u.com/articles/knowledge-base/modbus-rtu-made-simple-with-detailed-descriptions-and-examples/
 *  https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 *  https://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf
 *
 *  Note that byte order is big endian.
 *
 * USAGE:
 *
 * 1) Implement functions modbus_callback_function() and modbus_uart_transmit_function()
 *      - modbus_uart_transmit_function() sends data via UART
 *      - modbus_callback_function() does the real work: read sensors, set outputs...
 *        note that when filling buffers (e.g. input_registers[]) user must
 *        ensure that all data is big-endian
 *    These functions are implementation-specific.
 * 2) Set device address (variable modbus_device_address); you can do this either
 *      - setting modbus_device_address directly (modbus.h needs to be included, duh)
 *      - using modbus_set_device_address(uint8_t address) function
 *    Or you can leave address as-is (MODBUS_DEFAULT_SLAVE_ADDRESS) and set it via
 *    Modbus during runtime
 * 3) Call modbus_process_msg() after message reception; you need to observe Modbus RTU timing:
 *      - pauses between chars in frame are less or equal to 1.5 char
 *      - pauses between frames are at least 3.5 chars (of silence)
 *    For more information see section 2.5.1.1 (MODBUS Message RTU Framing)
 *    in "MODBUS over Serial Line: Specification and Implementation Guide"
 *
 */

#ifndef SRC_MODBUS_H_
#define SRC_MODBUS_H_

#include "stdint.h"

/*
 * Defines & macros
 */

#define MODBUS_BROADCAST_ADDR 0
#define MODBUS_DEFAULT_SLAVE_ADDRESS 254 /* 255 may be used for bridge device */
/* minimal frame length is 4 bytes: 1 B slave address, 1 B function code, 2 B CRC */
#define MODBUS_MINIMAL_FRAME_LEN 4
#define MODBUS_MAX_RTU_FRAME_SIZE 256
#define MODBUS_ERROR_FLAG 0x80
#define MODBUS_MAX_REGISTERS 125

/*
 * Return values
 */

#define MODBUS_OK 0
#define MODBUS_ERROR -1 // generic error
#define MODBUS_ERROR_CRC -2 // checksum failed
#define MODBUS_ERROR_FRAME_INVALID -3 // invalid frame format / length
#define MODBUS_ERROR_OUT_OF_BOUNDS -4 // requested register is out of bounds
#define MODBUS_ERROR_FUNCTION_NOT_IMPLEMENTED -5 // function not implemented in callback
#define MODBUS_ERROR_REGISTER_NOT_IMPLEMENTED -6 // register not implemented in callback

/*
 * Data types
 */

/* Public functions codes (Modbus Application protocol specification, section 5.1) */
typedef enum {
	/* single bit access functions */
	MODBUS_READ_COILS = 1,
	MODBUS_READ_DO = 1, // alias
	MODBUS_READ_DISCRETE_INPUTS = 2,
	MODBUS_READ_DI = 2, // alias
	MODBUS_WRITE_SINGLE_COIL = 5,
	MODBUS_WRITE_SINGLE_DO = 5, // alias
	MODBUS_WRITE_MULTIPLE_COILS = 15,
	MODBUS_WRITE_MULTIPLE_DO = 15, // alias
	/* 16-bit access functions */
	MODBUS_READ_HOLDING_REGISTERS = 3,
	MODBUS_READ_AO = 3, // alias
	MODBUS_READ_INPUT_REGISTERS = 4,
	MODBUS_READ_AI = 4, // alias
	MODBUS_WRITE_SINGLE_REGISTER = 6,
	MODBUS_WRITE_SINGLE_AO = 6, // alias
	MODBUS_WRITE_MULTIPLE_REGISTERS = 16,
	MODBUS_WRITE_MULTIPLE_AO = 16, // alias
	MODBUS_MASK_WRITE_REGISTER = 22,
	MODBUS_READ_WRITE_MULTIPLE_REGISTERS = 23,
	MODBUS_READ_FIFO_QUEUE = 24,
	/* file record access */
	MODBUS_READ_FILE_RECORD = 20,
	MODBUS_WRITE_FILE_RECORD = 21,
	/* diagnostics */
	MODBUS_READ_EXCEPTION_STATUS = 7,
	MODBUS_DIAGNOSTIC = 8, /* sub codes: 00-18,20 */
	MODBUS_GET_COM_EVENT_COUNTER = 11,
	MODBUS_GET_COM_EVENT_LOG = 12,
	MODBUS_REPORT_SLAVE_ID = 17,
	MODBUS_READ_DEVICE_IDENTIFICATION = 43, /* sub codes: 14 */
} modbus_function_code_t;

typedef enum {
	MODBUS_EXCEPTION_ILLEGAL_FUNCTION = 1,
	MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS = 2,
	MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE = 3,
	MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE = 4,
	MODBUS_EXCEPTION_ACKNOWLEDGE = 5,
	MODBUS_EXCEPTION_SLAVE_DEVICE_BUSY = 6,
	MODBUS_EXCEPTION_MEMORY_PARITY_ERROR = 8,
	MODBUS_EXCEPTION_GATEWAY_PATH_UNAVAILABLE = 10,
	MODBUS_EXCEPTION_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 11,
} modbus_exception_code_t;

typedef struct {
	uint8_t exception_code;
} exception_t;

typedef struct {
	modbus_function_code_t function_code : 8;
	uint16_t register_address; // e.g. first register of A0: 0
	uint16_t register_number;  // e.g. first register of A0: 40001
	uint8_t  register_count; // number of registers to be read/written

	exception_t exception;

	union {
		uint8_t buffer8b[MODBUS_MAX_RTU_FRAME_SIZE];
		uint16_t buffer16b[MODBUS_MAX_RTU_FRAME_SIZE/2];
		uint16_t input_registers[MODBUS_MAX_REGISTERS];
		uint16_t holding_registers[MODBUS_MAX_REGISTERS];
	};
} modbus_transaction_t;

typedef enum {
	MODBUS_DO_START_NUMBER = 1, // Discrete output coils
	MODBUS_DO_END_NUMBER = 9999,
	MODBUS_DI_START_NUMBER = 10001, // Discrete input contacts
	MODBUS_DI_END_NUMBER = 19999,
	MODBUS_AI_START_NUMBER = 30001, // Analog input registers
	MODBUS_AI_END_NUMBER = 39999,
	MODBUS_AO_START_NUMBER = 40001, // Analog output (holding registers)
	MODBUS_AO_END_NUMBER = 49999
} modbus_register_number_t;


/*
 * Global variables
 */

/* device address: declared in modbus.c */
extern uint8_t modbus_slave_address;

/* shared modbus buffer; defined in modbus.c; may be used elsewhere in code */
extern uint8_t modbus_buffer[];

/*
 * Function prototypes
 */

/* process message: should be called in when modbus message was received (e.g. in main.c)
 * modbus_process_msg() may call following functions:
 *     - modbus_callback_function() if data readout is requested
 *     - modbus_uart_transmit_function() if response is required
 * Both functions have to be implemented by user.
 */
int8_t modbus_slave_process_msg(const uint8_t *buffer, int len);
int8_t modbus_slave_set_address(uint8_t address);
/* modbus callback function type - should be implemented by user (e.g. in main.c) */
int8_t modbus_slave_callback(modbus_transaction_t *transaction);
/* UART transmit function type - should be implemented by user (e.g. in main.c) */
int8_t modbus_transmit_function(uint8_t *buffer, int data_len);

#endif /* SRC_MODBUS_H_ */
