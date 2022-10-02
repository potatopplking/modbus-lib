# Modbus slave RTU library

*(does NOT support ASCII and TCP)*

USAGE:

1. Implement functions `modbus_callback_function()` and `modbus_uart_transmit_function()`
    * `modbus_uart_transmit_function()` sends data via UART
    * `modbus_callback_function()` does the real work: read sensors, set outputs...

note that when filling buffers (e.g. `input_registers[]`) user must ensure that all data is big-endian. These functions are implementation-specific.

2. Set device address (variable `modbus_device_address`); you can do this either
    * setting `modbus_device_address` directly (modbus.h needs to be included, duh)
    * using `modbus_set_device_address(uint8_t address)` function
    
Or you can leave address as-is (`MODBUS_DEFAULT_SLAVE_ADDRESS`) and set it via Modbus during runtime

3. Call `modbus_process_msg()` after message reception; you need to observe Modbus RTU timing:
    * pauses between chars in frame are less or equal to 1.5 char
    * pauses between frames are at least 3.5 chars (of silence)

For more information see section 2.5.1.1 (MODBUS Message RTU Framing) in "MODBUS over Serial Line: Specification and Implementation Guide"

Note that byte order is big endian.

## Useful links:

https://www.picotech.com/library/oscilloscopes/modbus-serial-protocol-decoding

https://ipc2u.com/articles/knowledge-base/modbus-rtu-made-simple-with-detailed-descriptions-and-examples/

https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf

https://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf----------------------------------------

