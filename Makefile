BUILD_DIR=build

clean:
	rm -rf $(BUILD_DIR)

all:
	mkdir $(BUILD_DIR) 2> /dev/null | true
	gcc -o $(BUILD_DIR)/test_in_out tests/test_in_out.c src/modbus.c -I include/
