BUILD_DIR=build

all:
	mkdir $(BUILD_DIR) 2> /dev/null | true
	gcc -o $(BUILD_DIR)/test_in_out tests/test_in_out.c src/modbus.c -I include/ -ggdb3
clean:
	rm -rf $(BUILD_DIR)
