#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <stdint.h>
#define MAX_BUFFER_SIZE 255



#define UART_PORT_RPI 	"/dev/ttyAMA1"
#define UART_PORT1_NXP 	"/dev/ttyUSB0"
#define UART_PORT2_NXP	"/dev/ttyUSB1"


/**
 * Routine to open serial port, set the necessary configurations
 * and return the file id
 * @param    speed_t - uart_baud - Baud rate. Refer termios.h for valid values
 * @param	 path_name - Path to serial port to open
 * @return    file-id of the Serial port opened
 * \todo Add additional check and assign default baud rate if wrong parameter is sent
 * */
int OpenSerialPort(const char* path_name, speed_t uart_baud);


typedef enum{
	FAILURE = -1,/**< FAILURE */
	SUCCESS = 1, /**< SUCCESS */
}FUNC_STATUS;

