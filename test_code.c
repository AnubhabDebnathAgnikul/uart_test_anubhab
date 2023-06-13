/**
 ******************************************************************************
 * @file           : uart.c
 * @project        : Agnikul Generic Software Library
 * @brief          : UART configuration and functionalities for send and receive.
 * @author         : Sreedhar Mahadevan, Shawn Nagar
 * @version        : v2.0
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) Agnikul Cosmos Private Limited
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Linux headers
#include <fcntl.h>  // Contains file controls like O_RDWR
#include <errno.h>  // Error integer and strerror() function
#include <unistd.h> // write(), read(), close()

// Project headers
//#include "uart.h"
#include "description.h"
//#include "fault_handling.h"
//#define PRINT_ERR perror
#define FAILURE -1

/**
 * Routine to open serial port, set the necessary configurations
 * and return the file id
 * @param    speed_t - uart_baud - Baud rate. Refer termios.h for valid values
 * @return    file-id of the Serial port opened
 * */
int OpenSerialPort(const char* path_name, speed_t uart_baud)
{
	/**<Open the serial port in read / write mode*/
	int serial_port = open(path_name, O_RDWR);
	if(serial_port < 0){
		PRINT_ERR();
		return FAILURE;
	}

	/**<Create new termios struct, we call it 'tty' for convention*/
	struct termios tty;

	/**<Read in existing settings, and handle any error*/
	if(tcgetattr(serial_port, &tty) != 0) {
		PRINT_ERR();
		return FAILURE;
	}

	tty.c_cflag &= (uint) ~PARENB; 	/**<Clear parity bit, disabling parity (most common)*/
	tty.c_cflag &= (uint) ~CSTOPB; 	/**<Clear stop field, only one stop bit used in communication (most common)*/
	tty.c_cflag &= (uint) ~CSIZE; 	/**<Clear all bits that set the data size*/
	tty.c_cflag |= (uint) CS8; 		/**<8 bits per byte (most common)*/
	tty.c_cflag &= (uint) ~CRTSCTS; /**<Disable RTS/CTS hardware flow control (most common)*/
	tty.c_cflag |= (uint) CREAD | CLOCAL; /**<Turn on READ & ignore ctrl lines (CLOCAL = 1)*/

	tty.c_lflag &= (uint) ~ICANON;	/**<Disable canonical mode*/
	tty.c_lflag &= (uint) ~ECHO; 	/**<Disable echo*/
	tty.c_lflag &= (uint) ~ECHOE; 	/**<Disable erasure*/
	tty.c_lflag &= (uint) ~ECHONL; 	/**<Disable new-line echo*/
	tty.c_lflag &= (uint) ~ISIG; 	/**<Disable interpretation of INTR, QUIT and SUSP*/
	tty.c_iflag &= (uint) ~(IXON | IXOFF | IXANY); /**<Turn off s/w flow ctrl*/
	tty.c_iflag &= (uint) ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); /**<Disable any special handling of received bytes*/

	tty.c_oflag &= (uint) ~OPOST; /**<Prevent special interpretation of output bytes (e.g. newline chars)*/
	tty.c_oflag &= (uint) ~ONLCR; /**<Prevent conversion of newline to carriage return/line feed*/
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

	tty.c_cc[VTIME] = 10;    /**<Wait for up to 1s (10 deciseconds), returning as soon as any data is received.*/
	tty.c_cc[VMIN]  = 100;    /**<Wait for the UART blocking read to check atleast 50 bytes (TODO: Check details)*/

	/**<Set in/out baud rate to be 'uart_baud' parameter*/
	if(cfsetispeed(&tty, uart_baud) < 0){
		PRINT_ERR();
		return FAILURE;
	}
	if(cfsetospeed(&tty, uart_baud) < 0){
		PRINT_ERR();
		return FAILURE;
	}

	/**<Save tty settings, also checking for error*/
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
		//printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		//return 0;
		PRINT_ERR();
		return FAILURE;
	}

	return serial_port;
}

