/*
 * Copyright (C) 2014 DFRobot                                                  
 *                                                                                                                             
 * www.github.com/dfrobot/df_rplidar_to_socket (github as default source provider)
 *  DFRobot-A great source for opensource hardware and robot.                                     
 *                                                                             
 * df_rplidar_to_socket is free software: you can redistribute it and/or         
 * modify it under the terms of the GNU General Public License as       
 * published by the Free Software Foundation, either version 3 of              
 * the License, or any later version.                                          
 *                                                                             
 * df_rplidar_to_socket is distributed in the hope that it will be useful,       
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               
 * GNU General Public License for more details.                         
 *                                                                             
 * df_rplidar_to_socket is distributed in the hope that it will be useful,       
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               
 * GNU General Public License for more details.                         
 *                                                                             
 * You should have received a copy of the GNU General Public            
 * License along with df_rplidar_to_socket. If not, see                          
 * <http://www.gnu.org/licenses/>.                                             
 *                                                                             
 */

/*
 *	name:				df_rplidar_to_socket
 *	version:			0.1
 *	Author:				lisper <lisper.li@dfrobot.com>
 *	Date:				2014-08-06
 *	official website:		http://www.dfrobot.com
 *	Description:			rplidar't data send to socket for df_rplidar_map
 *	this code is based on RoboPeak's Demo App
 */




#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <sys/socket.h>
#include <resolv.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#define MAXBUF 1024

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

RPlidarDriver * drv ;

int my_socket, new_fd;
int pid;
char buf[MAXBUF+1];
socklen_t len;

struct sockaddr_in my_addr, their_addr;

//
bool checkRPLIDARHealth(RPlidarDriver * drv) {
	u_result     op_result;
	rplidar_response_device_health_t healthinfo;


	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
		printf("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
			// enable the following code if you want rplidar to be reboot by software
			// drv->reset();
			return false;
		} else {
			return true;
		}

	} else {
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}
}

//
int main(int argc, const char * argv[]) {
	void* send_data (void *arg);
	pthread_t send_tid;

	if ((my_socket = socket (AF_INET, SOCK_STREAM, 0)) < 0) {
		perror ("[error in socket]\n");	
		exit (1);
	}

	memset (&my_addr, 0, sizeof (my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons (atoi (argv[3]));
	my_addr.sin_addr.s_addr = INADDR_ANY;


	if (bind (my_socket, (struct sockaddr *) &my_addr, sizeof (struct sockaddr)) == -1) {
		perror ("error in bind\n");
		exit (2);
	}

	if (listen (my_socket, 10) == -1) {
		perror ("[error in listen!]\n");
		exit (3);
	}

	printf ("wait for connect\n");

	const char * opt_com_path = NULL;
	_u32         opt_com_baudrate = 115200;

	// read serial port from the command line...
	if (argc>1) 
		opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

	// read baud rate from the command line if specified...
	if (argc>2) 
		opt_com_baudrate = strtoul(argv[2], NULL, 10);


	if (!opt_com_path) {
#ifdef _WIN32
		// use default com port
		opt_com_path = "\\\\.\\com3";
#else
		opt_com_path = "/dev/ttyUSB0";
#endif
	}

	// create the driver instance
	drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

	if (!drv) {
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}


	// make connection...
	if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
				, opt_com_path);
		goto on_finished;
	}



	// check health...
	if (!checkRPLIDARHealth(drv)) {
		goto on_finished;
	}


	// start scan...
	drv->startScan();

	// fetech result and print it out...
	//
	len = sizeof (struct sockaddr);
	while (1) {
		if ((new_fd = accept (my_socket, (struct sockaddr *) &their_addr, &len)) == -1) {
			perror ("[error in accept]\n");
			exit (4);
		} else {
			char clntName[INET_ADDRSTRLEN];
			if (inet_ntop (AF_INET, &their_addr.sin_addr.s_addr, clntName, sizeof (clntName)) != NULL) {
				printf ("Handing client %s/%d\n", clntName, ntohs (their_addr.sin_port));
			} else {
				puts ("Unable to get client address");
			}

			pthread_create (&send_tid, NULL, send_data, (void*)new_fd);
		}

	}


	// done!
on_finished:
	RPlidarDriver::DisposeDriver(drv);
	close (my_socket);
	return 0;
}


void* send_data (void* arg) {
	while (1) {
		rplidar_response_measurement_node_t nodes[360*2];
		size_t   count = _countof(nodes);


		u_result op_result = drv->grabScanData(nodes, count);

		if (IS_OK(op_result)) {
			drv->ascendScanData (nodes, count);
			for (int pos = 0; pos < (int)count; ++pos) {
				short current_angle = (short) ((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f); 
				short current_dist =   (short) (nodes[pos].distance_q2/4.0f);
				char current_quality =  (char) (nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
				//char sync_quality =   (char) ((nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? 'S' : ' ');

				if (current_quality >= 10) {
					sprintf (buf, "%d,%d,%d,", (int)current_angle, (int)current_dist, (int)current_quality);
					int send_result = send ((int)arg, buf, strlen (buf), MSG_NOSIGNAL);
					if (send_result == -1) {
						printf ("send error\n");
						goto thread_end;
					}

					// printf("[%d] [%3d] %-2c theta: %d Dist: %d Q: %d \n", 
					// 		count,
					// 		pos,
					// 		sync_quality,
					// 		current_angle,
					// 		current_dist,
					// 		current_quality);

				}
			}
			send ((int)arg, "\n", 1, MSG_NOSIGNAL);
		}
	}
thread_end:
	printf ("end pthread\n");
	close ((int)arg);
	return 0;
}



