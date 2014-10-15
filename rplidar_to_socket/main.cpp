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
#include <stdint.h>
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


//
struct pthread_arg {
	int fd;
	char name[INET_ADDRSTRLEN];
	uint16_t port;
};


//
RPlidarDriver * drv ;

int sock;
int pthread_num;
int pthread_num_need;

char buf[MAXBUF+1];


rplidar_response_measurement_node_t nodes[360*2];
size_t   count ;

pthread_mutex_t counter_lock = PTHREAD_MUTEX_INITIALIZER;

void* send_data (void *arg);
void* scan_data (void *arg);
int createTCP (uint16_t the_port);

//
int createTCP (uint16_t the_port) {
	int mysocket;
	struct sockaddr_in addr;
	if ((mysocket = socket (AF_INET, SOCK_STREAM, 0)) == -1) {
		perror ("error in socket()\n");
		exit (-1);
	}
	memset (&addr, 0, sizeof (addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons (the_port);
	addr.sin_addr.s_addr = INADDR_ANY;
	if (bind (mysocket, (struct sockaddr *)&addr, (socklen_t)sizeof (struct sockaddr)) == -1) {
		perror ("error in bind()\n");
		exit (-1);
	}

	if (listen (mysocket, 10) == -1) {
		perror ("error in listen()\n");
		exit (-1);
	}
	return mysocket;
}


//
void start_net_server (uint16_t port) {
	pthread_t send_tid;
	pthread_t data_tid;
	struct sockaddr_in their_addr;
	sock = createTCP (port);
	printf ("wait for connect\n");

	pthread_create (&data_tid, NULL, scan_data, NULL);
	//
	socklen_t len;
	len = sizeof (struct sockaddr);
	while (1) {
		int new_fd = accept (sock, (struct sockaddr *) &their_addr, &len);
		if (new_fd == -1) {
			perror ("[error in accept]\n");
			exit (4);
		} else {
			pthread_mutex_lock (&counter_lock);
			pthread_num++;
			struct pthread_arg arg;
			arg.fd = new_fd;

			if (inet_ntop (AF_INET, &their_addr.sin_addr.s_addr, arg.name, sizeof (arg.name)) != NULL) {
				arg.port = ntohs (their_addr.sin_port);
				printf ("Handing client %s/%d, online is %d\n", arg.name, arg.port, pthread_num);
			} else {
				puts ("Unable to get client address");
			}
			pthread_mutex_unlock (&counter_lock);

			pthread_create (&send_tid, NULL, send_data, (void*)&arg);
		}
	}
	close (sock);
}

//
void* send_data (void* arg) {
	int send_result;
	int min_distance ;
	int min_angle = 0;
	struct pthread_arg myarg = *(struct pthread_arg*)arg;

	while (1) {
		pthread_mutex_lock (&counter_lock);
		if (!pthread_num_need) {
			pthread_mutex_unlock (&counter_lock);
			continue;
		}
		pthread_num_need --;
		min_distance = 1000;
		for (int pos = 0; pos < (int)count; ++pos) {
			short current_angle = (short) ((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f); 
			short current_dist =   (short) (nodes[pos].distance_q2/4.0f);
			char current_quality =  (char) (nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
			//char sync_quality =   (char) ((nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? 'S' : ' ');

			if (current_quality >= 10 && current_dist > 10) {
				if (current_dist < min_distance) {
					min_distance = current_dist;
					min_angle = current_angle;
				}
				sprintf (buf, "%d,%d,%d,", (int)current_angle, (int)current_dist, (int)current_quality);
				send_result = send ((int)myarg.fd, buf, strlen (buf), MSG_NOSIGNAL);
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
	///	printf ("min_dist=%d, min_angle=%d\n", min_distance, min_angle);
		send_result = send ((int)myarg.fd, "\n", 1, MSG_NOSIGNAL);
		if (send_result == -1) {
			printf ("send error\n");
			goto thread_end;
		}
		pthread_mutex_unlock (&counter_lock);
	}
thread_end:
	pthread_num--;
	printf ("end pthread, online is %d\n", pthread_num);
	pthread_mutex_unlock (&counter_lock);
	close ((int)myarg.fd);
	return 0;
}

//
void* send_data (void* arg) {
	int send_result;
	int min_distance ;
	int min_angle = 0;
	struct pthread_arg myarg = *(struct pthread_arg*)arg;

	while (1) {
		pthread_mutex_lock (&counter_lock);
		if (!pthread_num_need) {
			pthread_mutex_unlock (&counter_lock);
			continue;
		}
		pthread_num_need --;
		min_distance = 1000;
		for (int pos = 0; pos < (int)count; ++pos) {
			short current_angle = (short) ((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f); 
			short current_dist =   (short) (nodes[pos].distance_q2/4.0f);
			char current_quality =  (char) (nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
			//char sync_quality =   (char) ((nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? 'S' : ' ');

			if (current_quality >= 10 && current_dist > 10) {
				if (current_dist < min_distance) {
					min_distance = current_dist;
					min_angle = current_angle;
				}
				sprintf (buf, "%d,%d,%d,", (int)current_angle, (int)current_dist, (int)current_quality);
				send_result = send ((int)myarg.fd, buf, strlen (buf), MSG_NOSIGNAL);
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
	///	printf ("min_dist=%d, min_angle=%d\n", min_distance, min_angle);
		send_result = send ((int)myarg.fd, "\n", 1, MSG_NOSIGNAL);
		if (send_result == -1) {
			printf ("send error\n");
			goto thread_end;
		}
		pthread_mutex_unlock (&counter_lock);
	}
thread_end:
	pthread_num--;
	printf ("end pthread, online is %d\n", pthread_num);
	pthread_mutex_unlock (&counter_lock);
	close ((int)myarg.fd);
	return 0;
}


//
void* scan_data (void* arg) {
	while (1) {
		pthread_mutex_lock (&counter_lock);
		u_result op_result = drv->grabScanData(nodes, count);	

		if (IS_OK(op_result)) {
			pthread_num_need = pthread_num;
			drv->ascendScanData (nodes, count);	//sort data
		}
		pthread_mutex_unlock (&counter_lock);
	}
	return NULL;
}

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

	const char * opt_com_path = NULL;
	_u32         opt_com_baudrate = 115200;
	uint16_t port = 8087;

	// read serial port from the command line...
	if (argc>1) 
		opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

	// read baud rate from the command line if specified...
	if (argc>2) 
		opt_com_baudrate = strtoul(argv[2], NULL, 10);
	if (argc>3) {
		port = strtoul(argv[3], NULL, 10);
		//port = atoi (argv[3]);
		printf ("port: %d\n", port);
		}

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


	count = _countof(nodes);
	//printf ("%d\n", (int)count);
	// start scan...
	drv->startScan();

	// fetech result and print it out...

	start_net_server (port);

	// done!
on_finished:
	RPlidarDriver::DisposeDriver(drv);
	return 0;
}



