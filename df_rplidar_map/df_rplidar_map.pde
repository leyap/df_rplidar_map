/*
 *******************************************************************************
 * Copyright (C) 2014 DFRobot                                                  *
 *                                                                             *
 * df_rplidar_map, This library provides a quite complete function for         *
 * DFPlayer mini mp3 module.                                                   *
 *                                                                             *
 * This file is part of the DFplayer_Mini_Mp3 library.                         *
 *                                                                             *
 * df_rplidar_map is free software: you can redistribute it and/or             *
 * modify it under the terms of the GNU General Public License as              *
 * published by the Free Software Foundation, either version 3 of              *
 * the License, or any later version.                                          *
 *                                                                             *
 * df_rplidar_map is distributed in the hope that it will be useful,           *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 * GNU General Public License for more details.                                *
 *                                                                             *
 * df_rplidar_map is distributed in the hope that it will be useful,           *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 * GNU General Public License for more details.                                *
 *                                                                             *
 * You should have received a copy of the GNU General Public                   *
 * License along with df_rplidar_map. If not, see                              *
 * <http://www.gnu.org/licenses/>.                                             *
 *                                                                             *
 ******************************************************************************/

/*
 *  name:		df_rplidar_map
 *  version:		0.1
 *  Author:		lisper <lisper.li@dfrobot.com>
 *  Date:		2014-08-06
 *  Description:	processing draw server for rplidar      
 */


import processing.net.*;

int port = 8087;

long  timer;

boolean myServerRunning = true;
int bgColor = 0;
int direction = 1;
int textLine = 60;
byte interesting = 10;
Server myServer;

int data[];
boolean redrawS = true;
boolean circleS = false;
boolean pointS = true;
boolean lineS = false;
boolean breakLineS = true;
boolean connectLineS = true;
boolean pauseS = false;

//color backcolor = color (15, 25, 15);
color backcolor = color (15, 15, 10);

float scale = 10.0;

//
void setup() {
	size(1366, 768);
	// textFont(createFont("Arial", 16));
	myServer = new Server(this, port); // Starts a myServer on port 10002
	background(backcolor);
	strokeWeight (3);
	textSize (48);
	fill (0, 100, 200);
	//frameRate(8);
}




int baud = 0;
int baud1 = 0;

void draw() {
	float xp, yp, x, y, x_save, y_save;

	if (millis () - timer > 1000) {
		timer = millis ();
		textSize (24);
		baud1 = baud;
		baud = 0;
	}



	if (myServerRunning == true && pauseS == false) {
		Client thisClient = myServer.available();
		if (thisClient != null) {
			if (thisClient.available() > 1) {
				String input = thisClient.readStringUntil(interesting); 
				// println (input);
				if (input != null && input.length() > 1) {

					baud ++;
					data = int (split (input, ','));

					if (redrawS == true) {
						background (backcolor);
						//fill (0, 100, 200);
						fill (5, 50, 3);
						textSize (24);
						text (baud1+" Hz", 20, 40);
					}
					translate (width/2, height/2);
					if (connectLineS == true) {
						noFill ();
						//stroke (125, 125, 245);
						stroke (5, 50, 3);
						strokeWeight (1);
						ellipse (0, 0, 30, 30);
						ellipse (0, 0, 300, 300);
						ellipse (0, 0, 600, 600);

						for (int i=0; i < 360; i+=30) {
							float hudu = i*PI/180.0; 
							float axis_x = 300*cos (hudu);
							float axis_y = 300*sin (hudu);
							line (0, 0, axis_x, axis_y);
							textSize (12);
							//fill (0, 100, 200);
							fill (5, 150, 180);
							text (i, axis_x+cos(hudu)*30-12, axis_y+sin(hudu)*30+6);
						}
					}


					xp = data[1] * sin (data[0]*3.14/180.0) / scale;
					yp = data[1] * cos (data[0]*3.14/180.0) / scale;
					x_save = xp; 
					y_save = yp;

					for (int i=0; i<data.length-1; i+=2) {
						float rad = data[i]*3.14/180.0;
						int distance = data[i+1];
						x = distance * sin (-rad) / scale;
						y = distance * cos (rad) / scale;
						//stroke (0, 100, 0);
						//strokeWeight (3);
						strokeWeight (1);
						if (lineS ) {
							stroke (100, 200, 0, 100);

							line (0, 0, x, y);
						}
						if (circleS == true) { 
							if (  breakLineS ? (sqrt ((x-xp)*(x-xp)+(y-yp)*(y-yp)) < 18) : true) {

								stroke (100, 200, 200, 100);
								line (x, y, xp, yp);
							}
							xp = x;
							yp = y;
						}
					}
					if (circleS == true) {
						if (breakLineS ? (sqrt ((x_save-xp)*(x_save-xp)+(y_save-yp)*(y_save-yp)) < 18) : true) {
							stroke (100, 200, 200, 100);						
							line (xp, yp, x_save, y_save);
						}
					}



					for (int i=0; i<data.length-1; i+=2) {
						float rad = data[i]*3.14/180.0;
						int distance = data[i+1];
						x = distance * sin (-rad) / scale;
						y = distance * cos (rad) / scale;
						if (pointS) {
							strokeWeight (5);
							//             stroke (200, 0, 0);
							stroke (100, 200, 200, 100);
							point (x, y);
							//println (x+" "+y);
						}
					}

					stroke (200, 150, 0);
					strokeWeight (1);
					if (lineS == true)
						line (0, 0, x_save, y_save);

					//text("mesage from: " + thisClient.ip() +" "+thisClient.readString (), 10, 10);
				}
			}
		} else {
			//println (".");
		}
	}
}


//
void keyPressed() {
	switch (key) {
		case 's':
			myServer.stop();
			myServerRunning = false;
			break;
		case 'a': 
			myServer.stop();
			myServer = new Server (this, port);
			myServerRunning = true;
			break;
		case '1':
			pointS = !pointS;
			break;
		case '2':
			lineS = !lineS;
			break;
		case '3':
			circleS = !circleS;
			break;
		case '4':
			breakLineS = !breakLineS;
			break;
		case 'r':
			redrawS = true;
			break;
		case 'e':
			redrawS = false;
			break;
		case 'c':
			connectLineS = !connectLineS;
			break;
		case ' ':
			pauseS = !pauseS;
	}
}

//
void mouseWheel(MouseEvent event) {
	float e = event.getCount();
	scale += e/2.0;
	if (scale <= 0.5) 
		scale = 0.5;
	// println(e);
}


