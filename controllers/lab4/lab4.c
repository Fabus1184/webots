#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#include "tools.h"

WbDeviceTag gyro;
WbDeviceTag gs1;
WbDeviceTag left;
WbDeviceTag right;

int turn_thresh = 45;
int direction = 1;
int turn_increment = 15;
double total_turned = 0;

int main()
{
		setup_lab4(&gyro, &gs1, &left, &right);
		wb_robot_step(TIME_STEP);
		flush();
		printf("START lab4.c\n--------\n");

		while (1) {
				bool on_line = wb_distance_sensor_get_value(gs1) < 500;

				if (on_line) {
						DRIVE:
						total_turned = 0;
						drive_tiles(0.2, left, right);
				} else {
						while (1) {
								bool first = true;
								for (int turned = 0; turned < turn_thresh;) {
										turn(turn_increment * direction, left, right, gyro);
										turned += turn_increment;
										total_turned += turn_increment;
										on_line = wb_distance_sensor_get_value(gs1) < 500;

										if (first && !on_line) direction *= -1;
										if (first) first = 0;
										if (on_line) goto DRIVE;
								}

								turn(direction * -1 * turn_thresh, left, right, gyro);
								total_turned += turn_thresh;
								if (total_turned >= 180) goto FINISH;
								direction *= -1;
						}
				}
		}

		FINISH:
		printf("FINISHED!\n");
		wb_robot_cleanup();
		return 0;
}