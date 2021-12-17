#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#include "tools.h"

WbDeviceTag gyro;
WbDeviceTag gs0;
WbDeviceTag gs1;
WbDeviceTag gs2;
WbDeviceTag left;
WbDeviceTag right;

int turn_thresh = 40;
int direction = 1;
int turn_increment = 5;
double total_turned = 0;

unsigned long long TIME = 0;

int main()
{
		setup_lab5(&gyro, &gs0, &gs1, &gs2, &left, &right);
		wb_robot_step(TIME_STEP);
		TIME++;

		WbDeviceTag sensors[] = {gs0, gs1, gs2};
		bool on[] = {0, 0, 0};
		int passed = 0;

		flush();
		printf("START lab5.c\n--------\n");

		drive_tiles(0.5, left, right);

		while (1) {
				if (on[0] && on[1] && on[2]) {
						if (passed++ == 8) goto FINISH;
						drive_tiles(0.1, left, right);
				}

				if (on[1]) {
						update_line(sensors, on, 3);
						DRIVE:
						drive_tiles(0.1, left, right);
						update_line(sensors, on, 3);
				} else {
						total_turned = 0;
						for (int turned = 0; turned < turn_thresh;) {
								if (on[0]) {
										//RECHTS
										direction = 1;
								}
								if (on[2]) {
										//LINKS
										direction = -1;
								}

								turn(turn_increment * direction, left, right, gyro);
								turned += turn_increment;
								total_turned += turn_increment;
								update_line(sensors, on, 3);

								if (on[1]) goto DRIVE;
						}
						direction *= -1;
						turn(turn_thresh * direction, left, right, gyro);

				}
		}

		FINISH:
		printf("FINISHED!\n");
		printf("%.2f sekunden\n", (float) TIME * TIME_STEP / 1000);
		wb_robot_cleanup();
		return 0;
}