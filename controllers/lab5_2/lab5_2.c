#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define MAGIC_NUMBER 0.00628

#define LEFT 0
#define RIGHT 1
#define TIME_STEP 32

#define NB_GROUND_SENS 3
#define GS_LEFT 0
#define GS_RIGHT 2
WbDeviceTag gs[NB_GROUND_SENS];
unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0};

WbDeviceTag left_motor, right_motor;
int lfm_speed[2];

#define LFM_FORWARD_SPEED 450
#define LFM_K_GS_SPEED 1

int main()
{
		int i, speed[2];

		wb_robot_init();

		char name[20];
		for (i = 0; i < NB_GROUND_SENS; i++) {
				gs[i] = wb_robot_get_device(name);
				wb_distance_sensor_enable(gs[i], TIME_STEP);
		}
		left_motor = wb_robot_get_device("left wheel motor");
		right_motor = wb_robot_get_device("right wheel motor");
		wb_motor_set_position(left_motor, INFINITY);
		wb_motor_set_position(right_motor, INFINITY);
		wb_motor_set_velocity(left_motor, 0.0);
		wb_motor_set_velocity(right_motor, 0.0);

		while (1) {
				wb_robot_step(TIME_STEP);

				wb_motor_set_velocity(left_motor, 0);
				wb_motor_set_velocity(right_motor, 0);
				wb_robot_step(TIME_STEP);

				for (i = 0; i < NB_GROUND_SENS; i++)
						gs_value[i] = wb_distance_sensor_get_value(gs[i]);

				speed[LEFT] = 0;
				speed[RIGHT] = 0;

				int DeltaS = gs_value[GS_RIGHT] - gs_value[GS_LEFT];

				speed[LEFT] = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
				speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;

				wb_motor_set_velocity(left_motor, MAGIC_NUMBER * speed[LEFT]);
				wb_motor_set_velocity(right_motor, MAGIC_NUMBER * speed[RIGHT]);
		}
}
