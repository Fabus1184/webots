#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define LEFT 0
#define RIGHT 1
#define TIME_STEP 32  // [ms]

#define NB_GROUND_SENS 3
#define GS_WHITE 900
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2
WbDeviceTag gs[NB_GROUND_SENS]; /* ground sensors */
unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0};

// Motors
WbDeviceTag left_motor, right_motor;

// LEDs
#define NB_LEDS 8
WbDeviceTag led[NB_LEDS];

int lfm_speed[2];

#define LFM_FORWARD_SPEED 200
#define LFM_K_GS_SPEED 0.4

void LineFollowingModule(void)
{

}

int main()
{
		int i, speed[2], ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0}, Mode = 1;

		wb_robot_init();

		char name[20];
		for (i = 0; i < NB_LEDS; i++) {
				sprintf(name, "led%d", i);
				led[i] = wb_robot_get_device(name); /* get a handler to the sensor */
		}

		for (i = 0; i < NB_GROUND_SENS; i++) {
				sprintf(name, "gs%d", i);
				gs[i] = wb_robot_get_device(name); /* ground sensors */
				wb_distance_sensor_enable(gs[i], TIME_STEP);
		}
		// motors
		left_motor = wb_robot_get_device("left wheel motor");
		right_motor = wb_robot_get_device("right wheel motor");
		wb_motor_set_position(left_motor, INFINITY);
		wb_motor_set_position(right_motor, INFINITY);
		wb_motor_set_velocity(left_motor, 0.0);
		wb_motor_set_velocity(right_motor, 0.0);

		for (;;) {  // Main loop
				wb_robot_step(TIME_STEP);

				for (i = 0; i < NB_DIST_SENS; i++)
						ps_offset[i] = PS_OFFSET_SIMULATION[i];
				wb_motor_set_velocity(left_motor, 0);
				wb_motor_set_velocity(right_motor, 0);
				wb_robot_step(TIME_STEP);  // Just run one step to make sure we get correct sensor values
				printf("\n\n\nSwitching to SIMULATION and reseting all BB variables.\n\n");

				// read sensors value
				for (i = 0; i < NB_DIST_SENS; i++)
						ps_value[i] = (((int) wb_distance_sensor_get_value(ps[i]) - ps_offset[i]) < 0) ?
									  0 :
									  ((int) wb_distance_sensor_get_value(ps[i]) - ps_offset[i]);
				for (i = 0; i < NB_GROUND_SENS; i++)
						gs_value[i] = wb_distance_sensor_get_value(gs[i]);

				speed[LEFT] = 0;
				speed[RIGHT] = 0;

				int DeltaS = 0;

				DeltaS = gs_value[GS_RIGHT] - gs_value[GS_LEFT];

				lfm_speed[LEFT] = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
				lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;

				speed[LEFT] = lfm_speed[LEFT];
				speed[RIGHT] = lfm_speed[RIGHT];

				wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
				wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
		}
		return 0;
}
