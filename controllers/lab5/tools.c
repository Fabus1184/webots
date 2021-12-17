//
// Created by fabian on 30.11.21.
//

#include "tools.h"

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gyro.h>

void flush()
{
		printf("\e[1;1H\e[2J\n");
}

void print_line(bool vals[])
{
		printf("----\n");
		for (int i = 0; i < 3; i++) {
				printf("%d\n", vals[i]);
		}
}

void update_line(WbDeviceTag sensors[], bool vals[], int n)
{
		for (int i = 0; i < n; i++) {
				vals[i] = wb_distance_sensor_get_value(sensors[i]) < 500;
		}
}

void setup_lab5(WbDeviceTag *gyro, WbDeviceTag *gs0, WbDeviceTag *gs1, WbDeviceTag *gs2, WbDeviceTag *left,
				WbDeviceTag *right)
{
		wb_robot_init();

		*gyro = wb_robot_get_device("gyro");
		wb_gyro_enable(*gyro, TIME_STEP);

		*gs0 = wb_robot_get_device("gs0");
		wb_distance_sensor_enable(*gs0, TIME_STEP);

		*gs1 = wb_robot_get_device("gs1");
		wb_distance_sensor_enable(*gs1, TIME_STEP);

		*gs2 = wb_robot_get_device("gs2");
		wb_distance_sensor_enable(*gs2, TIME_STEP);

		*left = wb_robot_get_device("left wheel motor");
		*right = wb_robot_get_device("right wheel motor");

		wb_motor_set_position(*left, INFINITY);
		wb_motor_set_position(*right, INFINITY);
		wb_motor_set_velocity(*left, 0.0);
		wb_motor_set_velocity(*right, 0.0);
}

void disttable(WbDeviceTag s)
{
		const double *table = wb_distance_sensor_get_lookup_table(s);

		int size = wb_distance_sensor_get_lookup_table_size(s);

		for (int i = 0; i < size * 3; i++) {
				if (i % 3 == 0)
						printf("\n");
				printf("%f\t\t\t", table[i]);
		}
		printf("\n");
}

void gyrotable(WbDeviceTag s)
{
		const double *table = wb_gyro_get_lookup_table(s);

		int size = wb_gyro_get_lookup_table_size(s);

		for (int i = 0; i < size * 3; i++) {
				if (i % 3 == 0)
						printf("\n");
				printf("%f\t\t\t", table[i]);
		}
		printf("\n");
}

double realdistance(WbDeviceTag s)
{
		double value = wb_distance_sensor_get_value(s);

		int size = wb_distance_sensor_get_lookup_table_size(s);
		const double *table = wb_distance_sensor_get_lookup_table(s);

		for (int i = 0; i < size; i++) {
				if (value > table[(3 * i) + 1]) {
						value = table[(3 * i)] + table[(3 * (i - 1))] / table[(3 * (i - 1) + 1)];
						return value;
				}
		}

		value = NAN;
		printf("%f : %f\n", wb_distance_sensor_get_value(s), value);

		return value;
}

void drive_tiles(double n, WbDeviceTag left, WbDeviceTag right)
{
		double t = 0;
		double vel = wb_motor_get_max_velocity(left);

		double stop = n * 5.49504225076930591511 / vel;

		while (t < stop) {
				wb_motor_set_velocity(left, vel);
				wb_motor_set_velocity(right, vel);
				wb_robot_step(TIME_STEP);
				TIME++;
				t += TIME_STEP / 1000.0;
		}

		wb_motor_set_velocity(left, 0.0);
		wb_motor_set_velocity(right, 0.0);
		wb_robot_step(TIME_STEP);
		TIME++;
}

void turn(double n, WbDeviceTag left, WbDeviceTag right, WbDeviceTag s)
{
		if (n == 0) return;

		double turned = 0;

		double gyro = wb_gyro_get_values(s)[2] / wb_gyro_get_lookup_table(s)[1] * wb_gyro_get_lookup_table(s)[0];

		double vel = 5;

		double to_turn = 2 * M_PI * n / 360;

		while (fabs(turned) < fabs(to_turn)) {
				wb_motor_set_velocity(left, (to_turn < 0 ? 1 : -1) * vel);
				wb_motor_set_velocity(right, (to_turn > 0 ? 1 : -1) * vel);

				wb_robot_step(TIME_STEP);
				TIME++;

				gyro = wb_gyro_get_values(s)[2] / wb_gyro_get_lookup_table(s)[1] * wb_gyro_get_lookup_table(s)[0];
				turned += TIME_STEP / 1000.0 * gyro;
		}

		wb_motor_set_velocity(left, 0);
		wb_motor_set_velocity(right, 0);
		wb_robot_step(TIME_STEP);
		TIME++;
}

bool greater(const double a[], double b, int s)
{
		for (int i = 0; i < s; i++) {
				if (a[i] > b)
						return true;
		}
		return false;
}

void update(WbDeviceTag sensors[], double val[])
{
		for (int i = 0; i < 8; i++) {
				val[i] = wb_distance_sensor_get_value(sensors[i]);
		}
}

void count(WbDeviceTag s, int t[], double *l)
{
		double value = wb_distance_sensor_get_value(s);
		double diff = *l - value;

		if (fabs(diff) < 300)
				return;

		if (value < 500)
				t[0] += 1;

		else
				t[1] += 1;

		*l = value;

		flush();
		printf("Schwarz: %i, Weiß: %i\n", t[0], t[1]);
}
