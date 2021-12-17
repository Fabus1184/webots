//
// Created by fabian on 30.11.21.
//

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gyro.h>

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#define TIME_STEP 32

extern unsigned long long int TIME;

void flush();

void print_line(bool vals[]);

void update_line(WbDeviceTag sensors[], bool vals[], int n);

void setup_lab5(WbDeviceTag *gyro, WbDeviceTag *gs0, WbDeviceTag *gs1, WbDeviceTag *gs2, WbDeviceTag *left,
				WbDeviceTag *right);

void disttable(WbDeviceTag s);

void gyrotable(WbDeviceTag s);

double realdistance(WbDeviceTag s);

void drive_tiles(double n, WbDeviceTag left, WbDeviceTag right);

void turn(double n, WbDeviceTag left, WbDeviceTag right, WbDeviceTag s);

bool greater(const double a[], double b, int s);

void update(WbDeviceTag sensors[], double val[]);

void count(WbDeviceTag s, int t[], double *l);
