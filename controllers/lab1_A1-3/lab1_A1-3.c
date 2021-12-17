#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/gyro.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 32

void ptable(WbDeviceTag s)
{
  const double *table = wb_distance_sensor_get_lookup_table(s);

  int size = wb_distance_sensor_get_lookup_table_size(s);

  for (int i = 0; i < size * 3; i++)
  {
    if (i % 3 == 0)
      printf("\n");
    printf("%f\t\t\t", table[i]);
  }
  printf("\n");
}

void gtable(WbDeviceTag s)
{
  const double *table = wb_gyro_get_lookup_table(s);

  int size = wb_gyro_get_lookup_table_size(s);

  for (int i = 0; i < size * 3; i++)
  {
    if (i % 3 == 0)
      printf("\n");
    printf("%f\t\t\t", table[i]);
  }
  printf("\n");
}

float distance(WbDeviceTag s)
{
  double value = wb_distance_sensor_get_value(s);

  int size = wb_distance_sensor_get_lookup_table_size(s);
  const double *table = wb_distance_sensor_get_lookup_table(s);

  for (int i = 0; i < size; i++)
  {
    if (value > table[(3 * i) + 1])
    {
      value = table[(3 * i)] + table[(3 * (i - 1))] / table[(3 * (i - 1) + 1)];
      return value;
      break;
    }
  }

  value = NAN;
  printf("%f : %f\n", wb_distance_sensor_get_value(s), value);

  return value;
}

void drive_tiles(int n, WbDeviceTag left, WbDeviceTag right)
{
  float t = 0;
  float vel = 5;

  float stop = n * 5.49504225076930591511 / vel ;

  while (t < stop)
  {
    wb_motor_set_velocity(left, vel);
    wb_motor_set_velocity(right, vel);
    wb_robot_step(TIME_STEP);
    t += (float)TIME_STEP / 1000.0;
  }

  wb_motor_set_velocity(left, 0.0);
  wb_motor_set_velocity(right, 0.0);
}

void turn(double n, WbDeviceTag left, WbDeviceTag right, WbDeviceTag s)
{
  double turned = 0;

  double gyro = wb_gyro_get_values(s)[2] / wb_gyro_get_lookup_table(s)[1] * wb_gyro_get_lookup_table(s)[0];

  float vel = 0.5;

  while (turned > -M_PI_2)
  {
    wb_motor_set_velocity(left, +vel);
    wb_motor_set_velocity(right, -vel);

    wb_robot_step(TIME_STEP);
    
    gyro = wb_gyro_get_values(s)[2] / wb_gyro_get_lookup_table(s)[1] * wb_gyro_get_lookup_table(s)[0];
    turned += (double) TIME_STEP / 1000.0 * gyro; 
  }

  wb_motor_set_velocity(left, 0);
  wb_motor_set_velocity(right, 0);
}

int main(int argc, char **argv)
{
  wb_robot_init();
  printf("SOOS\n");

  WbDeviceTag left = wb_robot_get_device("left wheel motor");
  WbDeviceTag right = wb_robot_get_device("right wheel motor");

  WbDeviceTag s0 = wb_robot_get_device("ps0");
  WbDeviceTag s1 = wb_robot_get_device("ps1");
  WbDeviceTag s2 = wb_robot_get_device("ps2");
  WbDeviceTag s3 = wb_robot_get_device("ps3");
  WbDeviceTag s4 = wb_robot_get_device("ps4");
  WbDeviceTag s5 = wb_robot_get_device("ps5");
  WbDeviceTag s6 = wb_robot_get_device("ps6");
  WbDeviceTag s7 = wb_robot_get_device("ps7");

  WbDeviceTag sensors[] = {s0, s1, s2, s3, s4, s5, s6, s7};

  for (int i = 0; i < 8; i++)
  {
    wb_distance_sensor_enable(sensors[i], TIME_STEP);
  }

  WbDeviceTag gyro = wb_robot_get_device("gyro");
  printf("%f\n", wb_gyro_get_lookup_table(gyro)[2]);

  wb_gyro_enable(gyro, TIME_STEP);

  wb_motor_set_position(left, INFINITY);
  wb_motor_set_position(right, INFINITY);

  wb_motor_set_velocity(left, 0.0);
  wb_motor_set_velocity(right, 0.0);

  wb_robot_step(TIME_STEP * 10);

  drive_tiles(4,left,right);
  turn(1, left, right, gyro);
  drive_tiles(4,left,right);
  turn(1, left, right, gyro);
  drive_tiles(4,left,right);
  turn(1, left, right, gyro);
  drive_tiles(4,left,right);
  turn(1, left, right, gyro);
  

  wb_robot_step(TIME_STEP);

  printf("TEST\n");
  wb_robot_cleanup();
  return 0;
}
