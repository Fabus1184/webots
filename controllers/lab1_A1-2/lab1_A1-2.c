#include <webots/robot.h>
#include <stdio.h>
#include <unistd.h>
int main() {
  // Initialize Webots
  wb_robot_init();
  while(wb_robot_step(32) != -1) {
    printf("Hello World!\n");
    sleep(1);
    }
  wb_robot_cleanup();
  return 0;
}