#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 8

#define MAX_SPEED 6.28

WbDeviceTag compass;

/*
 * copied from: https://cyberbotics.com/doc/reference/compass
 * */
double getRobotBearing()
{
    /* calculate bearing angle in degrees */
    const double *north = wb_compass_get_values(compass);
    double rad = atan2(north[0], north[2]);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < 0.0)
        bearing = bearing + 360.0;

    return bearing;
}

double cartesianCalcThetaDot(double heading, double destinationTheta) {
    double theta_dot = destinationTheta - heading;

    if (theta_dot > 180)
        theta_dot = -(360-theta_dot);
    else if (theta_dot < -180)
        theta_dot = (360+theta_dot);

    return theta_dot;
}

// entry point of the controller
int main(int argc, char **argv) {
  // initialize the Webots API
  wb_robot_init();
  
  // internal variables
  bool turn = true;
  const double *coords;
  double bearing;
  double destination[2] = {-4, -4};
  
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // prime compass and gps, min 2 steps
  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  // feedback loop: step simulation until an exit event is received
  while (wb_robot_step(TIME_STEP) != -1) {
    
    bearing = getRobotBearing();
    coords = wb_gps_get_values(gps);
    double theta = atan2(destination[1] - coords[1],
                         destination[0] - coords[0]) * 180 / M_PI;
  
    if(turn) {
      double heading = -cartesianCalcThetaDot(bearing, theta);
  
      printf("gps coords x: %f y: %f bearing: %f° heading: %f° θ: %f°\n", 
              coords[0], coords[1], bearing, heading, theta);
 
      float duration = (double)abs(theta) / 255;

      if (0 > theta) { // left +y
        wb_motor_set_velocity(left_motor, MAX_SPEED);
        wb_motor_set_velocity(right_motor, -MAX_SPEED);
      }
      else if (0 < theta) { // right -y
        wb_motor_set_velocity(left_motor, -MAX_SPEED);
        wb_motor_set_velocity(right_motor, MAX_SPEED);
      }

      double start_time = wb_robot_get_time() + duration;
//      printf("start_time %f duration %f\n", start_time, duration);
      do {
        coords = wb_gps_get_values(gps);
        theta = atan2(destination[1] - coords[1],
                      destination[0] - coords[0]) * 180 / M_PI;
        wb_robot_step(TIME_STEP);
//        printf("gps coords x: %f y: %f bearing: %f° heading: %f° θ: %f time %f°\n", 
//                coords[0], coords[1], bearing, heading, theta, wb_robot_get_time());
      }
      while (wb_robot_get_time() < start_time);
      
      printf("gps coords x: %f y: %f bearing: %f° heading: %f° θ: %f°\n", 
              coords[0], coords[1], bearing, heading, theta);
      wb_motor_set_velocity(left_motor, MAX_SPEED);
      wb_motor_set_velocity(right_motor, MAX_SPEED);
      turn = false;
    }
//    printf("gps coords x: %f y: %f bearing: %f° θ: %f°\n", 
//            coords[0], coords[1], bearing, theta);

    if((int)destination[0] == (int)coords[0] || (int)destination[1] == (int)coords[1]) {
      turn = true;
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
      printf("gps coords x: %f y: %f bearing: %f° θ: %f°\n", 
              coords[0], coords[1], bearing, theta);
      break;
    }

  }

  wb_camera_disable(camera);
  wb_compass_disable(compass);
  wb_gps_disable(gps);

  // cleanup the Webots API
  wb_robot_cleanup();
  return 0; //EXIT_SUCCESS
}
