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
#include <webots/pen.h>
#include <webots/robot.h>

#define TIME_STEP 32

#define MAX_SPEED 6.28
#define TURN_SPEED (MAX_SPEED / 100)

#define IMAGE 1024
#define FLOOR 7.5
#define UNIT (IMAGE / FLOOR)

#define PATHS 3
#define MAX_POINTS 100  // 50 x 2 fixed size for array allocation
#define LINE_LEN 32

const double HALF_IMAGE = IMAGE / 2;
const double RANGE = FLOOR / 2;
const double SCALE = FLOOR / IMAGE;

WbDeviceTag compass;
WbDeviceTag gps;


/* copied from: https://cyberbotics.com/doc/reference/compass */
double getRobotBearing()
{
    /* calculate bearing angle in degrees */
    const double *north = wb_compass_get_values(compass);
    double rad = atan2(north[0], north[1]);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < 0.0)
        bearing = bearing + 360.0;

    return bearing;
}

double convertCoord(int coord) {
    double c = (coord - HALF_IMAGE) * SCALE; // % * area / 2
    return c;
}

double getTheta(double dest[], const double coords[]) {
  double theta = atan2(dest[1] - coords[1], dest[0] - coords[0]) * 180 / M_PI;

  return theta;
}

bool checkDestination(double dest[], const double coords[]) {
  double error = 0.02;
  bool x =  false;
  bool y =  false;

  return ((dest[0] - error) < coords[0] && coords[0] < (dest[0] + error)  &&  
          (dest[1] - error) < coords[1] && coords[1] < (dest[1] + error));
}

// entry point of the controller
int main(int argc, char **argv) {
  
  // internal variables
  bool turn = true;
  const double *coords;
  double dest[2] = {0};
  double bearing = 0;
  double heading = 0;
  double theta = 0;
  double write = false;

  FILE *fp = NULL;
  const char* FILES[PATHS] = {"prm_path_1.txt", "prm_path_2.txt", "prm_path_3.txt"};
  char line[LINE_LEN + 1] = {0};
  int destinations[PATHS][MAX_POINTS] = {{-1}}; // :( not initializing all elements
  
    for(int f = 0; f < 3 ; f++) {

        for (int i = 0; i < MAX_POINTS; i++)    // :( initializing all elements
            destinations[f][i] = -1;

        printf("\rReading file %s\n", FILES[f]);
        fp = fopen(FILES[f], "r");
        if (!fp) {
            printf("File %s not found or could not be read\n", FILES[f]);
            return EXIT_FAILURE;
        }

        for (int i = 0; fgets(line, LINE_LEN, fp); i+=2) {
            sscanf(strtok(line, ","),"%d",&destinations[f][i]);
            sscanf(strtok(NULL, "\n"),"%d",&destinations[f][i+1]);
        }

        fclose(fp);
    }

  // initialize the Webots API
  wb_robot_init();

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  WbDeviceTag pen = wb_robot_get_device("pen");
  
  // prime compass and gps, min 2 steps
  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);
  
    for(int f = 0; f < 3 ; f++) {

      coords = wb_gps_get_values(gps);
      bearing = getRobotBearing();
      heading = bearing;
      printf("gps x: %f y: %f z: %f compass: %f° heading: %f° <----- initial\n", 
              coords[0], coords[1], coords[2], bearing, heading);
 
      // turn off pen between paths
      write = false;
      wb_pen_write(pen, write);

      for(int d = 0; d < MAX_POINTS && destinations[f][d] != -1; d+=2) {

        dest[0] = convertCoord(destinations[f][d]); // x
        dest[1] = -convertCoord(destinations[f][d+1]); // y
        printf("next x: %d y: %d dest_x: %f dest_y: %f\n",
              destinations[f][d], destinations[f][d+1], dest[0], dest[1]);

        coords = wb_gps_get_values(gps);
        double theta0 = getTheta(dest, coords);
        double theta1 = theta0 - theta;
        theta = theta0;

        // feedback loop: step simulation until an exit event is received
        while (wb_robot_step(TIME_STEP) != -1) {

          bearing = getRobotBearing();

          if(turn) {

            heading += theta1;
            if(heading > 360)
            {
              heading -= 360;
              theta -= 360;
            }

            printf("gps x: %f y: %f compass: %f° heading: %f° θ: %f° θ0: %f° θ1: %f° <----- turn", 
                    coords[0], coords[1], bearing, heading, theta, theta0, theta1);

            if (theta1 > 0) { // left
              printf(" Left\n");
              wb_motor_set_velocity(left_motor, -TURN_SPEED);
              wb_motor_set_velocity(right_motor, TURN_SPEED);
            } else { // right
              printf(" Right\n");
              wb_motor_set_velocity(left_motor, TURN_SPEED);
              wb_motor_set_velocity(right_motor, -TURN_SPEED);
            }

            wb_pen_write(pen, false);
            while ((trunc(bearing * 10.0) / 10.0 ) != (trunc(heading * 10.0) / 10.0 )) {
              bearing = getRobotBearing();
              // printf("compass: %f° heading: %f° \n", (trunc(bearing * 10.0) / 10.0 ) , (trunc(heading * 10.0) / 10.0 ));
              wb_robot_step(TIME_STEP);
            }
            wb_pen_write(pen, write);

            // printf("gps x: %f y: %f compass: %f° heading: %f° θ: %f° θ0: %f° θ1: %f°\n", 
                    // coords[0], coords[1], bearing, heading, theta, theta0, theta1);
            wb_motor_set_velocity(left_motor, MAX_SPEED);
            wb_motor_set_velocity(right_motor, MAX_SPEED);
            turn = false;
          }
          // printf("gps x: %f y: %f compass: %f° heading: %f° θ: %f°\n", 
          //         coords[0], coords[1], bearing, heading, theta);

          if(checkDestination(dest,coords)) {
            write = turn = true;
            wb_pen_write(pen, write);
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);
            printf("gps x: %f y: %f compass: %f° heading: %f° θ: %f° <----- destination\n", 
                    coords[0], coords[1], bearing, heading, theta);
            break;
          }
        }
      }
  }

  wb_camera_disable(camera);
  wb_compass_disable(compass);
  wb_gps_disable(gps);

  // cleanup the Webots API
  wb_robot_cleanup();
  return 0; //EXIT_SUCCESS
}
