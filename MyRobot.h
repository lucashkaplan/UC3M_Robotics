#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   The header file for a navigation protocol for the final project in Robotics. The robot
 * will navigate to the bottom of the obstacle course and identify one of the
 * green pillars. After identification, the robot will drive to the pillar, stop
 * and then repeat this process for the second pillar. After identifying both pillars,
 * the robot will return to the top of the obstacle course.
 *
 * @author  Lucas Kaplan
 * @date    5-20-2022
 */

#include <iostream>
#include <limits>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Compass.hpp>
#include <math.h>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED       8

#define NUM_DISTANCE_SENSORS 3
#define DISTANCE_SENSOR_THRESHOLD 150

#define FRONT_THRESHOLD 150
#define SIDE_THRESHOLD 250

#define END_STOP_THRESH 300

#define BOTTOM_DIST_THRESHOLD 125
#define SIDE_BOTTOM_THRESHOLD 300

#define RED "\033[1;31m"
#define GREEN "\x1B[32m"
#define BLUE "\x1B[34m"
#define RESET_COLOR "\033[0m"


//for odometry
#define WHEEL_RADIUS    0.0825 //[=] meters
#define WHEELS_DISTANCE 0.32   //[=] meters
#define ENCODER_TICS_PER_RADIAN 5.0

#define COLOR_THRESHOLD 60
#define GREEN_FRONT_THRESHOLD 80
#define GREEN_SIDE_THRESHOLD 60
#define GREEN_CENTER_THRESHOLD 15

#define CAM_T 40


class MyRobot : public Robot {
    public:
        /**
         * @brief Empty constructor of the class.
         */
        MyRobot();

        /**
         * @brief Destructor of the class.
         */
        ~MyRobot();

        /**
         * @brief Function with the logic of the controller.
         * @param
         * @return
         */
        void run();

        /**
         * @brief Function for converting bearing vector from compass to angle (in degrees).
         * @param in_vector Input vector with the values to convert.
         * @return The value in degrees.
         */
        double convert_bearing_to_degrees(const double* in_vector);
        
        
        /** Strategy to navigate using compass*/
        void compassStrategy();
        
        /** Strategy to navigate using distance sensors*/
        void distSensorStrategy();
       
        /** Strategy to navigate using cameras*/
        void camerasStrategy();
        
        //detecting green cylinders
        bool GreenClose();
        
        //detecting green cylinders
        bool GreenInFront();
        
        //detecting yellow line at end
        bool YellowLine();


    private:
        // Infinity constant
        const double _infinity = numeric_limits<double>::infinity();
        
        // time step
        int _time_step;
        
        // velocities
        double _left_speed, _right_speed;
        
        //value of the compass
        double compass_angle;
        
        //values of distance sensors
        double ir_front, ir_left, ir_right;


        // Sensors
        Compass *_my_compass;
        
        // Motors
        Motor* _left_wheel_motor;
        Motor* _right_wheel_motor;
        
        //Distance sensors
        DistanceSensor* distanceSensors[NUM_DISTANCE_SENSORS];
        
        //states for the controller
        enum Mode{
          FORWARD,
          TURNING_RIGHT,
          TURNING_LEFT,
          WALLFOLLOW,
          STOP,
          SPIN,
          REVERSE_SPIN
          };
          
        Mode _mode;
        
        
        float _x, _y;   // [=] meters
        float _theta;   // [=] rad
        float _odometriaAcumuladaRuedaDerecha, _odometriaAcumuladaRuedaIzquierda;
        
        
        // Motor Position Sensor
        PositionSensor* _left_wheel_sensor;
        PositionSensor* _right_wheel_sensor;
        
        
        /**
         * @brief Updates the odometry of the robot in meters and radians. The atributes _x, _y, _theta are updated.
         */
        void compute_odometry();
        
        /** To finish the program */
        bool goalReached;
        
        //for determining strategy used
        int bottomReached;
        
        int DESIRED_ANGLE;
        
        
        // Camera sensors
        Camera* _forward_camera;
        Camera* _spherical_camera;
        
        
        //size of the image cameras
        int image_width_f, image_height_f, image_width_s, image_height_s;
        
        const double pi = 3.1416;
};

#endif