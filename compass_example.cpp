/**
 * @file    compass_example.cpp
 * @brief   The main program enabling the robot to enact a navigation protocol 
 * used for the final project in Robotics. The robot
 * will navigate to the bottom of the obstacle course and identify one of the
 * green pillars. After identification, the robot will drive to the pillar, stop
 * and then repeat this process for the second pillar. After identifying both pillars,
 * the robot will return to the top of the obstacle course.
 *
 * @author  Lucas Kaplan
 * @date    5-20-2022
 */

#include "MyRobot.h"

/**
 * @brief Main program.
 */
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}

