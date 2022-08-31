/**
 * @file    MyRobot.cpp
 * @brief   A navigation protocol for the final project in Robotics. The robot
 * will navigate to the bottom of the obstacle course and identify one of the
 * green pillars. After identification, the robot will drive to the pillar, stop
 * and then repeat this process for the second pillar. After identifying both pillars,
 * the robot will return to the top of the obstacle course.
 *
 * @author  Lucas Kaplan
 * @date    5-20-2022
 */

#include "MyRobot.h"

 //////////////////////////////////////////////

MyRobot::MyRobot() : Robot() {
    // init default values
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    //for determining strategy used
    bottomReached = 0;

    //desired angle intially moves robot down
    DESIRED_ANGLE = 270; //down = -90 = 270

    // get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");

    distanceSensors[0] = getDistanceSensor("ds0");
    distanceSensors[0]->enable(_time_step);
    distanceSensors[1] = getDistanceSensor("ds3");
    distanceSensors[1]->enable(_time_step);
    distanceSensors[2] = getDistanceSensor("ds12");
    distanceSensors[2]->enable(_time_step);


    //to control the robot on velicity
    // set position to infinity, to allow velocity control 
    _left_wheel_motor->setPosition(_infinity);
    _right_wheel_motor->setPosition(_infinity);

    // set velocity to 0 
    _left_wheel_motor->setVelocity(0.0);
    _right_wheel_motor->setVelocity(0.0);

    _mode = FORWARD; //starting mode is forward

    //for odometry
    _x = _y = _theta = 0.0;
    _odometriaAcumuladaRuedaDerecha = _odometriaAcumuladaRuedaIzquierda = 0.0;

    // Motor Position Sensor initialization
    _left_wheel_sensor = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");

    _left_wheel_sensor->enable(_time_step);
    _right_wheel_sensor->enable(_time_step);

    // get cameras and enable them
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot() {
    // disable devices
    _my_compass->disable();
    
    for(int i = 0; i<3; i++){
      distanceSensors[i]->disable();
    }

    _left_wheel_sensor->disable();
    _right_wheel_sensor->disable();

    // disable camera devices
    _forward_camera->disable();
    _spherical_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run() {
    // get size of images for forward camera
    image_width_f = _forward_camera->getWidth();
    image_height_f = _forward_camera->getHeight();
    cout << "Size of forward camera image: " << image_width_f << ", " << image_height_f << endl;
    cout << "Mode: " << _mode << endl;

    int numCylinders = 0;
    double cylinder1_y = 1000, cylinder1_theta = 1000;
    int greenFound = 0, nearCylinder = 0, forwardAfterSpin = 0;
    bool endProcess = 0;
    int stop_cnt = 0;

    while (step(_time_step) != -1) {

        //print odometry
        compute_odometry();
        cout << "ODOMETRY_INFORMATION: x->" << _x << "; y->" << _y << "; theta-> " << _theta << endl;


        //read compass and distance sensors values
        const double* compass_values = _my_compass->getValues();
        compass_angle = convert_bearing_to_degrees(compass_values);

        ir_front = distanceSensors[0]->getValue();
        ir_left = distanceSensors[1]->getValue();
        ir_right = distanceSensors[2]->getValue();
        
        cout << "compass: " << RED << compass_angle << RESET_COLOR <<
            " ir_front: " << RED << ir_front << RESET_COLOR <<
            " ir_left: " << RED << ir_left << RESET_COLOR <<
            " ir_right: " << RED << ir_right << RESET_COLOR << endl;
        
        cout << "Desired angle: " << DESIRED_ANGLE << endl;
        cout << "Bottom Reached: " << bottomReached << endl;
        cout << "Stop count: " << stop_cnt << endl;
        
        cout << "Mode: " << _mode << endl;
        
        //after robot has reached bottom and then sees yellow line
        if (bottomReached == 2 && (YellowLine() || endProcess)) {
            //move forward until close to wall in front
            if(ir_front > END_STOP_THRESH){
              cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
              cout << "Goal accomplished!!!" << endl;
              cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
              
              _mode = STOP;
            }
            
            else{
               cout << BLUE << "Coming to stop at end" << RESET_COLOR << endl;
               
               endProcess = 1;
               _mode = FORWARD;
            }
        }

        //after both cylinders found, navigate to top
        if (numCylinders == 2) {
            DESIRED_ANGLE = 90;
            bottomReached = 2;
        }
       
        //STARTING FROM TOP
        //once robot has reached bottom, check for green cylinders
        //use yellow line on bottom to indicate bottom
        if((abs(_x) > 7 && YellowLine() && bottomReached == 0) 
            || bottomReached == 1 || bottomReached == 3){
           
           //once robot identifies yellow line on bottom, move forward for a little
           if (stop_cnt < 15 && (bottomReached == 0 || bottomReached == 3)) {
                cout << BLUE "\nBottom reached for 1st time!" RESET_COLOR << endl;
                
                _mode = FORWARD;
                stop_cnt++;
                
                bottomReached = 3; //robot has identified bottom, but still moving there
                
                continue;
            }
            
            //flag that robot has reached bottom
            bottomReached = 1;
           
            cout << RED << "Bottom Reached!" RESET_COLOR << endl;
            cout << "No. of Cylinders: " << numCylinders << endl;
            
            //if cylinder hasn't been detected
            if (!greenFound) {
                //if 1st cylinder hasn't been found, spin CCW
                if(numCylinders == 0){
                   _mode = SPIN;
                  cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
                  cout << "\n\nSpinning \n\n" << endl;
                  cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
                }
                //otherwise, spin CW
                else if(numCylinders == 1){
                  //if robot has already spun around completely, move forward and spin again
                  if(((abs(_theta) <= abs(cylinder1_theta) + (2*pi) + (2*pi/2.5)) &&
                     (abs(_theta) >= abs(cylinder1_theta) + (2*pi) + (pi/2.5))) ||
                     forwardAfterSpin){
                         if(ir_front < BOTTOM_DIST_THRESHOLD){
                           _mode = FORWARD;
                           
                           cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
                           cout << "Moving forward after spinning" << endl;
                           cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
                           
                           cout << "\nCYLINDER 1 COORDS" << endl;
                           cout << "y-coordinate: " << cylinder1_y << endl;
                           cout << "theta: " << cylinder1_theta << endl;
                           
                           forwardAfterSpin = 1;
                           
                           continue;
                         }
                  }
                   
                    _mode = REVERSE_SPIN;
                    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
                    cout << "\n\nReverse Spinning \n\n" << endl;
                    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
                }
                
                if (GreenInFront()) {
                   greenFound = 1;
                }
                
                //reset stop count
                //will be skipped if stop count changed
                stop_cnt = 0;
                forwardAfterSpin = 0;
            }

            //if cylinder has been detected but it's far away
            else if(greenFound && !nearCylinder) {
                _mode = FORWARD;

                cout << "\nMoving towards cylinder" << endl;

                //if robot is near cylinder, set nearCylinder flag high
                if (GreenClose()) {

                    //if it's first cylinder, record odometry
                    if (numCylinders == 0) {
                      cylinder1_y = _y;
                      cylinder1_theta = _theta;
                      cout << GREEN << "\nFirst cylinder found!" << RESET_COLOR <<endl;
                      cout << "y-coordinate: " << cylinder1_y << endl;
                      cout << "theta: " << cylinder1_theta << endl;
                      
                      nearCylinder = 1;
                    }
                    //if it's second cylinder, only set near cylinder flag high 
                    //if odometry has changed significantly
                    else if((_y > cylinder1_y + 0.5 || _y < cylinder1_y - 0.5) && 
                            (_theta > cylinder1_theta + 0.5 || _theta < cylinder1_theta - 0.5)){
                        nearCylinder = 1;
                        cout << BLUE << "\nNew object far enough from cylinder 1" << RESET_COLOR << endl;
                        cout << "\nCYLINDER 1 COORDS" << endl;
                        cout << "y-coordinate: " << cylinder1_y << endl;
                        cout << "theta: " << cylinder1_theta << endl;
                    }
                    //otherwise, unregister cylinder found
                    else{
                        greenFound = 0;
                        cout << BLUE << "\nNew object too close to cylinder 1" << RESET_COLOR << endl;
                        cout << "\nCYLINDER 1 COORDS" << endl;
                        cout << "y-coordinate: " << cylinder1_y << endl;
                        cout << "theta: " << cylinder1_theta << endl;
                    }
                    
                    cout << "\nVery close to cylinder" << endl;
                }
            }

            else if (greenFound && nearCylinder) {
                if (stop_cnt < 50) {
                    _mode = STOP;
                    stop_cnt++;
                    cout << "Stop count: " << stop_cnt << endl;
                }
                else {
                    //reset cylinder flag, stop count, and nearCylinder flag
                    greenFound = 0;
                    stop_cnt = 0;
                    nearCylinder = 0;
                    
                    numCylinders++; //increment number of cylinders
                }
                cout << BLUE << "\nStopped at cylinder" << RESET_COLOR <<endl;
            }
        }

        //if robot is not at bottom and is not at end, navigate
        if(bottomReached != 1 && !endProcess){
            //if no obstacles, navigate with compass
            if ((ir_front < FRONT_THRESHOLD) &&
                (ir_left < SIDE_THRESHOLD) &&
                (ir_right < SIDE_THRESHOLD)) {
                
                cout << GREEN << "\nUsing compass strategy" RESET_COLOR << endl;
    
                compassStrategy();
            }
    
            //if obstacles, navigate with distance sensors
            else {
                distSensorStrategy();
    
                cout << BLUE << "\nUsing distance sensor strategy" RESET_COLOR << endl;
            }
        }
        

        //speeds for each of robot's modes
        switch (_mode) {
          case FORWARD:
              _left_speed = MAX_SPEED;
              _right_speed = MAX_SPEED;
              break;
          
          case TURNING_RIGHT:
              //cout << "Turning right"<<endl;
              _left_speed = MAX_SPEED;
              _right_speed = MAX_SPEED / 3;
              break;
  
          case TURNING_LEFT:
              //cout << "Turning left"<<endl;
              _left_speed = MAX_SPEED / 3;
              _right_speed = MAX_SPEED;
              break;
  
          case WALLFOLLOW:
              //cout << "Wallfollow"<<endl;
              _left_speed = MAX_SPEED;
              _right_speed = MAX_SPEED;
              break;
  
          case STOP:
              _left_speed = 0;
              _right_speed = 0;
              break;
  
          case SPIN:
              _left_speed = 0;
              _right_speed = MAX_SPEED / 5;
              break;
          
          case REVERSE_SPIN:
              _left_speed = MAX_SPEED / 6;
              _right_speed = 0;
              break;
        }

        //to set the speed of the motors      
        _left_wheel_motor->setPosition(_infinity);
        _right_wheel_motor->setPosition(_infinity);


        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);
    }
}

//////////////////////////////////////////////
// converting compass measurements to degrees
double MyRobot::convert_bearing_to_degrees(const double* in_vector) {
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);
    
    //if angle is negative, make positive
    if(deg < 0){
      deg = deg + 360;
    }
    
    return deg;
}

//////////////////////////////////////////////
// computing robots odometry
void MyRobot::compute_odometry() {
    float nuevaOdometriaRuedaDer = WHEEL_RADIUS * _right_wheel_sensor->getValue();
    float incrementoOdometriaRuedaDerecha = nuevaOdometriaRuedaDer - _odometriaAcumuladaRuedaDerecha;

    float nuevaOdometriaRuedaIzq = WHEEL_RADIUS * _left_wheel_sensor->getValue();
    float incrementoOdometriaRuedaIzquierda = nuevaOdometriaRuedaIzq - _odometriaAcumuladaRuedaIzquierda;


    _x = _x + ((incrementoOdometriaRuedaDerecha + incrementoOdometriaRuedaIzquierda) / 2 * cos(_theta + ((incrementoOdometriaRuedaDerecha - incrementoOdometriaRuedaIzquierda) / (2 * WHEELS_DISTANCE))));

    _y = _y + ((incrementoOdometriaRuedaDerecha + incrementoOdometriaRuedaIzquierda) / 2 * sin(_theta + ((incrementoOdometriaRuedaDerecha - incrementoOdometriaRuedaIzquierda) / (2 * WHEELS_DISTANCE))));

    _theta = _theta + ((incrementoOdometriaRuedaDerecha - incrementoOdometriaRuedaIzquierda) / WHEELS_DISTANCE);

    _odometriaAcumuladaRuedaDerecha = nuevaOdometriaRuedaDer;
    _odometriaAcumuladaRuedaIzquierda = nuevaOdometriaRuedaIzq;

}

//////////////////////////////////////////////
// Object color checks

// Checking color of encountered object (very close to robot)
bool MyRobot::GreenClose() {
    cout << BLUE << "\nChecking object color" RESET_COLOR << endl;

    const unsigned char* image_f = _forward_camera->getImage();
    unsigned char green = 0, red = 0, blue = 0;
    int green_l = 0, green_c = 0, green_r = 0;
    double p_green_l = 0.0, p_green_c = 0.0, p_green_r = 0.0;
    int x = 0, y = 0;

    //determine number of green pixels on left, center, and right
    for (y = 0; y < image_height_f; y++) {
        //left
        for (x = 0; x < image_width_f / 3; x++) {
            green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
            red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
            blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

            if (green >= 50) {
                if ((green - blue) >= green / 2 && (green - red) >= green / 2) {
                    green_l++;
                }
            }
        }

        //center
        for (x = image_width_f / 3; x < image_width_f * 2 / 3; x++) {
            green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
            red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
            blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

            if (green >= 50) {
                if ((green - blue) >= green / 2 && (green - red) >= green / 2) {
                    green_c++;
                }
            }
        }

        //right
        for (x = image_width_f * 2 / 3; x < image_width_f; x++) {
            green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
            red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
            blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

            if (green >= 50) {
                if ((green - blue) >= green / 2 && (green - red) >= green / 2) {
                    green_r++;
                }
            }
        }
    }

    p_green_l = (green_l / (float)((image_width_f / 3) * image_height_f)) * 100;
    p_green_c = (green_c / (float)((image_width_f / 3) * image_height_f)) * 100;
    p_green_r = (green_r / (float)((image_width_f / 3) * image_height_f)) * 100;

    cout << "Percentage of green in LEFT: " << p_green_l << endl;
    cout << "Percentage of green in CENTER: " << p_green_c << endl;
    cout << "Percentage of green in RIGHT: " << p_green_r << endl;

    if (p_green_l > GREEN_SIDE_THRESHOLD || p_green_c > GREEN_FRONT_THRESHOLD || p_green_r > GREEN_SIDE_THRESHOLD) {
        cout << BLUE << "Green object very close" << RESET_COLOR << endl;
        return 1;
    }
    else {
        return 0;
    }
}

// determining color of object in front of robot, but far away
bool MyRobot::GreenInFront() {
      cout << GREEN << "\nChecking object color" RESET_COLOR << endl;
  
      const unsigned char* image_f = _forward_camera->getImage();
      unsigned char green = 0, red = 0, blue = 0;
      int blackPixels_l = 0, blackPixels_c = 0, blackPixels_r = 0;
      int green_l = 0, green_c = 0, green_r = 0;
      double percentBlackLeft = 0.0, percentBlackCenter = 0.0, percentBlackRight = 0.0;
      double p_green_l = 0.0, p_green_c = 0.0, p_green_r = 0.0;
      int x = 0, y = 0;

      //determine number of black pixels on left, center, and right
      for(y = 0; y < image_height_f; y++) {
          //left
          for (x = 0; x < image_width_f/3; x++) {
              green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
              red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
              blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

              if ((green < COLOR_THRESHOLD) && (red < COLOR_THRESHOLD) && (blue < COLOR_THRESHOLD)) {
                  blackPixels_l++;
              }

              if(green >= 50){
                  if((green - blue) >= green/2 && (green - red) >= green/2){
                    green_l++;
                  }
              }
          }

          //center
          for (x = image_width_f/3; x < image_width_f*2/3; x++) {
              green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
              red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
              blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

              if ((green < COLOR_THRESHOLD) && (red < COLOR_THRESHOLD) && (blue < COLOR_THRESHOLD)) {
                  blackPixels_c++;
              }

              if(green >= 50){
                  if((green - blue) >= green/2 && (green - red) >= green/2){
                    green_c++;
                  }
              }
          }

          //right
          for (x = image_width_f*2/3; x < image_width_f; x++) {
              green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
              red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
              blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

              if ((green < COLOR_THRESHOLD) && (red < COLOR_THRESHOLD) && (blue < COLOR_THRESHOLD)) {
                  blackPixels_r++;
              }

              if(green >= 50){
                  if((green - blue) >= green/2 && (green - red) >= green/2){
                    green_r++;
                  }
              }
          }
      }

      percentBlackLeft = (blackPixels_l / (float) ((image_width_f/3) * image_height_f)) * 100;
      percentBlackCenter = (blackPixels_c / (float) ((image_width_f/3) * image_height_f)) * 100;
      percentBlackRight = (blackPixels_r / (float) ((image_width_f/3) * image_height_f)) * 100;

      p_green_l = (green_l / (float) ((image_width_f/3) * image_height_f)) * 100;
      p_green_c = (green_c / (float) ((image_width_f/3) * image_height_f)) * 100;
      p_green_r = (green_r / (float) ((image_width_f/3) * image_height_f)) * 100;

      cout << "Percentage of black in LEFT: " << percentBlackLeft << endl;
      cout << "Percentage of black in CENTER: " << percentBlackCenter << endl;
      cout << "Percentage of black in RIGHT: " << percentBlackRight << endl;

      cout << "Percentage of green in LEFT: " << p_green_l << endl;
      cout << "Percentage of green in CENTER: " << p_green_r << endl;
      cout << "Percentage of green in RIGHT: " << p_green_c << endl;

    if ((p_green_c > GREEN_CENTER_THRESHOLD &&
        p_green_c > p_green_l && p_green_c > p_green_r)
        ||
        (p_green_c > 1 && (percentBlackLeft > 25 || percentBlackCenter > 25 || percentBlackRight > 25))){
        cout << GREEN << "Green object in front" << RESET_COLOR << endl;
        return 1;
    }
    else {
        return 0;
    }

}

//////////////////////////////////////////////
// identifying yellow line at end
bool MyRobot::YellowLine(){
    cout << RED << "\nChecking for yellow line" RESET_COLOR << endl;

    const unsigned char* image_f = _forward_camera->getImage();
    unsigned char green = 0, red = 0, blue = 0;
    int yellow_l = 0, yellow_c = 0, yellow_r = 0;
    double p_yellow_l = 0.0, p_yellow_c = 0.0, p_yellow_r = 0.0;
    int x = 0, y = 0;

    //determine number of green pixels on left, center, and right
    for (y = 0; y < image_height_f; y++) {
        //left
        for (x = 0; x < image_width_f / 3; x++) {
            green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
            red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
            blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

            if(red >= 150 && green >= 150 && (red - blue >= 50) && (green - blue >= 50)){
                yellow_l++;
            }   
        }

        //center
        for (x = image_width_f / 3; x < image_width_f * 2 / 3; x++) {
            green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
            red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
            blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

            if(red >= 150 && green >= 150 && (red - blue >= 50) && (green - blue >= 50)){
                yellow_c++;
            }  
        }

        //right
        for (x = image_width_f * 2 / 3; x < image_width_f; x++) {
            green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
            red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
            blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

            if(red >= 150 && green >= 150 && (red - blue >= 50) && (green - blue >= 50)){
                yellow_r++;
            }  
        }
    }

    p_yellow_l = (yellow_l / (float)((image_width_f / 3) * image_height_f)) * 100;
    p_yellow_c = (yellow_c / (float)((image_width_f / 3) * image_height_f)) * 100;
    p_yellow_r = (yellow_r / (float)((image_width_f / 3) * image_height_f)) * 100;

    cout << "Percentage of yellow in LEFT: " << p_yellow_l << endl;
    cout << "Percentage of yellow in CENTER: " << p_yellow_c << endl;
    cout << "Percentage of yellow in RIGHT: " << p_yellow_r << endl;

    if (p_yellow_l > 1 && p_yellow_c > 4 && p_yellow_r > 1) {
        cout << GREEN << "Yellow line in front" << RESET_COLOR << endl;
        return 1;
    }
    else {
        return 0;
    }
}


//////////////////////////////////////////////
// Distance Sensor Navigation
void MyRobot::distSensorStrategy() {
    
    cout << "In dist sensor strategy" << endl;
    cout << "Mode: " << _mode << endl;
    
    srand((unsigned) _x*_y*_theta);
    
    //the logic about wallfollower
    if (_mode == FORWARD) {
        //if there's something in front of robot
        if (ir_front > DISTANCE_SENSOR_THRESHOLD) { 
            //if robot is closer to obstacle on left, turn right
            if(ir_left > ir_right + 75){
              cout << "\tTurning right from dist sensors" << endl;
              _mode = TURNING_RIGHT;
            }
            //if robot closer to object on right, turn left
            else if(ir_left < ir_right - 75){
              _mode = TURNING_LEFT;
              cout << "\tTurning left from dist sensors"<<endl;
            }
            //if no obstacle's on either side, turn randomly
            else{
              if(rand()%2){
                cout << "\tTurning right randomly from dist sensors" << endl;
                _mode = TURNING_RIGHT;
              }
              else{
                _mode = TURNING_LEFT;
                cout << "\tTurning left randomly from dist sensors"<<endl;
              }
            }
        }
    }
    else if (_mode == TURNING_LEFT) {
        //if there's nothing in front
        if (ir_front < DISTANCE_SENSOR_THRESHOLD) {
            _mode = FORWARD;
            cout << "\tForward from dist sensors"<<endl;
        }
    }
    else if (_mode == TURNING_RIGHT) {
        //if nothing in front
        if (ir_front < DISTANCE_SENSOR_THRESHOLD) {
            _mode = FORWARD;
            cout << "\tForward from dist sensors"<<endl;
        }
    }
    if(_mode == STOP){
      _mode = FORWARD;
    }
}

/////////////////////////////////////////////
// Compass Navigation
void MyRobot::compassStrategy() {
    if (compass_angle < (DESIRED_ANGLE - 2)) {
          // turn right
          _mode = TURNING_RIGHT;
          cout << "Turning right by compass" << endl;
      }
      else {
          if (compass_angle > (DESIRED_ANGLE + 2)) {
              // turn left
              _mode = TURNING_LEFT;
              cout << "Turning left by compass" << endl;
          }
          else {
              // move straight forward
              _mode = FORWARD;
              cout << "Move forward by compass" << endl;
          }
      }
}

