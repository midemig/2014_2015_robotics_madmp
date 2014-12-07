/**
 * @file    MyRobot.cpp
 * @brief   A controller rescue 2 green cylinders
 *
 * @author  Miguel Angel de Miguel Paraiso <100292950@alumnos.uc3m.es>
 * @author  Ainoa Zugasti Hernandez        <100292754@alumnos.uc3m.es>
 * @date    05-12-2014
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{

    _time_step = 64;
    //counts the steps of time
    _time_counter = 0;

    _left_speed = 0;
    _right_speed = 0;

    //Distance sensors
    for(int i=0; i<16; i++)
    {
        string matriz;
        ostringstream conversion;
        conversion<<i;
        matriz= conversion.str();
        _distance_sensor[i] = getDistanceSensor("ds"+matriz);
        _distance_sensor[i]->enable(_time_step);

    }

    //Cameras
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);

    //Odometry
    enableEncoders(_time_step);
    _last_left_encoder = 0;
    _last_right_encoder = 0;
    _absolut_z = 0;
    _absolut_y = 0;
    _counter = 0;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    //Initialices some variables
    _stopped = false;
    _first_cylinder = false;
    _status = OBSTACLE;
    _second_cylinder = false;
    _turning = false;
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    _forward_camera->disable();
    _spherical_camera->disable();
    for (int x=0; x<NUM_DISTANCE_SENSOR; x++) {
        _distance_sensor[x]->disable();
    }
}

//////////////////////////////////////////////

void MyRobot::run()
{


    // Get size of images for forward camera
    _image_width_f = _forward_camera->getWidth();
    _image_height_f = _forward_camera->getHeight();
    cout << "Size of forward camera image: " << _image_width_f << ", " <<  _image_height_f << endl;

    // Get size of images for spherical camera
    _image_width_s = _spherical_camera->getWidth();
    _image_height_s = _spherical_camera->getHeight();
    cout << "Size of spherical camera image: " << _image_width_s << ", " << _image_height_s << endl;

    while (step(_time_step) != -1)
    {
        //If time counter is pair
        if(_time_counter%2 == 0)
        {
        refresh_angle();
        refresh_odometry();
        }
        //If time counter is odd
        else
        {
        //If the robot sees a cylinder and it hasn't found the two cylinders
        if(!_second_cylinder && cylinder_angle())
        {
            _status = CYLINDER;
        }
        else
        {
            //If the robot has found the second cylinder
            if(_second_cylinder)
            {
                _desired_angle = 0;
                //Changes the angle to go in the oposit direction
                if(_angle >= 0)
                {
                    _angle = _angle - 180;
                }
                else
                {
                    _angle = _angle + 180;
                }
            }
            else
            {
                _desired_angle = 0;
            }
        }

        get_distances();

        double sum = 0;

        //Checks if there is any distance sensor detecting a wall
        for (int i = 0; i < NUM_DISTANCE_SENSOR; i++)
        {
            sum = sum + _dist_val[i];
        }
        //If there isn't, the robot will follow the compass
        //300 is  becouse sometimes a sensor  detects something but is too far to care about it
        if (sum <= 300)
        {
           //Set speeds to go to the desired angle
           go_to_angle();
        }
        else
        {
            //If there is a front cylinder
            if(detect_front_cylinder())
            {
                //cout << "Front cylinder" << endl;
            }
            //If there isn't the robot avoid obstacles
            else
            {
                choose_situation();
                if(!_turning && _status == OBSTACLE)
                {
                send_mode_to_motors();
                }
            }
        }
        //Set wheels speed

        if(_second_cylinder && _absolut_z <0)
        {
            _left_speed = 0;
            _right_speed = 0;
            cout << "Home" << endl;
        }
        //cout << "0D: " << _desired_angle << "  A: " << _angle << endl;
        setSpeed(_left_speed, _right_speed);
        }
        _time_counter ++;
    }
}

//////////////////////////////////////////////

bool MyRobot::cylinder_angle()
{
    int zone[9] = {0,0,0,0,0,0,0,0,0};
    _image_s = _spherical_camera->getImage();
    unsigned char green = 0, red = 0, blue = 0;
    double percentage_green[9];
    // Count number of pixels that are green
    for (int x = 0; x < _image_width_s; x++)
    {
        for (int y = 0; y < _image_height_s; y++)
        {
            green = _forward_camera->imageGetGreen(_image_s, _image_width_s, x, y);
            red = _forward_camera->imageGetRed(_image_s, _image_width_s, x, y);
            blue = _forward_camera->imageGetBlue(_image_s, _image_width_s, x, y);

            //If the pixel is green, the zoone of the pixel increments in oone
            if ((green > 255-COLOUR_ERROR) && (red >= 41-COLOUR_ERROR && red <= 41+COLOUR_ERROR) && (blue >= 14-COLOUR_ERROR && blue <= 14+COLOUR_ERROR))
            {
                if (x < 65 && y < 20)//F
                    zone[0] ++;
                if (x > 65 && x < 95 && y < 20)
                    zone[1] ++;
                if (x > 95 && x < 140 && y < 20)
                    zone[2] ++;
                if (x > 140 && y < 80)//R
                    zone[3] ++;
                if (x > 140 && y > 80)
                    zone[4] ++;
                if (x > 80 && y > 140)//B
                    zone[5] ++;
                if (x < 80 && y > 140)
                    zone[6] ++;
                if (x < 20 && y > 80)//L
                    zone[7] ++;
                if (x < 20 && y < 80)
                    zone[8] ++;

            }
        }
    }
    //Calculates the percentage of greeen in each zone
    percentage_green[0] = zone[0] / (65*20/100);
    percentage_green[1] = zone[1] / (30*20/100);
    percentage_green[2] = zone[2] / (65*20/100);
    percentage_green[3] = zone[3] / (80*20/100);
    percentage_green[4] = zone[4] / (80*20/100);
    percentage_green[5] = zone[5] / (80*20/100);
    percentage_green[6] = zone[6] / (80*20/100);
    percentage_green[7] = zone[7] / (80*20/100);
    percentage_green[8] = zone[8] / (80*20/100);

    //If the first cylinder has been found
    if(_first_cylinder)
    {
        //gets the angle of the first cylinder and the zone of the camerawhere it is
        get_first_cylinder_angle();
        //Those zones are zero now so the robot doesn't see the first cylinder
        percentage_green[_position_first_angle[0]] = 0;
        percentage_green[_position_first_angle[1]] = 0;
        percentage_green[_position_first_angle[2]] = 0;
        //cout << "Buscando segundo. Anulando:" << _position_first_angle[0] << _position_first_angle[1] << _position_first_angle[2] << endl;
    }

    //Determinates wich zone has a higer percentage of green
    double sum = 0;
    int best_zone = 0;
    for(int i = 0; i < 9; i++)
    {
        sum = sum + percentage_green[i];
        if(percentage_green[i] > percentage_green[best_zone])
            best_zone = i;
    }

    //If there is a cylinder
    if(sum > 0)
    {
        //Sets the angle of the cylinder
        _cylinder_angle = get_cylinder_angle(best_zone);
        _desired_angle = _angle + _cylinder_angle;
        if(_desired_angle > 180)
            _desired_angle = _desired_angle - 360;
        if(_desired_angle < -180)
            _desired_angle = _desired_angle + 360;

        return true;
    }
    //If there isn't
    else
        return false;
}

//////////////////////////////////////////////

void MyRobot::refresh_angle()
{
    const double *compass_val = _my_compass->getValues();
    double rad = atan2(compass_val[0], compass_val[2]);
    _angle = (rad * (180.0 / M_PI));
    _angle = -_angle + 45;

    //Corrects the angle in the range
    if (_angle > 180)
        _angle = _angle - 360;
    if (_angle < -180)
        _angle = _angle + 360;
}

//////////////////////////////////////////////

void MyRobot::go_to_angle()
{
    //Corrects the angle in the range
    if(_desired_angle > 180)
        _desired_angle = _desired_angle - 360;

    if(_desired_angle < -180)
        _desired_angle = _desired_angle + 360;

    //The robot turns proportionally to the error of its orientation
    if (_angle > _desired_angle)
    {
        _left_speed = 60 + abs(_angle - _desired_angle)/4.5;
        _right_speed = 60 - abs(_angle - _desired_angle)/4.5;
    }
    else
    {
        _left_speed = 60 - abs(_angle - _desired_angle)/4.5;
        _right_speed = 60 + abs(_angle - _desired_angle)/4.5;
    }

    //In order to avoid speed overflows max speed
    if(_left_speed > 100)
        _left_speed = 100;
    if(_left_speed < -100)
        _left_speed = -100;
    if(_right_speed > 100)
        _right_speed = 100;
    if(_right_speed < -100)
        _right_speed = -100;

}

//////////////////////////////////////////////

double MyRobot::get_cylinder_angle(int zone)
{
    //Returns the angle of the zone the robot sees the cylinder
    switch (zone)
    {
        case 0:
            return 20;
        case 1:
            return 0;
        case 2:
            return -20;
        case 3:
            return -60;
        case 4:
            return -120;
        case 5:
            return -160;
        case 6:
            return 160;
        case 7:
            return 120;
        case 8:
            return 60;
        default:
            return 0;
    }
}

//////////////////////////////////////////////

int MyRobot::get_cylinder_zone(double angle)
{
    //returns the zone of the first cylinder havong the angle
    if(angle > 180)
        angle = angle - 360;

    if(angle < -180)
        angle = angle + 360;

    if(angle > -45 && angle <= -10 )
            return 2;

    if(angle > -10 && angle <= 10 )
            return 1;

    if(angle > 10 && angle <= 45 )
            return 0;

    if(angle > 45 && angle <= 90 )
            return 8;

    if(angle > 90 && angle <= 135 )
            return 7;

    if(angle > 135 && angle <= 180 )
            return 6;

    if(angle > -180 && angle <= -135 )
            return 5;

    if(angle > -135 && angle <= -90 )
            return 4;

    if(angle > -90 && angle <= -45 )
            return 3;
    else
        return -1;

}
//////////////////////////////////////////////

bool MyRobot::detect_front_cylinder()
{
    _image_f = _forward_camera->getImage();
    int sum = 0;
    unsigned char green = 0, red = 0, blue = 0;
    double percentage_of_green;

    // Count number of pixels that are green
    for (int x = 0; x < _image_width_f; x++)
    {
        for (int y = 0; y < _image_height_f; y++)
        {
            green = _forward_camera->imageGetGreen(_image_f, _image_width_f, x, y);
            red = _forward_camera->imageGetRed(_image_f, _image_width_f, x, y);
            blue = _forward_camera->imageGetBlue(_image_f, _image_width_f, x, y);

            if ((green > 255-COLOUR_ERROR) && (red >= 41-COLOUR_ERROR && red <= 41+COLOUR_ERROR) && (blue >= 14-COLOUR_ERROR && blue <= 14+COLOUR_ERROR))
            {
                sum++;
            }
        }
    }
    percentage_of_green = (100*sum) / (_image_width_f*_image_height_f);

    //If there is enough green in the camera, the robot will stop
    if ((percentage_of_green > 25 || _stopped) && !_second_cylinder)
    {
        cylinder_found();
        return true;
    }
    else
        return false;
}

//////////////////////////////////////////////

void MyRobot::cylinder_found()
{
    //Stops 2 seconds
    if (_counter < 35)
    {
    _left_speed = 0;
    _right_speed = 0;
        if(_counter == 34)
        {
            _stopped = true;
            //If it is the first cylinder, saves its position in order to avoid it later
            if(_first_cylinder == false)
            {
            _first_cylinder_z = _absolut_z + 0.5*cos(_angle);
            _first_cylinder_y = _absolut_y + 0.5*sin(_angle);
            cout << "First Cylinder in position: Z: " << _first_cylinder_z << "   Y: " << _first_cylinder_y << endl;
            }
        }
    }
    //Turns 360 degrees
    if (_counter > 34 && _counter < 55)
    {
        _left_speed = 75;
        _right_speed = -75;
    }
    _counter ++;
    //Resets everything
    if(_counter == 54)
    {
        _stopped = false;
        _counter = 0;
        if(_first_cylinder == true)
        {
            _second_cylinder = true;
            cout << "*****+++++Second found+++++*****" << endl;
        }
        if(_first_cylinder == false)
        {
            _first_cylinder = true;
            cout << "*****+++++First found+++++*****" << endl;
        }
    }
}

//////////////////////////////////////////////

void MyRobot::get_first_cylinder_angle()
{
    //Gets and corrects the first cylinder angle
    double angle = (((atan2((_first_cylinder_y - _absolut_y),(_first_cylinder_z - _absolut_z)))*57.29) - _angle);
    if(angle > 180)
        angle = angle - 360;
    if(angle < -180)
        angle = angle + 360;

    //Determinates the zones of the camera where it is
    //cout << "Avoiding angle: " << angle << "  z" << _first_cylinder_z << "  y" << _first_cylinder_y << endl;
    _position_first_angle[0] = get_cylinder_zone(angle);
    _position_first_angle[1] = get_cylinder_zone(angle + ANGLE_OF_IGNORE);
    _position_first_angle[2] = get_cylinder_zone(angle - ANGLE_OF_IGNORE);
}

//////////////////////////////////////////////

void MyRobot::print_odometry_data()
{
    cout << "Z: " << _absolut_z << " - X: " << _absolut_y << " - Angle: " <<_angle << endl;
}

//////////////////////////////////////////////

void MyRobot::refresh_odometry()
{
    //Gets the radians of each encoder
    _left_encoder = getLeftEncoder()/ENCODER_RESOLUTION;
    _right_encoder = getRightEncoder()/ENCODER_RESOLUTION;

    //Calculates the increment of each encoder
    double left_increment = _left_encoder - _last_left_encoder;
    double right_increment = _right_encoder - _last_right_encoder;

    //Calculates the absolut position and orientation of the robot
    _absolut_z = _absolut_z + ((right_increment + left_increment)/2)*cos(_angle/57.29 + ((right_increment - left_increment)/2*DISTANCE_BETWEEN_WHEELS))*WHEEL_RADIO;
    _absolut_y = _absolut_y + ((right_increment + left_increment)/2)*sin(_angle/57.29 + ((right_increment - left_increment)/2*DISTANCE_BETWEEN_WHEELS))*WHEEL_RADIO;
    _odometry_angle = _odometry_angle + ((right_increment - left_increment)/DISTANCE_BETWEEN_WHEELS)*WHEEL_RADIO;

    //Corrects the angle to a range of [-180,180] degrees
    if(_odometry_angle > 180/57.29)
        _odometry_angle = _odometry_angle - 360/57.29;

    if(_odometry_angle < -180/57.29)
        _odometry_angle = _odometry_angle + 360/57.29;

    //Refresh the data of last encoder value
    _last_left_encoder = _left_encoder;
    _last_right_encoder = _right_encoder;
}

//////////////////////////////////////////////

void MyRobot::print_distance_sensor()
{

    cout << "Distance sensor (m)" << endl;
    cout << "ds0: " << _dist_val[0] << "|ds1: " << _dist_val[1]
    << "|ds2: " << _dist_val[2] << "|ds3: " << _dist_val[3] << endl
    << "|ds4: " << _dist_val[4] << "|ds5: " << _dist_val[5]
    << "|ds6: " << _dist_val[6] << "|ds7: " << _dist_val[7] << endl
    << "|ds8: " << _dist_val[8] << "|ds9: " << _dist_val[9]
    << "|ds10: " << _dist_val[10] << "|ds11: " << _dist_val[11] << endl
    << "|ds12: " << _dist_val[12] << "|ds13: " << _dist_val[13]
    << "|ds14: " << _dist_val[14] << "|ds15: " << _dist_val[15] << endl;
}

//////////////////////////////////////////////

void MyRobot::choose_situation()
{

    //IF the robot is rounded by walls it will turn 180 degrees
    if((_dist_val[3] > FOLLOWING_DISTANCE && _dist_val[12] > FOLLOWING_DISTANCE && (_dist_val[1] > FOLLOWING_DISTANCE || _dist_val[14] > FOLLOWING_DISTANCE) ) || _turning)
    {
        turn_around(12);
    }
    else
    {
    //If the robot detects a wall in front of him
    if(_dist_val[1] > FOLLOWING_DISTANCE || _dist_val[14] > FOLLOWING_DISTANCE)
    {
        //The robot follows right or left wall depending on the sensor values
        if(turning_condition() == 1)
        {
         follow_left_wall();
         }
         else
         {
         follow_right_wall();
         }
    }
    else
    {
       //If the robot detects a wall in front too close
       if (_dist_val[0] > 600 || _dist_val[15] > 600)
        {
           //Turns left or right depending of which wall is nearer
           if(turning_condition() == 1)
           {
           follow_left_wall();
           }
           else
           {
           follow_right_wall();
           }
        }
        else
       {
           //If there is a wall in the left too close
           if(_dist_val[3] > 700 || _dist_val[2] > 700)
           {
           _mode = TURN_RIGHT;
           }
           else
           {
               //If there is a wall in the right too close
               if (_dist_val[13] > 700 || _dist_val[12] > 700)
               {
               _mode = TURN_LEFT;
               }
               else
               {
                   //If there is a wall in the left
                   if(_dist_val[3] > FOLLOWING_DISTANCE)
                   {
                       follow_left_wall();
                   }
                   else
                   {
                       //If there is a wall in the right
                       if(_dist_val[12] > FOLLOWING_DISTANCE)
                       {
                       follow_right_wall();
                       }
                       //Else follows the desired angle
                       else
                       {
                           go_to_angle();
                           _status = POINT;
                       }
                   }

               }
           }
       }
    }
    }
}

//////////////////////////////////////////////

void MyRobot::follow_left_wall()
{
    //This is to go to the cylinder instead of following the wall
    if(_status == CYLINDER && ((_desired_angle - _angle < -30)))
    {
        go_to_angle();
    }
    else
    {
    //This is to go to the desired angle instead of following the wall (if poddible)
    if((_angle - _desired_angle) < (90 + ANGLE_OF_AVOID_WALL) && (_angle - _desired_angle) > (90 - ANGLE_OF_AVOID_WALL) && _dist_val[0] < FOLLOWING_DISTANCE)
    {
        go_to_angle();
        _status = POINT;
        return;
    }
    else
    {
    _status = OBSTACLE;
    // If there is a wall in front of the robot
    if ((_dist_val[1] > FOLLOWING_DISTANCE) || (_dist_val[14] > FOLLOWING_DISTANCE))
    {
        //If there is a back wall too close
        if((_dist_val[7] > 800) || (_dist_val[8] > 800))
        {
         _mode = TURN_RIGHT;
        }
        //If its only front wall
        else
        {
        _mode = TURN_OVER_EDGE_LEFT;
        }

     }
     else
     {
         //If the robot is too close of left wall
        if (_dist_val[2] > FOLLOWING_DISTANCE || _dist_val[3] > 600)
         {
             _mode = TURN_RIGHT;
         }
         else
         {
             //If the robot doesn't detect a left wall
             if ((_dist_val[3] == 0) && (_dist_val[4] == 0) && (_dist_val[2] == 0))
             {
                 go_to_angle();
                 _status = POINT;
             }
             else
             {
                 //If the left wall is too far
                 if ((_dist_val[2] < FOLLOWING_DISTANCE + 50))
                 {
                     _mode = TURN_LEFT;
                 }
                 //Else it goes to the desired angle
                 else
                 {
                     go_to_angle();
                     _status = POINT;
                 }

             }
            }
         }
     }
    }
}

//////////////////////////////////////////////

void MyRobot::follow_right_wall()
{
    //This is to go to the cylinder instead of following the wall
    if(_status == CYLINDER && ((_desired_angle - _angle > 30)))
    {
        go_to_angle();
    }
    else
    {
    //This is to go to the desired angle instead of following the wall (if poddible)
    if((_angle - _desired_angle) < (-90 + ANGLE_OF_AVOID_WALL) && (_angle - _desired_angle) > (-90 - ANGLE_OF_AVOID_WALL) && _dist_val[15] < FOLLOWING_DISTANCE)
    {
        go_to_angle();
        _status = POINT;
        return;
    }
    else
    {
    _status = OBSTACLE;
    // If there is a wall in front of the robot
    if ((_dist_val[1] > FOLLOWING_DISTANCE) || (_dist_val[14] > FOLLOWING_DISTANCE))
    {
        //If there is a back wall too close
        if((_dist_val[7] > 800) || (_dist_val[8] > 800))
        {
         _mode = TURN_LEFT;
         }
        //If its only front wall
         else
         {
         _mode = TURN_OVER_EDGE_RIGHT;
         }

     }
     else
     {
        //If the robot is too close of right wall
        if (_dist_val[13] > FOLLOWING_DISTANCE || _dist_val[12] > 600)
         {
             _mode = TURN_LEFT;
         }
         else
         {
             //If the robot doesn't detect a right wall
             if ((_dist_val[13] == 0) && (_dist_val[11] == 0) && (_dist_val[12] == 0))
             {
                 go_to_angle();
                 _status = POINT;
             }
             else
             {
                 //If the right wall is too far
                 if ((_dist_val[13] < FOLLOWING_DISTANCE + 50))
                 {
                     _mode = TURN_RIGHT;
                 }
                 //Else it goes to the desired angle
                 else
                 {
                     go_to_angle();
                     _status = POINT;
                 }

             }

         }
        }
     }
    }
}

//////////////////////////////////////////////

void MyRobot::turn_around(int time)
{
    _turning = true;
    //First stops
    if(_counter < 7)
    {
        _left_speed = 0;
        _right_speed = 0;
        _counter++;
    }
    //Then turns as much as time variable says
    else
    {
        if(_counter < time)
        {
            _left_speed = 60;
            _right_speed = -60;
            _counter++;
        }
        if (_counter == time)
        {
            _turning = false;
            _counter = 0;
        }
    }

}

//////////////////////////////////////////////

void MyRobot::send_mode_to_motors()
{
            // Send actuators commands according to the mode
    switch (_mode){
        case FORWARD:
            _left_speed = MAX_SPEED/2;
            _right_speed = MAX_SPEED/2;
            break;
        case TURN_LEFT:
            _left_speed = MAX_SPEED / 2.6;
            _right_speed = MAX_SPEED/1.3;
            break;
        case TURN_RIGHT:
            _left_speed = MAX_SPEED/1.3;
            _right_speed = MAX_SPEED / 2.6;
            break;
        case TURN_OVER_EDGE_LEFT:
             _left_speed = 0;
             _right_speed = -MAX_SPEED / 2;
              break;
        case TURN_OVER_EDGE_RIGHT:
             _left_speed = -MAX_SPEED / 2;
             _right_speed = 0;
              break;
        case TURN_AROUND_LEFT:
             _left_speed = -MAX_SPEED /5;
             _right_speed = MAX_SPEED / 5;
              break;
        case TURN_AROUND_RIGHT:
             _left_speed = MAX_SPEED / 5;
             _right_speed = -MAX_SPEED / 5;
              break;
        default:
            break;
    }
}

//////////////////////////////////////////////

void MyRobot::get_distances()
{
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
     {
        _dist_val[i] = _distance_sensor[i]->getValue();
     }

}

//////////////////////////////////////////////

int MyRobot::turning_condition()
{
    //Sums the distance sensor of each side to decide where is a closer wall, inthe right or in the left
    double sum_left = _dist_val[0] + _dist_val[1] + _dist_val[2] + _dist_val[3];
    double sum_right = _dist_val[15] + _dist_val[14] + _dist_val[13] + _dist_val[12];
    if(sum_left > sum_right)
        return 1;
    else
        return 2;

}
