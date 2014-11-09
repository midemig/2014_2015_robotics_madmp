/**
 * @file    MyRobot.cpp
 * @brief   A controller to avoid obstacles
 *
 * @author  Miguel Angel de Miguel Paraiso
 * @date    09-11-2014
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    _counter = 0;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds1");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds2");
    _distance_sensor[2]->enable(_time_step);
    _distance_sensor[3] = getDistanceSensor("ds3");
    _distance_sensor[3]->enable(_time_step);
    _distance_sensor[4] = getDistanceSensor("ds4");
    _distance_sensor[4]->enable(_time_step);
    _distance_sensor[5] = getDistanceSensor("ds5");
    _distance_sensor[5]->enable(_time_step);
    _distance_sensor[6] = getDistanceSensor("ds6");
    _distance_sensor[6]->enable(_time_step);
    _distance_sensor[7] = getDistanceSensor("ds7");
    _distance_sensor[7]->enable(_time_step);
    _distance_sensor[8] = getDistanceSensor("ds8");
    _distance_sensor[8]->enable(_time_step);
    _distance_sensor[9] = getDistanceSensor("ds9");
    _distance_sensor[9]->enable(_time_step);
    _distance_sensor[10] = getDistanceSensor("ds10");
    _distance_sensor[10]->enable(_time_step);
    _distance_sensor[11] = getDistanceSensor("ds11");
    _distance_sensor[11]->enable(_time_step);
    _distance_sensor[12] = getDistanceSensor("ds12");
    _distance_sensor[12]->enable(_time_step);
    _distance_sensor[13] = getDistanceSensor("ds13");
    _distance_sensor[13]->enable(_time_step);
    _distance_sensor[14] = getDistanceSensor("ds14");
    _distance_sensor[14]->enable(_time_step);
    _distance_sensor[15] = getDistanceSensor("ds15");
    _distance_sensor[15]->enable(_time_step);

    _mode = FORWARD;
}


//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    _my_compass->disable();

    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
            _distance_sensor[i]->disable();
        }
}

//////////////////////////////////////////////

void MyRobot::run()
{
double sum;

    while (step(_time_step) != -1)
    {

        get_distances();
        print_distance_sensor();

        sum = 0;

        //Checks if there is any distance sensor detecting a wall
        for (int i = 0; i < NUM_DISTANCE_SENSOR; i++)
        {
            sum = sum + _dist_val[i];
        }
        //If there isn't, the robot will follow the compass
        //300 is  becouse sometimes a sensor  detects something but is too far to care about it
        if (sum <= 300)
        {
            follow_compass(45);
        }
        //If there is, the robot will follow the wall
        if (sum > 300)
        {
            difficult_situation();
            send_mode_to_motors();
        }



        // Set the motor speeds
        cout << "-------->>>" <<_left_speed << " - " << _right_speed <<endl;
        setSpeed(_left_speed, _right_speed);
    }
}


//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
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

void MyRobot::follow_compass(double angle)
{
    // Read the sensors
    const double *compass_val = _my_compass->getValues();

    // Convert compass bearing vector to angle, in degrees
    _compass_angle = convert_bearing_to_degrees(compass_val);

    // Print sensor values to console
    cout << "Compass angle (degrees): " << _compass_angle << endl;

    // Simple bang-bang control
    if (_compass_angle < (angle - 2))
    {
        // Turn right
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED /1.5;
    }
    else {
        if (_compass_angle > (angle + 2)) {
            // Turn left
            _left_speed = MAX_SPEED /1.5;
            _right_speed = MAX_SPEED;
        }
        else {
            // Move straight forward
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED;
        }
    }
}

//////////////////////////////////////////////

void MyRobot::difficult_situation()
{
    //If the robot detects a wall behind him
    if((_dist_val[7] > CRITICAL_DISTANCE || _dist_val[8] > CRITICAL_DISTANCE || _dist_val[6] > CRITICAL_DISTANCE +50 || _dist_val[9] > CRITICAL_DISTANCE +50 || _dist_val[5] > CRITICAL_DISTANCE +100 || _dist_val[10] > CRITICAL_DISTANCE +100))
    {
        //The robot turns right or left depending on the sensor values
        if(_dist_val[7] > _dist_val[8])
        {
         _mode = TURN_RIGHT;
          cout << "1-TURN_RIGHT" << endl;
         }
         else
         {
         _mode = TURN_LEFT;
         cout << "1-TURN_LEFT" << endl;
         }
    }
    else
    {
       //If the robot detects a wall behind and in both sides
       if ((_dist_val[3] > FOLLOWING_DISTANCE) && (_dist_val[0] > FOLLOWING_DISTANCE ) && (_dist_val[12]  > FOLLOWING_DISTANCE))
        {
           //Turns left or right depending of which wall is nearer
           if(_dist_val[2] > _dist_val[13])
           {
            _mode = TURN_AROUND_LEFT;
            cout << "2-TURN_AROUND_LEFT" << endl;
           }
           else
           {
           _mode = TURN_AROUND_RIGHT;
           cout << "2-TURN_AROUND_RIGHT" << endl;
           }
        }
        else
        {
           //If the robot isn't in a difficult situation
            follow_wall();
        }
    }
}

//////////////////////////////////////////////

void MyRobot::follow_wall()
{
    {
    // If there is a wall in front of the robot
    if ((_dist_val[1] > FOLLOWING_DISTANCE + 50 || _dist_val[14] > FOLLOWING_DISTANCE + 50 || (_dist_val[3] > CRITICAL_DISTANCE && _dist_val[4] > FOLLOWING_DISTANCE ) || (_dist_val[12] > CRITICAL_DISTANCE && _dist_val[11] > FOLLOWING_DISTANCE )))
    {
        //The robot goes backwards and right or left depending on where is the lateral wall
        if(turning_condition())
        {
         _mode = TURN_OVER_EDGE_LEFT;
          cout << "3-WALL_FOLLOWER_LEFT" << endl;
         }
         else
         {
         _mode = TURN_OVER_EDGE_RIGHT;
         cout << "4-WALL_FOLLOWER_RIGHT" << endl;
         }

     }
     else
     {
         //If the robot is too close of right wall or too far of left wall turns left
        if (((_dist_val[13] > FOLLOWING_DISTANCE) || ((_dist_val[2] < FOLLOWING_DISTANCE + 50) && (_dist_val[2]  != 0)))/* && (_dist_val[3] < CRITICAL_DISTANCE && _dist_val[4] < CRITICAL_DISTANCE && _dist_val[12] < CRITICAL_DISTANCE && _dist_val[11] < CRITICAL_DISTANCE) && (_dist_val[7] < CRITICAL_DISTANCE && _dist_val[8] < CRITICAL_DISTANCE)*/)
         {
             _mode = TURN_LEFT;
             cout << "5-TURN_LEFT" << endl;
         }
         else
         {
             //If the robot is too far of right wall or too close of left wall turns right
             if ((((_dist_val[13] < FOLLOWING_DISTANCE + 50) &&( _dist_val[13]  != 0 )) || _dist_val[2] > FOLLOWING_DISTANCE)/* && (_dist_val[3] < CRITICAL_DISTANCE && _dist_val[4] < CRITICAL_DISTANCE && _dist_val[12] < CRITICAL_DISTANCE && _dist_val[11] < CRITICAL_DISTANCE) && (_dist_val[7] < CRITICAL_DISTANCE && _dist_val[8] < CRITICAL_DISTANCE)*/)
             {
                 _mode = TURN_RIGHT;
                 cout << "6-TURN_RIGHT" << endl;
             }
             else
             {
                 //If the robot detects a wall behind it
                 if (_dist_val[5] > FOLLOWING_DISTANCE || _dist_val[6] > FOLLOWING_DISTANCE || _dist_val[9] > FOLLOWING_DISTANCE || _dist_val[10] > FOLLOWING_DISTANCE)
                 {
                     _mode = FORWARD;
                     cout << "7-FORWARD" << endl;
                 }
                 //If there is a non contempñated case, the robot will do the same it has done before
                 else
                 {
                     cout << "8-anterior" << endl;
                 }
             }

         }
     }
    }
}

//////////////////////////////////////////////

void MyRobot::send_mode_to_motors()
{
            // Send actuators commands according to the mode
    switch (_mode){
        case FORWARD:
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED;
            break;
        case TURN_LEFT:
            _left_speed = MAX_SPEED / 1.35;
            _right_speed = MAX_SPEED;
            break;
        case TURN_RIGHT:
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED / 1.35;
            break;
        case TURN_OVER_EDGE_LEFT:
             _left_speed = -MAX_SPEED / 15.0;
             _right_speed = -MAX_SPEED / 5.0;
              break;
        case TURN_OVER_EDGE_RIGHT:
             _left_speed = -MAX_SPEED / 5.0;
             _right_speed = -MAX_SPEED / 15.0;
              break;
        case TURN_AROUND_LEFT:
             _left_speed = -MAX_SPEED / 5.0;
             _right_speed = MAX_SPEED / 5.0;
              break;
        case TURN_AROUND_RIGHT:
             _left_speed = MAX_SPEED / 5.0;
             _right_speed = -MAX_SPEED / 5.0;
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

bool MyRobot::turning_condition()
{
    if(_dist_val[3] == 0 && _dist_val[12] == 0)
    {
        if(_dist_val[2] > _dist_val[13])
            return true;
        else
            return false;
    }
    else
    {
        if(_dist_val[3] > _dist_val[12])
            return true;
        else
            return false;
    }
}
