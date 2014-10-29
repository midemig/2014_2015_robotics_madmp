/**
 * @file    MyRobot.cpp
 * @brief   A simple example for maintaining a straight line with the compass.
 *
 * @author  Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date    2014-07
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;



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
    while (step(_time_step) != -1)
    {
        get_distances();
        print_distance_sensor();
        follow_wall();
        apply_mode();
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
/**
 *
 */
void MyRobot::follow_wall()
{

    if (_mode == FORWARD)
    {
                // Move forward

                // When sufficiently close to a wall in front of robot,
                // switch mode to wall following
            if ((_dist_val[1] > FOLLOWING_DISTANCE) || (_dist_val[14] > FOLLOWING_DISTANCE))
            {
                if(_dist_val[0] > _dist_val[15])
                    _mode = WALL_FOLLOWER_LEFT;

                else
                    _mode = WALL_FOLLOWER_RIGHT;

                    cout << "Mode : Wall following mode activated" << endl;
            }
    }
    else
    {
            if ((_dist_val[1] > FOLLOWING_DISTANCE) || (_dist_val[14] > FOLLOWING_DISTANCE))
            {
                if(_dist_val[1] > _dist_val[14])
                {
                 _mode = WALL_FOLLOWER_LEFT;
                 cout << "WALL_FOLLOWER_LEFT" << endl;
                }
                else
                {
                 _mode = WALL_FOLLOWER_RIGHT;
                 cout << "WALL_FOLLOWER_RIGHT" << endl;
                 }

            }
            else
            {
                 if ((_dist_val[13] > FOLLOWING_DISTANCE || (_dist_val[2] < FOLLOWING_DISTANCE + 50) && _dist_val[2]  != 0 )) {
                        _mode = TURN_LEFT;
                        cout << "TURN_LEFT" << endl;
                    }
                 else
                 {
                     if (((_dist_val[13] < FOLLOWING_DISTANCE + 50) &&( _dist_val[2]  != 0 )) || _dist_val[2] > FOLLOWING_DISTANCE )
                     {
                         _mode = TURN_RIGHT;
                         cout << "TURN_RIGHT" << endl;
                     }
                     else
                     {
                         _mode = FORWARD;
                         cout << "FORWARD" << endl;
                     }
                  }
             }

     }

}


void MyRobot::apply_mode()
{
            // Send actuators commands according to the mode
    switch (_mode){
        case STOP:
            _left_speed = 0;
            _right_speed = 0;
            break;
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
        case WALL_FOLLOWER_RIGHT:
            _left_speed = -MAX_SPEED / 5;
            _right_speed = -MAX_SPEED / 20.0;
            break;
        case WALL_FOLLOWER_LEFT:
             _left_speed = -MAX_SPEED / 20.0;
             _right_speed = -MAX_SPEED / 5.0;
              break;
        case TURN_OVER_EDGE:
             _left_speed = MAX_SPEED / 5.0;
             _right_speed = -MAX_SPEED / 5.0;
              break;
        default:
            break;
    }
}

void MyRobot::get_distances()
{
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
     {
        _dist_val[i] = _distance_sensor[i]->getValue();
     }

}
