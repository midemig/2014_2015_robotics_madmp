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

    _counter = 0;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _distance_sensor[0] = getDistanceSensor("ds3");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds2");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds1");
    _distance_sensor[2]->enable(_time_step);
    _distance_sensor[3] = getDistanceSensor("ds0");
    _distance_sensor[3]->enable(_time_step);
    _distance_sensor[4] = getDistanceSensor("ds15");
    _distance_sensor[4]->enable(_time_step);
    _distance_sensor[5] = getDistanceSensor("ds14");
    _distance_sensor[5]->enable(_time_step);
    _distance_sensor[6] = getDistanceSensor("ds13");
    _distance_sensor[6]->enable(_time_step);
    _distance_sensor[7] = getDistanceSensor("ds12");
    _distance_sensor[7]->enable(_time_step);

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
    int n_max = 200;
    int j;

    while (step(_time_step) != -1)
    {

        for (j = 0; j < n_max; j++)
        {
            for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
            {
                if (j == 0)
                _dist_val[i] = _distance_sensor[i]->getValue();

                if (j != n_max && j!= 0)
                    _dist_val[i] = _dist_val[i] + _distance_sensor[i]->getValue();

                if (j == (n_max - 1))
                    _dist_val[i] = _dist_val[i]/(n_max - 1);
            }
        }



        print_distance_sensor();

        if (_mode == FORWARD)
        {
                    // Move forward

                    // When sufficiently close to a wall in front of robot,
                    // switch mode to wall following
                if ((_dist_val[3] > FOLLOWING_DISTANCE) || (_dist_val[6] > FOLLOWING_DISTANCE)) {
                        _mode = WALL_FOLLOWER;
                        cout << "Mode " << WALL_FOLLOWER << ": Wall following mode activated" << endl;
                    }
        }
        else
        {
                // Wall following

                if ((_dist_val[3] > FOLLOWING_DISTANCE) || (_dist_val[4 ] > FOLLOWING_DISTANCE)) {
                    _mode = WALL_FOLLOWER;
                    cout << "Backing up and turning left." << endl;
                }
                else {
                     if (_dist_val[6] > FOLLOWING_DISTANCE) {
                            _mode = TURN_LEFT;
                            cout << "Turning left." << endl;
                        }
                        else {
                            if (_dist_val[6] < FOLLOWING_DISTANCE + 50) {
                                _mode = TURN_RIGHT;
                                cout << "Turning right." << endl;
                            }
                            else {
                                _mode = FORWARD;
                                cout << "Moving forward." << endl;
                            }
                        }
                    }
         }

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
                        _left_speed = MAX_SPEED / 1.25;
                        _right_speed = MAX_SPEED;
                        break;
                    case TURN_RIGHT:
                        _left_speed = MAX_SPEED;
                        _right_speed = MAX_SPEED / 1.25;
                        break;
                    case WALL_FOLLOWER:
                        _left_speed = -MAX_SPEED / 3;
                        _right_speed = -MAX_SPEED / 20.0;
                        break;
                    default:
                        break;
                }

        // Set the motor speeds
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
double MyRobot::convert_to_distance(double number)
{
    if (number >= 1000)
        return 0.1;

    if (number < 1000 && number >= 400)
        return (0.1 + (1000 - number)*0.1/600);

    if (number < 400 && number >= 50)
        return (0.2 + (400 - number)*0.1/350);

    if (number < 50 && number >= 30)
        return (0.3 + (50 - number)*0.07/20);

    if (number < 30)
        return -1;
    return -1;
}




void MyRobot::print_distance_sensor()
{

    cout << "Distance sensor (m)" << endl;
    cout << "ds3: " << _dist_val[0] << "|ds2: " << _dist_val[1]
    << "|ds1: " << _dist_val[2] << "|ds0: " << _dist_val[3] << endl;
    cout << "ds15: " << _dist_val[4] << "|ds14: " << _dist_val[5]
    << "|ds13: " << _dist_val[6] << "|ds12: " << _dist_val[7] << endl;
}

void MyRobot::convert_distances_to_meters()
{
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
    {
        _dist_val[i] = convert_to_distance(_dist_val[i]);
    }
}
