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
/// \brief MyRobot::run
///

void MyRobot::run()
{
    double compass_angle;



    while (step(_time_step) != -1)
    {

        // Read the sensors
        const double *compass_val = _my_compass->getValues();
        for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
        {

            _dist_val[i] = _distance_sensor[i]->getValue();
        }


        // Convert distance of the sensors to meters
        convert_distances_to_meters();
        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Print sensor values to console
        print_distance_sensor();

        if ((_dist_val[3] >= 0.1 || _dist_val[3] == -1))
        {
            // Simple bang-bang control
            if (compass_angle < (DESIRED_ANGLE - 2)) {
                // Turn right
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED - 15;
            }
            else {
                if (compass_angle > (DESIRED_ANGLE + 2)) {
                    // Turn left
                    _left_speed = MAX_SPEED - 15;
                    _right_speed = MAX_SPEED;
                }
                else {
                    // Move straight forward
                    _left_speed = MAX_SPEED;
                    _right_speed = MAX_SPEED;
                }
            }




        }
        else
        {
         _left_speed = 0;
         _right_speed = 0;
        }
         // Set the motor speeds
         setSpeed(_left_speed, _right_speed);
    }
}


//////////////////////////////////////////////
/// \brief MyRobot::convert_bearing_to_degrees
/// \param in_vector
/// \return
///

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}


//////////////////////////////////////////////
/// \brief MyRobot::print_distance_sensor
///


void MyRobot::print_distance_sensor()
{

    cout << "Distance sensor (m)" << endl;
    cout << "ds3: " << _dist_val[0] << "|ds2: " << _dist_val[1]
    << "|ds1: " << _dist_val[2] << "|ds0: " << _dist_val[3] << endl;
    cout << "ds15: " << _dist_val[4] << "|ds14: " << _dist_val[5]
    << "|ds13: " << _dist_val[6] << "|ds12: " << _dist_val[7] << endl;
}


