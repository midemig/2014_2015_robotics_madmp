/**
 * @file    MyRobot.h
 * @brief   The robot goes to the other side of the map with odometry
 *
 * @author  Miguel Angel de Miguel Paraiso <100292950@alumnos.uc3m.es>
 * @date    09-11-2014
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    enableEncoders(_time_step);

    _left_speed = 100;
    _right_speed = 100;
    _distance = 0;
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    disableEncoders();
}

//////////////////////////////////////////////

void MyRobot::run()
{

    // Main loop:
    // Perform simulation steps of 64 milliseconds
    // and leave the loop when the simulation is over
    while (step(_time_step) != -1)
    {

        //Refresh encoders values
        _left_encoder = getLeftEncoder();
        _right_encoder = getRightEncoder();

        //Converts rad to meters
        _distance = _left_encoder/ENCODER_RESOLUTION*WHEEL_RADIO;

        print_odometry_data();

        //Keeps the robot straight
        if(_left_encoder > _right_encoder)
        {
            _left_speed = 90;
            _right_speed = 100;
        }
        else
        {
            _left_speed = 100;
            _right_speed = 90;
        }

        setSpeed(_left_speed, _right_speed);

        if (_distance > DESIRED_DISTANCE)
        {
            _left_speed = 0;
            _right_speed = 0;

            setSpeed(_left_speed, _right_speed);
        }
    }
}

//////////////////////////////////////////////

void MyRobot::print_odometry_data()
{
    cout<< "Left Enc: " << _left_encoder << endl << "Right Enc: " << _right_encoder
        << endl << "Distance: " << _distance << endl;
}
