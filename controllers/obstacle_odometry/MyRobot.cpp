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
    _status = 1;
    _distance = 0;
    _angle = 0;
    _absolut_angle = ANGLE_0;
    _absolut_z =  Z_0;
    _absolut_y = Y_0;
    _last_left_encoder = 0;
    _last_right_encoder = 0;
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
refresh_odometry();

        //This is the list of actions that the robot has to do
        switch (_status)
        {
        case 1:
            go_to_point(-4,4);
            break;
        case 2:
            go_to_point(4,4);
            break;
        case 3:
            go_to_point(9,0);
            break;
        case 4:
            go_to_point(4,-4);
            break;
        case 5:
            go_to_point(-4,-4);
            break;
        case 6:
            go_to_point(-9,0);
            break;
        case 7:
            _status = 1;
            break;
        default:
            break;
        }

    }
}

//////////////////////////////////////////////

void MyRobot::print_odometry_data()
{
    cout<< "Left Enc: " << _left_encoder << endl << "Right Enc: " << _right_encoder
        << endl << "Distance: " << _distance << " - Angle: " << _angle << " - Status: " <<_status << endl
        << "Z: " << _absolut_z << " - Y: " << _absolut_y << " - O_Angle: " <<_absolut_angle*57.29 << endl;
}

//////////////////////////////////////////////

void MyRobot::reset_status()
{
    _left_speed = 0;
    _right_speed = 0;

    setSpeed(_left_speed, _right_speed);
    _distance = 0;
    _angle = 0;
    setEncoders(0,0);
    _status ++;
    print_odometry_data();
    _last_left_encoder = 0;
    _last_right_encoder = 0;
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
    _absolut_z = _absolut_z + ((right_increment + left_increment)/2)*cos(_absolut_angle + ((right_increment - left_increment)/2*DISTANCE_BETWEEN_WHEELS))*WHEEL_RADIO;
    _absolut_y = _absolut_y + ((right_increment + left_increment)/2)*sin(_absolut_angle + ((right_increment - left_increment)/2*DISTANCE_BETWEEN_WHEELS))*WHEEL_RADIO;
    _absolut_angle = _absolut_angle + ((right_increment - left_increment)/DISTANCE_BETWEEN_WHEELS)*WHEEL_RADIO;

    //Corrects the angle to a range of [-180,180] degrees
    if(_absolut_angle > 180/57.29)
        _absolut_angle = _absolut_angle - 360/57.29;

    if(_absolut_angle < -180/57.29)
        _absolut_angle = _absolut_angle + 360/57.29;

    //Refresh the data of last encoder value
    _last_left_encoder = _left_encoder;
    _last_right_encoder = _right_encoder;
}

//////////////////////////////////////////////

void MyRobot::go_to_point(double z, double y)
{
    print_odometry_data();

    double desired_angle = (atan2((y - _absolut_y),(z - _absolut_z)))*57.29;

    //Corrects trayectory with big angles
    if(desired_angle > 150 && _absolut_angle < 0)
        desired_angle = desired_angle - 360;

    if(desired_angle < -150 && _absolut_angle > 0)
        desired_angle = desired_angle + 360;

    //The robot turns proportionally to the error of its orientation
    if (_absolut_angle*57.29 > desired_angle)
    {
        _left_speed = 50 + abs(_absolut_angle*57.29 - desired_angle)/4.5;
        _right_speed = 50 - abs(_absolut_angle*57.29 - desired_angle)/4.5;
    }
    else
    {
        _left_speed = 50 - abs(_absolut_angle*57.29 - desired_angle)/4.5;
        _right_speed = 50 + abs(_absolut_angle*57.29 - desired_angle)/4.5;
    }
    setSpeed(_left_speed, _right_speed);

    cout << "Desired angle:" << desired_angle << endl;

    if((_absolut_z > (z - POSITION_ERROR)) && (_absolut_z < (z + POSITION_ERROR)) && (_absolut_y > (y - POSITION_ERROR)) && (_absolut_y < (y + POSITION_ERROR)))
    {
        reset_status();
    }

}
