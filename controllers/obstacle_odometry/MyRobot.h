/**
 * @file    MyRobot.cpp
 * @brief   The robot goes to the other side of the map with odometry
 *
 * @author  Miguel Angel de Miguel Paraiso <100292950@alumnos.uc3m.es>
 * @date    09-11-2014
 */

// You may need to add general include files such as
// <iostream>, <cmath>, etc.

#include <iostream>
#include <cmath>

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
#include <webots/DifferentialWheels.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

#define MAX_SPEED 100
#define ENCODER_RESOLUTION 5
#define WHEEL_RADIO 0.0825
#define DISTANCE_BETWEEN_WHEELS 0.32
#define X_0 0
#define Y_0 0
#define Z_0 -9
#define ANGLE_0 0
#define POSITION_ERROR 0.25


// Here is the declaration class of your controller.
// This class declares how to initialize and how to run your controller.
// Note that this class derives from DifferentialWheels who derives from Robot,
// both inherits all its functions
class MyRobot : public DifferentialWheels {
    private:

        double _left_encoder, _right_encoder;
        double _last_left_encoder, _last_right_encoder;

        double _left_speed, _right_speed;

        double _distance;
        double _angle;
        double _absolut_z, _absolut_y, _absolut_angle;

        int _status;

        //  Compass* _my_compass;
        int _time_step;

        /**
         * @brief Sets the encoders to 0 and goes to the next step
         * @param
         * @return
         */
        void reset_status();

        /**
         * @brief Refresh the position and orientation values
         * @param
         * @return
         */
        void refresh_odometry();

        /**
         * @brief The robot goes to the requested point
         * @param The coordenades Z and Y of the requested point
         * @return
         */
        void go_to_point(double z, double y);

    public:
    
        /**
         * @brief Empty constructor of the class.
         * @param
         * @return
         */
        MyRobot();

        /**
         * @brief Destructor of the class.
         * @param
         * @return
         */
        ~MyRobot();

        /**
         * @brief Shows the odometry data
         * @param
         * @return
         */
        void print_odometry_data();

        /**
         * @brief User defined function for initializing and running the template class.
         * @param
         * @return
         */
        void run();
};
