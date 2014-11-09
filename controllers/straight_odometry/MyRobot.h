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

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
#include <webots/DifferentialWheels.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

#define MAX_SPEED 100
#define ENCODER_RESOLUTION 5
#define WHEEL_RADIO 0.0825
#define DESIRED_DISTANCE 18


// Here is the declaration class of your controller.
// This class declares how to initialize and how to run your controller.
// Note that this class derives from DifferentialWheels who derives from Robot,
// both inherits all its functions
class MyRobot : public DifferentialWheels {
    private:

        double _left_encoder, _right_encoder;

        double _left_speed, _right_speed;

        double _distance;

        int _time_step;

    public:
        // You may need to define your private methods or variables, like
        //  Constructors, helper functions, etc.
    
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
         * @brief Prints the distance and encoders values
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
