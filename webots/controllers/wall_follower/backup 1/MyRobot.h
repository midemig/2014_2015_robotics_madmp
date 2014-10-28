/**
 * @file    MyRobot.h
 * @brief   A simple example for maintaining a straight line with the compass.
 *
 * @author  Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date    2014-07
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED       100
#define DESIRED_ANGLE   45
#define NUM_DISTANCE_SENSOR 8
#define FOLLOWING_DISTANCE      100

class MyRobot : public DifferentialWheels {
    private:
        int _time_step;

        Compass * _my_compass;
        double _left_speed, _right_speed;

        long _counter;
        double _dist_val[NUM_DISTANCE_SENSOR];

        DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];

        double convert_to_distance(double number);
        void convert_distances_to_meters();

        enum Mode {
                    STOP,
                    FORWARD,
                    TURN_LEFT,
                    TURN_RIGHT,
                    WALL_FOLLOWER
                };

        Mode _mode;

    public:
        /**
         * @brief Empty constructor of the class.
         */
        MyRobot();

        /**
         * @brief Destructor of the class.
         */
        ~MyRobot();

        /**
         * @brief User defined function for initializing and running the template class.
         */
        void run();

        /**
          * @brief An example for converting bearing vector from compass to angle (in degrees).
          */
        double convert_bearing_to_degrees(const double* in_vector);

        void print_distance_sensor();
};
