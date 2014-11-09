/**
 * @file    MyRobot.h
 * @brief   A controller to avoid obstacles
 *
 * @author  Miguel Angel de Miguel Paraiso
 * @date    09-11-2014
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED       75
#define DESIRED_ANGLE   45
#define NUM_DISTANCE_SENSOR 16
#define FOLLOWING_DISTANCE      100
#define CRITICAL_DISTANCE      1000
#define DESIRED_ANGLE 45

class MyRobot : public DifferentialWheels {
    private:
        int _time_step;

        Compass * _my_compass;
        double _left_speed, _right_speed;

        long _counter;
        double _compass_angle;
        double _dist_val[NUM_DISTANCE_SENSOR];
        int _minor_distance_sensor;
        bool _critical_mode;
        int _critical_sensor;
        double _angle_of_wall;

        DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];

        /**
          * @brief Refresh the data of the distance sensors
          * @param
          * @return
          */
        void get_distances();

        /**
          * @brief Controls the robot in a difficult situation
          * @param
          * @return
          */
        void difficult_situation();

        /**
          * @brief Controls the robot to follow a wall
          * @param
          * @return
          */
        void follow_wall();

        /**
          * @brief Converts the mode into motor speed
          * @param
          * @return
          */
        void send_mode_to_motors();

        /**
          * @brief Follows the compass
          * @param
          * @return
          */
        void follow_compass(double angle);

        /**
          * @brief Special condition for an if
          * @param
          * @return bool
          */
        bool turning_condition();

        enum Mode {
                    FORWARD,
                    TURN_LEFT,
                    TURN_RIGHT,
                    TURN_OVER_EDGE_LEFT,
                    TURN_OVER_EDGE_RIGHT,
                    TURN_AROUND_LEFT,
                    TURN_AROUND_RIGHT
                };

        Mode _mode;

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
         * @brief User defined function for initializing and running the template class.
         * @param
         * @return
         */
        void run();

        /**
          * @brief Converting bearing vector from compass to angle (in degrees).
          * @param
          * @return
          */
        double convert_bearing_to_degrees(const double* in_vector);

        /**
          * @brief Prints distance data
          * @param
          * @return
          */
        void print_distance_sensor();
};
