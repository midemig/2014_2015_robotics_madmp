/**
 * @file    MyRobot.h
 * @brief   A controller rescue 2 green cylinders
 *
 * @author  Miguel Angel de Miguel Paraiso <100292950@alumnos.uc3m.es>
 * @author  Ainoa Zugasti Hernandez        <100292754@alumnos.uc3m.es>
 * @date    05-12-2014
 */

#include <iostream>
#include <cmath>
#include <unistd.h>
#include <sstream>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;


#define MAX_SPEED    75
#define COLOUR_ERROR 10
#define ENCODER_RESOLUTION 5
#define WHEEL_RADIO 0.0825
#define DISTANCE_BETWEEN_WHEELS 0.32
#define NUM_DISTANCE_SENSOR 16
#define FOLLOWING_DISTANCE      100
#define ANGLE_OF_IGNORE      40
#define ANGLE_OF_AVOID_WALL  20
#define TURNING_TIME  20


class MyRobot : public DifferentialWheels {
    private:
        int _time_step;

        Camera * _forward_camera;
        Camera * _spherical_camera;
        int _image_width_f;
        int _image_height_f;
        int _image_width_s;
        int _image_height_s;
        const unsigned char *_image_s;
        const unsigned char *_image_f;
        bool _stopped;
        long long _time_counter;
        int _counter;
        double _left_encoder, _right_encoder;
        double _last_left_encoder, _last_right_encoder;
        double _absolut_z, _absolut_y, _odometry_angle;
        DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];
        double _dist_val[NUM_DISTANCE_SENSOR];

        Compass * _my_compass;
        GPS * _gps;

        double _angle;
        double _cylinder_angle;
        double _desired_angle;
        int _position_first_angle[3];
        double _left_speed, _right_speed;
        bool _front_cylinder;
        bool cylinder_mode;
        bool obstacle_avoid_mode;
        double _first_cylinder_z;
        double _first_cylinder_y;
        bool _first_cylinder;
        bool _second_cylinder;
        bool _turning;

        enum Status {
                   CYLINDER,
                   OBSTACLE,
                   POINT
                };
        Status _status;

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

        /**
          * @brief Detects if there is a cylinder a calculates its angle
          * @param
          * @return return true if detects a cylinder
          */
        bool cylinder_angle();

        /**
          * @brief Refresh the data of the compass sensors
          * @param
          * @return
          */
        void refresh_angle();

        /**
          * @brief Set the motor speed to go to "_desired angle"
          * @param
          * @return
          */
        void go_to_angle();

        /**
          * @brief transfor a zone of spherical camera into an angle
          * @param
          * @return the angle of the cylinder
          */
        double get_cylinder_angle(int zone);

        /**
          * @brief gets the angle of the first cylinder to avoid it
          * @param
          * @return
          */
        void get_first_cylinder_angle();

        /**
          * @brief Detects if there is a cylinder in front of the robot
          * @param
          * @return true if there is a cylinder in front
          */
        bool detect_front_cylinder();

        /**
          * @brief Stops the robot 2 seconds and makes it turn around
          * @param
          * @return
          */
        void cylinder_found();

        /**
          * @brief Refresh the data of the odometry (Position)
          * @param
          * @return
          */
        void refresh_odometry();

        /**
          * @brief Prints the data of the odometry
          * @param
          * @return
          */
        void print_odometry_data();

        /**
          * @brief Gets a zone of the spherical camera from an angle
          * @param The angle
          * @return
          */
        int get_cylinder_zone(double angle);

        /**
          * @brief Makes the robot stop and turn around
          * @param The time keeps turning
          * @return
          */
        void turn_around(int time);

        /**
          * @brief Refresh the data of the distance sensors
          * @param
          * @return
          */
        void get_distances();

        /**
          * @brief Choose if the robot follows left or right wall or turns around
          * @param
          * @return
          */
        void choose_situation();

        /**
          * @brief Controls the robot to follow left wall
          * @param
          * @return
          */
        void follow_left_wall();

        /**
          * @brief Controls the robot to follow right wall
          * @param
          * @return
          */
        void follow_right_wall();

        /**
          * @brief Converts the mode into motor speed
          * @param
          * @return
          */
        void send_mode_to_motors();

        /**
          * @brief Decides if the robot turns right or left
          * @param
          * @return 1 if left and 2 if right
          */
        int turning_condition();

        /**
          * @brief Prints the distance sensors data
          * @param
          * @return
          */
        void print_distance_sensor();

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
         * @brief Function for initializing and running the template class.
         * @param
         * @return
         */
        void run();

};
