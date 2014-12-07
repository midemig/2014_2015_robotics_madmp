/**
 * @file    final_controller.cpp
 * @brief   A controller rescue 2 green cylinders
 *
 * @author  Miguel Angel de Miguel Paraiso <100292950@alumnos.uc3m.es>
 * @author  Ainoa Zugasti Hernandez        <100292754@alumnos.uc3m.es>
 * @date    05-12-2014
 */

#include "MyRobot.h"

// This is the main program of your controller.
// It creates an instance of your Robot subclass, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

/**
 * @brief Main program.
 */
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
