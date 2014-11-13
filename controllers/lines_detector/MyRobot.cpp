/**
 * @file    MyRobot.h
 * @brief   A controller to detect yellow lines with spherical camera
 *
 * @author  Miguel Angel de Miguel Paraiso <100292950@alumnos.uc3m.es>
 * @date    13-11-2014
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    _spherical_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    int sum = 0;
    unsigned char green = 0, red = 0, blue = 0;
    double percentage_yellow = 0.0;

    // Get size of images for spherical camera
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();
    cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;

    while (step(_time_step) != -1) {
        sum = 0;

        // Get current image from spherical camera
        const unsigned char *image_s = _spherical_camera->getImage();

        // Count number of pixels that are yellow
        for (int x = 0; x < image_width_s; x++) {
            for (int y = 0; y < image_height_s; y++) {
                green = _forward_camera->imageGetGreen(image_s, image_width_s, x, y);
                red = _forward_camera->imageGetRed(image_s, image_width_s, x, y);
                blue = _forward_camera->imageGetBlue(image_s, image_width_s, x, y);

                if ((green > 255-COLOUR_ERROR) && (red > 255-COLOUR_ERROR) && (blue < COLOUR_ERROR)) {
                    sum = sum + 1;
                }
            }
        }

        percentage_yellow = (sum / (float) (image_width_f * image_height_f)) * 100;
        cout << "Percentage of Line in spherical camera image: " << percentage_yellow << endl;

        //If there is enough yellow (assumed to have pixel value error of "COLOUR_ERROR")
        if(percentage_yellow >= 0.1400)
            cout << "Line" << endl;
        else
            cout << "No Line" << endl;

        // Turn around slowly
        _left_speed = 5;
        _right_speed = -5;

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

