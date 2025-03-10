#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

 inline pros::Motor intake(-17);
 inline pros::Motor lb(-10);
 inline pros::adi::DigitalOut mogoClamp('H', false);
 inline pros::adi::DigitalOut doinker('A');
 inline pros::Rotation lbRotation(-1);