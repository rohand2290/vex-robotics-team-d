#ifndef DEPEND_H
#define DEPEND_H

// includes all header files...
// there is no need to import within the other files...

#include "variables.h"

#define TERMINATE() while (true) pros::delay(1000)
#define ABS(x) x > 0 ? x : -x
#define ARE_SAME(x, y) ABS(x - y) < ACCURACY ? true : false

#include "api.h"
#include <math.h>
#include <vector>
#include <chrono>

#include "cartline.h"
#include "vectorxd.h"
#include "autons.h"
#include "items.h"
#include "robot.h"
#include "waypoint.h"
#include "location.h"
#include "filters.h"

double sin(double);
double cos(double);

#endif // DEPEND_H