#pragma once

#include <math.h>

const double kDeg2rad = M_PI / 180.;
const double kRad2deg = 180. / M_PI;

// param of lidar
const int kScanSize = 1450;  // 1450个扫描束每周
const int kPixelPerMeter = 50;  // 每个像素2cm， 50个像素1m


// value in map
const int kFreeSpace = 155;
const int kUnknown = 255;
const int kObstacle = 0;

// move
const double kPositionTolerance = 3;
const double kAngleTolerance = 3;
