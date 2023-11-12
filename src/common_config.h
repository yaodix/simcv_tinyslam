#pragma once


const double kDeg2rad = M_PI / 180.;
const double kRad2deg = 180. / M_PI;

// param of lidar
const int kScanSize = 1450;  // 1450个扫描束每周
const int kPixelPerMeter = 20;  // 每个像素5cm， 20个像素1m


// value in map
const int kFreeSpace = 255;
const int kUnknown = 55;
const int kObstacle = 0;