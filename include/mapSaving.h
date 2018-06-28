#pragma once

#include <opencv2/opencv.hpp>
#include <mapHandler.h>
#include <iostream>
#include <fstream>

using namespace std;

void savemap(PLSLAM::MapHandler* map, string path);
void saveStvoTfw(const vector<pair<int,Matrix4d>>& vStvoTfw, string path);