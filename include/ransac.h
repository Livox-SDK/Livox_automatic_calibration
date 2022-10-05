#ifndef _RANSAC_H_
#define _RANSAC_H_
#include<cstdlib>
// #include"fitline.h"
// #include"type.h"
#include<cmath>
#include<iostream>
#include<ctime>
#include<cstring>
#include<cstdio>
#include <vector>
// #include <cstdio>


typedef struct
{
	float x;
	float y;
}Point2D32f;

// float Ransac(Point2D32f* points, size_t Cnt, float *lines);
float Ransac(std::vector<Point2D32f>& points, size_t Cnt, float *line, int numForEstimate, float successProbability, float maxOutliersPercentage);
#endif
