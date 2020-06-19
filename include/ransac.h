#ifndef _RANSAC_H_
#define _RANSAC_H_
#include<cstdlib>
#include"fitline.h"
#include<iostream>
#include<ctime>
#include<cstring>
#include<cstdio>

float Ransac(Point2D32f* points, size_t Cnt, float *lines);
float Ransac(Point2D32f* points, size_t Cnt, float *line, int numForEstimate, float successProbability, float maxOutliersPercentage);
#endif
