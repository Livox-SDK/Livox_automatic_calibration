#ifndef FITLINE_H
#define FITLINE_H
#include"type.h"
#include<cmath>
int FitLine2D( Point2D32f * points, int _count, float *weights, float *line );
double CalcDist2D( Point2D32f * points, int count, float *_line, float *dist );
int FitLine2D(Point2D32f * points, int count, float *line);
void WeightL1( float *d, int count, float *w );
void WeightL12( float *d, int count, float *w );
void WeightHuber( float *d, int count, float *w, float _c );
void WeightFair( float *d, int count, float *w, float _c );
void WeightWelsch( float *d, int count, float *w, float _c );
double max(double a, double b);
#endif
