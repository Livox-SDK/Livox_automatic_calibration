#include"fitline.h"
const double eps = 1e-6;
int FitLine2D( Point2D32f * points, int _count, float *weights, float *line )
{
    double x = 0, y = 0, x2 = 0, y2 = 0, xy = 0, w = 0;
    double dx2, dy2, dxy;
    int i;
    int count = _count;
    float t;

    /* Calculating the average of x and y... */
    if( weights == 0 )
    {
        for( i = 0; i < count; i += 1 )
        {
            x += points[i].x;
            y += points[i].y;
            x2 += points[i].x * points[i].x;
            y2 += points[i].y * points[i].y;
            xy += points[i].x * points[i].y;
        }
        w = (float) count;
    }
    else
    {
        for( i = 0; i < count; i += 1 )
        {
            x += weights[i] * points[i].x;
            y += weights[i] * points[i].y;
            x2 += weights[i] * points[i].x * points[i].x;
            y2 += weights[i] * points[i].y * points[i].y;
            xy += weights[i] * points[i].x * points[i].y;
            w += weights[i];
        }
    }

    x /= w;
    y /= w;
    x2 /= w;
    y2 /= w;
    xy /= w;

    dx2 = x2 - x * x;
    dy2 = y2 - y * y;
    dxy = xy - x * y;

    t = (float) atan2( 2 * dxy, dx2 - dy2 ) / 2;
    line[0] = (float) cos( t );
    line[1] = (float) sin( t );

    line[2] = (float) x;
    line[3] = (float) y;

    return 0;
}


double CalcDist2D( Point2D32f * points, int count, float *_line, float *dist )
{
    int j;
    float px = _line[2], py = _line[3];
    float nx = _line[1], ny = -_line[0];
    double sum_dist = 0.;

    for( j = 0; j < count; j++ )
    {
        float x, y;

        x = points[j].x - px;
        y = points[j].y - py;

        dist[j] = (float) fabs( nx * x + ny * y );
        sum_dist += dist[j];
    }

    return sum_dist;
}



int FitLine2D(Point2D32f * points, int count, float *line)
{
	//进行一遍拟合，获取直线参数
	FitLine2D(points, count, NULL, line);
	//计算权值，再进行一次拟合
	float *dist  = new float[count];
	float *W = new float[count];
	
	//迭代进行加权拟合，迭代次数不小于三次
	for(int i = 0; i < 5; i++)
	{
		CalcDist2D(points, count, line, dist);
	 	WeightL1( dist, count, W);
		FitLine2D(points, count, W, line);
	}
	delete [] dist;
}

void WeightL1( float *d, int count, float *w )
{
    int i;

    for( i = 0; i < count; i++ )
    {
        double t = fabs( (double) d[i] );
        w[i] = (float)(1. / max(t, eps));
    }
}

void WeightL12( float *d, int count, float *w )
{
    int i;

    for( i = 0; i < count; i++ )
    {
        w[i] = 1.0f / (float) sqrt( 1 + (double) (d[i] * d[i] * 0.5) );
    }
}


void WeightHuber( float *d, int count, float *w, float _c )
{
    int i;
    const float c = _c <= 0 ? 1.345f : _c;

    for( i = 0; i < count; i++ )
    {
        if( d[i] < c )
            w[i] = 1.0f;
        else
            w[i] = c/d[i];
    }
}


void WeightFair( float *d, int count, float *w, float _c )
{
    int i;
    const float c = _c == 0 ? 1 / 1.3998f : 1 / _c;

    for( i = 0; i < count; i++ )
    {
        w[i] = 1 / (1 + d[i] * c);
    }
}

void WeightWelsch( float *d, int count, float *w, float _c )
{
    int i;
    const float c = _c == 0 ? 1 / 2.9846f : 1 / _c;

    for( i = 0; i < count; i++ )
    {
        w[i] = (float) exp( -d[i] * d[i] * c * c );
    }
}
double max(double a, double b){

	return a > b ? a : b;
}


