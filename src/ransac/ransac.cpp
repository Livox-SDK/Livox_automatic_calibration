#include "ransac.h"
using namespace std;

void WeightL1( float *d, int count, float *w )
{
    int i;

    for( i = 0; i < count; i++ )
    {
        double t = fabs( (double) d[i] );
        w[i] = (float)(1. / max(t, 0.000000000000001));
    }
}

int FitLine2D( vector<Point2D32f>& points, int _count, float * weights, float *line  , bool haveWeight)
{
    std::cout<<"FitLine2D---------- Started the 4Input ...\n\n";

    double x = 0, y = 0, x2 = 0, y2 = 0, xy = 0, w = 0;
    double dx2, dy2, dxy;
    int i;
    int count = _count;
    float t;
    std::cout<<"4Input 111...\n\n";

    /* Calculating the average of x and y... */
    if( !haveWeight )
    {
        for( i = 0; i < count; i += 1 )
        {
            x += points[i].x;
            y += points[i].y;
            x2 += points[i].x * points[i].x;
            y2 += points[i].y * points[i].y;
            xy += points[i].x * points[i].y;
			            
			x += points[i].x;
            y += points[i].y;
            x2 += points[i].x * points[i].x;
            y2 += points[i].y * points[i].y;
            xy += points[i].x * points[i].y;
        }
        w = (float) count;

        std::cout<<"4Input 222...\n\n";

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
        std::cout<<"4Input 333...\n\n";

    }
    std::cout<<"4Input 444...\n\n";

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
    
    std::cout<<"4Input 555...\n\n";

    return 0;
}


double CalcDist2D( vector<Point2D32f>& points, int count, float *_line, float *dist )
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

int FitLine2D_1(vector<Point2D32f>& points, int count, float * line)
{
	// FitLine2D(points, count, NULL, line , false);
    
    // std::cout<<"FitLine2D---------000...\n\n";

	// //计算权值，再进行一次拟合
	// float *dist  = new float[count];
	// float *W = new float[count];
	
	// //迭代进行加权拟合，迭代次数不小于三次
    // // int j = 0;
	// // while(1)
	// // {
    // //     std::cout<<"FitLine2D---------  j: " << to_string(j)<<" 0 ...\n";

	// // 	// CalcDist2D(points, count, line, dist);
    // //     std::cout<<"FitLine2D---------  j: " << to_string(j)<<" 1 ...\n";

	// //  	// WeightL1( dist, count, W);
    // //     std::cout<<"FitLine2D---------  j: " << to_string(j)<<" 2 ...\n";

	// // 	// FitLine2D(points, count, W, line , true);

    // //     j++;
    // //     if (j>3) break;
	// // }

    // std::cout<<"FitLine2D---------- Before delete ...\n\n";

	// delete[] dist;
	// delete[] W;

    std::cout<<"FitLine2D---------- After delete ...\n\n";

}



float Ransac( 	vector<Point2D32f>& points, 
				size_t Cnt, 
				float *line,
				int numForEstimate,
				float successProbability,
				float maxOutliersPercentage )
{



	//1 − p = (1 − w^n)^k
	//p = 
	//float outlierPercentage = maxOutliersPercentage;//估计值
	float numerator = log(1.0f-successProbability);
	float denominator = log(1.0f- pow(1.0-maxOutliersPercentage, numForEstimate));
	
	int ransac_times = (int)(numerator/denominator + 0.5);
	ransac_times = 1;
	
	//printf("ransac_times： %d\n", ransac_times);
	int numDataObjects = Cnt;
	//int numForEstimate = Cnt*0.1;
	int maxVoteCnt = 0;
	float* tempLine = new float[4];
	float inliersPercentage = 0.0;
	std::cout<<"RRR---------------------ransac...\n\n";
	
	int *Chosen = new int[numDataObjects];

	// Point2D32f *subPoints = new Point2D32f[numForEstimate];
	vector<Point2D32f> subPoints;

	int pointCnt = 0;
	int voteCnt = 0;
	std::cout<<"RRR---------------------start loop...\n\n";

	for(int i = 0; i < ransac_times; i++)
	{
		//randomly select data for exact model fit ('numForEstimate' objects).
        memset(Chosen,0,numDataObjects*sizeof(int));
        int maxIndex = numDataObjects-1;
		for(int j = 0; j < numForEstimate; j++)
		{
			int selectedIndex = rand() % numDataObjects;
			Chosen[selectedIndex] = 1;
		}
		
		pointCnt = 0;
		for(int k = 0; k < numDataObjects; k++)
		{
			if(Chosen[k])
			{
				// subPoints[pointCnt].x = points[k].x;
				// subPoints[pointCnt].y = points[k].y;
				Point2D32f tmp;
				tmp.x = points[k].x;
				tmp.y = points[k].y;
				subPoints.push_back(tmp);

				pointCnt++;
			}
		}
		std::cout<<"RRR---------- "<<to_string(i)<< " ---------Before FitLine2D...\n";

		FitLine2D_1(subPoints, pointCnt, tempLine);

		std::cout<<"RRR---------- "<<to_string(i)<< " ---------After FitLine2D...\n";

		float a = tempLine[1]/tempLine[0];          
		float b = tempLine[3] - a*tempLine[2];
		
		
		voteCnt = 0;
		for(int k = 0; k < Cnt; k++)
		{
			if(abs(points[k].y - a*points[k].x - b) < 2)
			{
				voteCnt++;
			}
		}

		if(voteCnt > maxVoteCnt)
		{
			maxVoteCnt = voteCnt;
			inliersPercentage = (float)maxVoteCnt/Cnt;
//			printf("a: %f\tb%f\tpercent: %f\n", a, b, inliersPercentage);
			for(int m = 0; m < 4; m++)
			{
				line[m] = tempLine[m];
			}
			
		}	
//		if(inliersPercentage > 0.2)
//		{
			// delete[] Chosen;	
			// delete[] subPoints;
//			return inliersPercentage;
//		}

	}

	std::cout<<"RRR----------Loop over...\n\n";

	delete[] tempLine;
	delete[] Chosen;	
	// delete[] subPoints;
	return inliersPercentage;
}
            
            
            
            
