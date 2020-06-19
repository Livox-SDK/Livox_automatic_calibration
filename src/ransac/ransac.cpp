#include"ransac.h"
using namespace std;

//返回有效点的比例
float Ransac(Point2D32f* points, size_t Cnt, float *line)
{
	int numDataObjects = Cnt;
	int numForEstimate = Cnt*0.1;
	int maxVoteCnt = 0;
	float tempLine[4];
	float inliersPercentage = 0.0;
	
	//随机抽取一定比例的点
	int ransac_times = 500;
	int *Chosen = new int[numDataObjects];

	Point2D32f *subPoints = new Point2D32f[numForEstimate];
	int pointCnt = 0;
	int voteCnt = 0;
	for(int i = 0; i < ransac_times; i++)
	{
		//随机抽取 
		//randomly select data for exact model fit ('numForEstimate' objects).
        memset(Chosen,0,numDataObjects*sizeof(int));
        int maxIndex = numDataObjects-1;
		for(int j = 0; j < numForEstimate; j++)
		{
			int selectedIndex = rand() % numDataObjects;
			Chosen[selectedIndex] = 1;
		}
		//拟合
		pointCnt = 0;
		for(int k = 0; k < numDataObjects; k++)
		{
			if(Chosen[k])
			{
				subPoints[pointCnt].x = points[k].x;
				subPoints[pointCnt].y = points[k].y;
				pointCnt++;
			}
		}
		FitLine2D(subPoints, pointCnt, tempLine);
		float a = tempLine[1]/tempLine[0];            
		float b = tempLine[3] - a*tempLine[2];
		
		
		//拟合完整之后要对拟合的结果进行鉴定，选出最优的结果
		voteCnt = 0;
		for(int k = 0; k < Cnt; k++)
		{
			//如果在直线上或者附近
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
		//当inliers的比例比较高的时候就可以直接取该值作为最优解
//		if(inliersPercentage > 0.2)
//		{
//			return inliersPercentage;
//		}
	}
	return inliersPercentage;

}

float Ransac
(
	Point2D32f* points, 
	size_t Cnt, 
	float *line,
	int numForEstimate,
	float successProbability,
	float maxOutliersPercentage
){

	//1 − p = (1 − w^n)^k
	//p = 
	//float outlierPercentage = maxOutliersPercentage;//估计值
	float numerator = log(1.0f-successProbability);
	float denominator = log(1.0f- pow(1.0-maxOutliersPercentage, numForEstimate));
	//随机抽取一定比例的点
	int ransac_times = (int)(numerator/denominator + 0.5);
	
	//printf("ransac_times： %d\n", ransac_times);
	int numDataObjects = Cnt;
	//int numForEstimate = Cnt*0.1;
	int maxVoteCnt = 0;
	float tempLine[4];
	float inliersPercentage = 0.0;
	
	
	int *Chosen = new int[numDataObjects];

	Point2D32f *subPoints = new Point2D32f[numForEstimate];
	int pointCnt = 0;
	int voteCnt = 0;
	for(int i = 0; i < ransac_times; i++)
	{
		//随机抽取 
		//randomly select data for exact model fit ('numForEstimate' objects).
        memset(Chosen,0,numDataObjects*sizeof(int));
        int maxIndex = numDataObjects-1;
		for(int j = 0; j < numForEstimate; j++)
		{
			int selectedIndex = rand() % numDataObjects;
			Chosen[selectedIndex] = 1;
		}
		//拟合
		pointCnt = 0;
		for(int k = 0; k < numDataObjects; k++)
		{
			if(Chosen[k])
			{
				subPoints[pointCnt].x = points[k].x;
				subPoints[pointCnt].y = points[k].y;
				pointCnt++;
			}
		}
		FitLine2D(subPoints, pointCnt, tempLine);

		float a = tempLine[1]/tempLine[0];          
		float b = tempLine[3] - a*tempLine[2];
		
		
		//拟合完整之后要对拟合的结果进行鉴定，选出最优的结果
		voteCnt = 0;
		for(int k = 0; k < Cnt; k++)
		{
			//如果在直线上或者附近
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
		//当inliers的比例比较高的时候就可以直接取该值作为最优解
//		if(inliersPercentage > 0.2)
//		{
//			return inliersPercentage;
//		}
	}
	return inliersPercentage;
}
            
            
            
            
