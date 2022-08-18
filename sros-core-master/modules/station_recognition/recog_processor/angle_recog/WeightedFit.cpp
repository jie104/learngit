#include "WeightedFit.h"
#include<cstdlib>
#include<cstdio>

int VoteBox[10000000];

//~ CV_IMPLEMENT_QSORT( IntQSort, int, cmp_pts )  // 该宏利用声明并定义函数IntQSort用于快速排序
int W[MAX_FITPOINTS_CNT];

// =(int * )malloc(sizeof(int) * Cnt);// 权值系数
int WeightedFit(int X[], int Y[], int Cnt, LinePara *EstLinePara) {
    // 加权最小二乘法
    // Cnt: 数据点计数
    // EstLinePara : 直线拟合的估计值，可以利用最小二乘法计算得到
    // 利用最小二乘进行估计
    int *Tmp;
    int FlagFlip = 0;// 是否对X,Y进行翻转过
    //FitPara(X , Y , Cnt , EstLinePara , NULL);
    //if(abs(EstLinePara->a) > 1 || EstLinePara->a == NAN || EstLinePara->a == -NAN)
    if (abs(X[0] - X[Cnt - 1]) < abs(Y[0] - Y[Cnt - 1])) {
//        std::cout << "斜率太小 ---- " << EstLinePara->a << std::endl;
        // 该段直线为斜率大于1
        // 沿45度线进行翻转
        // 将 X 和 Y 进行翻转
        Tmp = X;
        X = Y;
        Y = Tmp;
        FlagFlip = 1;  // 翻转
    }
//    FitPara(X , Y , Cnt , EstLinePara , NULL);
    //if(abs(EstLinePara->a) > 1 || EstLinePara->a == NAN || EstLinePara->a == -NAN)
    int i = 0;
    if (W == NULL)
        return -1;
    // Cal 3 times
    //   for(i = 0 ; i < 3 ; i++)
//    {
    // 计算权值
//        CalW(X , Y ,Cnt , EstLinePara , W );
    FitPara(X, Y, Cnt, EstLinePara, NULL);// 根据权值拟合参数
    //   }
    //   free(W);
    // EstLinePara->Dis = abs(EstLinePara->b)/(sqrt(EstLinePara->a * EstLinePara->a + EstLinePara->b * EstLinePara->b));
//    EstLinePara->a=-EstLinePara->a;
    if (FlagFlip == 0) {
        // 未翻转
        EstLinePara->theta = atan(EstLinePara->a);
    }
    else {
        // 翻转过
        if (abs(EstLinePara->a) < 0.00001) {
            EstLinePara->a = 100000;
//            std::cout << "斜率太小 ---- " << EstLinePara->a << std::endl;
        }
        else {
            EstLinePara->a = 1.0 / EstLinePara->a;
        }
        EstLinePara->b = -EstLinePara->b * (EstLinePara->a);
        EstLinePara->theta = atan(EstLinePara->a);
    }

    //X Y若翻转过，再翻转回去
    if (FlagFlip == 1) {
        // 该段直线为斜率大于1
        // 沿45度线进行翻转
        // 将 X 和 Y 进行翻转
        Tmp = X;
        X = Y;
        Y = Tmp;
    }
    //计算线段的两个端点
    if (abs(EstLinePara->a) >= 1) {
        EstLinePara->startPoint.y = Y[0];
        EstLinePara->startPoint.x = (Y[0] - EstLinePara->b) / EstLinePara->a;

        EstLinePara->endPoint.y = Y[Cnt - 1];
        EstLinePara->endPoint.x = (Y[Cnt - 1] - EstLinePara->b) / EstLinePara->a;
    } else {
        EstLinePara->startPoint.x = X[0];
        EstLinePara->startPoint.y = EstLinePara->a * X[0] + EstLinePara->b;

        EstLinePara->endPoint.x = X[Cnt - 1];
        EstLinePara->endPoint.y = EstLinePara->a * X[Cnt - 1] + EstLinePara->b;
    }
    float deltaX = EstLinePara->startPoint.x - EstLinePara->endPoint.x;
    float deltaY = EstLinePara->startPoint.y - EstLinePara->endPoint.y;
    EstLinePara->length = sqrt(deltaX * deltaX + deltaY * deltaY);//计算路径长度
    EstLinePara->angError = 2 * atan((EstLinePara->RMSE * 2 + 10) / EstLinePara->length);//角度偏差　＝　atan(2*RMSE/length)

    EstLinePara->disError = 2 * (EstLinePara->RMSE + 10)/ 1000;//距离偏差　＝　拟合偏差（mm）+ 传感器偏差（10mm）
//    cout << "disError———————————　" << EstLinePara->disError << endl;
//    cout << "angError———————————　" << 100 * EstLinePara->angError << endl;
//    cout << "disError/angError——　" << EstLinePara->disError / EstLinePara->angError << endl;
    EstLinePara->lineCov << 0, 0, 0, 0;
    EstLinePara->lineCov(1,1) = EstLinePara->disError * EstLinePara->disError;
    EstLinePara->lineCov(0,0) = EstLinePara->angError * EstLinePara->angError;
    EstLinePara->midPoint.x = 0.5f * (EstLinePara->startPoint.x + EstLinePara->endPoint.x);
    EstLinePara->midPoint.y = 0.5f * (EstLinePara->startPoint.y + EstLinePara->endPoint.y);
    float temp_midAngle = atan2(-EstLinePara->midPoint.y, -EstLinePara->midPoint.x);
//    cout << "temp_midAngle——　" << temp_midAngle * 180.0f / PI << endl;
//    cout << "midPoint.x————　" << EstLinePara->midPoint.x << endl;
//    cout << "midPoint.y————　" << EstLinePara->midPoint.y << endl;
//    cout << "theta——————————　" << EstLinePara->theta * 180.0f / PI << endl;

    EstLinePara->lineRho = fabs(EstLinePara->b) / sqrt(EstLinePara->a * EstLinePara->a + 1);
    float temp =EstLinePara->theta + M_PI_2;
//    cout << "temp ——————————" << temp * 180.0f / PI << endl;
    if (temp >= PI){
        temp -= PI;
    }else if(temp <= -PI){
        temp += PI;
    }
//    cout << "temp2 ——————————" << temp * 180.0f / PI << endl;
    float temp2 = temp - temp_midAngle;
    if (temp2 >= PI){
        temp2 -= PI2;
    }else if(temp2 <= -PI){
        temp2 += PI2;
    }
//    if (fabs(EstLinePara->theta + M_PI_2 - temp_midAngle) < M_PI_2) {//如果fabs(EstLinePara->theta + 90 - temp_midAngle) < 90，则表示+90为法线
    if (fabs(temp2) < M_PI_2) {//如果fabs(EstLinePara->theta + 90 - temp_midAngle) < 90，则表示+90为法线
        EstLinePara->normalAngle = EstLinePara->theta + M_PI_2;
    }else{
        EstLinePara->normalAngle = EstLinePara->theta - M_PI_2;
    }

//    cout << "normalAngle————"<< EstLinePara->normalAngle * 180.0f / PI << endl;


//    cout << "EstLinePara->length——————　" << EstLinePara->length << endl;
//    cout << "EstLinePara->angError————　" << EstLinePara->angError << endl;
//    cout << "EstLinePara->disError————　" << EstLinePara->disError << endl;
    //   printf("the nihe point is:start:%d,%d,end:%d,%d\nthe lase point is:start:%d,%d,end:%d,%d\n\n",EstLinePara->startPoint.x,
    //		   EstLinePara->startPoint.y, EstLinePara->endPoint.x,EstLinePara->endPoint.y,X[0],Y[0],X[Cnt-1],Y[Cnt-1]);
    return 0;
}

int Cmp(const void *a, const void *b) {
    return *(int *) a - *(int *) b;
}

int Med(int R[], int Cnt)// 求取中值
{
    qsort(R, Cnt, sizeof(R[0]), Cmp);
    //~ IntQSort(R , Cnt , 0);
    return R[Cnt / 2];
}

int CalW(int X[], int Y[], int Cnt, LinePara *EstLinePara, int W[]) {
    int i = 0;
    double a = (double) EstLinePara->a;
    double b = (double) EstLinePara->b;

    int Median = 0;
    double u;
    double tmp;
    for (i = 0; i < Cnt; i++) {
        tmp = (int) abs(Y[i] - a * X[i] - b);
        W[i] = tmp;

    }
    Median = Med(W, Cnt);
    Median = Median > 2 ? Median : 2;

    for (i = 0; i < Cnt; i++) {
        u = (double) (W[i] / (line_K * Median));

        if (u < 1) {
            W[i] = (int) ((1 - u * u) * (1 - u * u) * 100);   //将W范围限制在0-100
            //W[i] = (int)((1-u)*(1-u)*100);
        }
        else {
            W[i] = 0;
        }
    }

    return 0;
}

int FitPara(int X[], int Y[], int Cnt, LinePara *EstLinePara, int W[]) {

    int i = 0;
    long long P1 = 0; // sum(wi*xi*yi);
    long long P2 = 0; // sum(wi * xi * xi)
    long long P3 = 0; // sum(wi * xi)
    long long P4 = 0; // sum(wi * yi)
    long long P5 = 0; // sum(wi)



    if (W == NULL) // 直接进行最小二乘拟合，即所有数据的权值相等
    {
        //
        for (i = 0; i < Cnt; i++) {
            P1 += X[i] * Y[i];
            P2 += X[i] * X[i];
            P3 += X[i];
            P4 += Y[i];
            P5 += 1;
        }
    }
    else { //加权最小二乘拟合
        for (i = 0; i < Cnt; i++) {
            P1 += W[i] * X[i] * Y[i];
            P2 += W[i] * X[i] * X[i];
            P3 += W[i] * X[i];
            P4 += W[i] * Y[i];
            P5 += W[i];
        }
    }
    //printf("P1: %lld  P2: %lld P3: %lld P4: %lld P5: %lld\n",P1,P2,P3,P4,P5);

    EstLinePara->a = (((double) (((double) P1) * ((double) P5) - P4 * P3)) /
                      ((double) (((double) P2) * ((double) P5) - P3 * P3)));
    EstLinePara->b = (P1 - P2 * EstLinePara->a) / P3;
//    EstLinePara->R = getCoefficient(X, Y, Cnt, EstLinePara->a, EstLinePara->b);
    getFitCoefficient(X, Y, Cnt, EstLinePara);
    return 0;
}

int HoughArc(int X[], int Y[], int Cnt, int r, ArcPara *Arc) {
    vector<iPoint> center;
    vector<int> VoteCnt;
    double theta;
    int a, b;
    int minA, maxA, minB, maxB;
    int VotedFlag = 0;
    double deltaTheta = PI / 180;//间隔1度
    double startAngle = 150.0 * PI / 180;
    double endAngle = PI * 2 + PI / 6;
    center.clear();
    VoteCnt.clear();
    minA = maxA = X[0] - r;
    minB = maxB = X[0]; //theta = 0
    //计算a，b的最小和最大值
    for (int i = 0; i < Cnt; i++) {
        for (theta = startAngle; theta < endAngle; theta += deltaTheta) {
            a = (int) (X[i] - r * cos(theta) + 0.5);
            b = (int) (Y[i] - r * sin(theta) + 0.5);
            if (a > maxA) {
                maxA = a;
            } else if (a < minA) {
                minA = a;
            }

            if (b > maxB) {
                maxB = b;
            } else if (b < minB) {
                minB = b;
            }

        }
    }
    //确定a，b的范围之后，即确定了票箱的大小
    int aScale = maxA - minA + 1;
    int bScale = maxB - minB + 1;

    int *VoteBox = new int[aScale * bScale];
    //VoteBox初始化为0
    for (int i = 0; i < aScale * bScale; i++) {
        VoteBox[i] = 0;
    }
    //开始投票
    for (int i = 0; i < Cnt; i++) {
        //printf("%d  ",i);
        for (theta = startAngle; theta < endAngle; theta += deltaTheta) {

            a = (int) (X[i] - r * cos(theta) + 0.5);
            b = (int) (Y[i] - r * sin(theta) + 0.5);
            VoteBox[(b - minB) * aScale + a - minA] = VoteBox[(b - minB) * aScale + a - minA] + 1;
        }
    }

    //筛选票箱
    int VoteMax = 0;
    int VoteMaxX, VoteMaxY;
    for (int i = 0; i < bScale; i++) {
        for (int j = 0; j < aScale; j++) {
            if (VoteBox[i * aScale + j] > VoteMax) {
                VoteMax = VoteBox[i * aScale + j];
                VoteMaxY = i;
                VoteMaxX = j;
            }
        }
    }

    int Count = 0;
    //printf("VoteMax: %d",VoteMax);
    for (int i = 0; i < bScale; i++) {
        for (int j = 0; j < aScale; j++) {
            if (VoteBox[i * aScale + j] >= VoteMax) {
                Count++;
            }
        }
    }
    //printf("   %d \n",Count);
    //释放内存
    delete[] VoteBox;
    if (VoteMax > 3) {
        Arc->center.x = VoteMaxX + minA;
        Arc->center.y = VoteMaxY + minB;
        Arc->r = r;
        return 1;
    } else {
        return 0;
    }





    //for (int i = 0; i < Cnt;i++)
    //{
    //	printf("%d  ",i);
    //	for (theta = 0; theta < PI*2;theta += deltaTheta)
    //	{
    //
    //		a = X[i] - r*cos(theta);
    //		b = Y[i] - r*sin(theta);
    //		//投票
    //		VotedFlag = 0;
    //		for (int j = 0; j < center.size();j++)
    //		{
    //			if (a == center.at(j).x && b == center.at(j).y)
    //			{
    //				VoteCnt.at(j) = VoteCnt.at(j) + 1;
    //				VotedFlag = 1;
    //				break;
    //			}
    //		}
    //		if (VotedFlag == 0)
    //		{
    //			center.push_back(ipoint(a,b));
    //			VoteCnt.push_back(1);
    //		}
    //	}
    //}

    //int VoteMax = 0;
    //int VoteMaxIndex = 0;
    ////投票结束，筛选票箱
    //for (int i = 0; i < center.size();i++)
    //{
    //	if (VoteCnt.at(i) > VoteMax)
    //	{
    //		VoteMaxIndex = i;
    //		VoteMax = VoteCnt.at(i);
    //	}
    //}


    //if (VoteMax > Cnt/4)
    //{
    //	Arc->center.x = center.at(VoteMaxIndex).x;
    //	Arc->center.y = center.at(VoteMaxIndex).y;
    //	Arc->r = r;
    //	return 1;

    //}else {
    //	return 0;
    //}

    return 1;

}


int HoughArc2(int X[], int Y[], int Cnt, int r, ArcPara *Arc) {
    double theta;
    int a, b;
    int minA, maxA, minB, maxB;

    int VotedFlag = 0;
    double deltaTheta = PI / 180;//间隔1度
    double startAngle = 150.0 * PI / 180;
    double endAngle = PI * 2 + PI / 6;
    minA = maxA = X[0] - r;
    minB = maxB = X[0]; //theta = 0
    int halfWindowSize = 3;
    //计算a，b的最小和最大值
    for (int i = 0; i < Cnt; i++) {
        for (theta = startAngle; theta < endAngle; theta += deltaTheta) {
            a = (int) (X[i] - r * cos(theta) + 0.5);
            b = (int) (Y[i] - r * sin(theta) + 0.5);
            if (a + 1 > maxA) {
                maxA = a + halfWindowSize;
            } else if (a - halfWindowSize < minA) {
                minA = a - halfWindowSize;
            }

            if (b + halfWindowSize > maxB) {
                maxB = b + halfWindowSize;
            } else if (b - halfWindowSize < minB) {
                minB = b - halfWindowSize;
            }

        }
    }
    //确定a，b的范围之后，即确定了票箱的大小
    int aScale = maxA - minA + 1;
    int bScale = maxB - minB + 1;

    //int *VoteBox = new int[aScale*bScale];
    //VoteBox初始化为0
    for (int i = 0; i < aScale * bScale; i++) {
        VoteBox[i] = 0;
    }
    //开始投票
    for (int i = 0; i < Cnt; i++) {
        //printf("%d  ",i);
        for (theta = startAngle; theta < endAngle; theta += deltaTheta) {

            a = (int) (X[i] - r * cos(theta) + 0.5);
            b = (int) (Y[i] - r * sin(theta) + 0.5);
            for (int m = -halfWindowSize; m < halfWindowSize; m++) {
                for (int n = -halfWindowSize; n < halfWindowSize; n++) {
                    VoteBox[(b + halfWindowSize - minB) * aScale + a + halfWindowSize - minA]
                            = VoteBox[(b + halfWindowSize - minB) * aScale + a + halfWindowSize - minA] + 1;
                }
            }

        }
    }
    //筛选票箱
    int VoteMax = 0;
    int VoteMaxX, VoteMaxY;
    for (int i = 0; i < bScale; i++) {
        for (int j = 0; j < aScale; j++) {
            if (VoteBox[i * aScale + j] > VoteMax) {
                VoteMax = VoteBox[i * aScale + j];
                VoteMaxY = i;
                VoteMaxX = j;
            }
        }
    }

    int Count = 0;
    //printf("VoteMax: %d",VoteMax);
    for (int i = 0; i < bScale; i++) {
        for (int j = 0; j < aScale; j++) {
            if (VoteBox[i * aScale + j] >= VoteMax) {
                Count++;
            }
        }
    }
    //printf("   %d \n",Count);
    //根据Hough求出的圆求取方差
    int err = 0;
    a = VoteMaxX + minA;
    b = VoteMaxY + minB;
    for (int i = 0; i < Cnt; i++) {
        err += sqrt((double) (X[i] - a) * (X[i] - a) + (Y[i] - b) * (Y[i] - b)) - r;
        if (err > Cnt * 6) {
            //printf("  err:%d ",err);
            return 0;
        }
    }
    /*printf("  err:%d ",err);*/

    //释放内存
    //delete [] VoteBox;
    if (VoteMax > (2 * halfWindowSize + 1) * (2 * halfWindowSize + 1) * 2) {
        Arc->center.x = a;
        Arc->center.y = b;
        Arc->r = r;
        return 1;
    } else {
        return 0;
    }
    return 1;
}

float getCoefficient(int X[], int Y[], const int Cnt, double a, double b) {
    double Y_line[Cnt];
    int Y_total = 0;
    double sse = 0.0;
    double sst = 0.0;
    for (int i = 0; i < Cnt; ++i) {
        Y_total += Y[i];
        Y_line[i] = a * X[i] + b;
        sse += (Y[i] - Y_line[i]) * (Y[i] - Y_line[i]);
    }
    double Y_mean = (double) Y_total / (double) (Cnt);
    for (int i = 0; i < Cnt; ++i) {
        sst += (Y[i] - Y_mean) * (Y[i] - Y_mean);
    }
//    cout << "sse————————" << sse << endl;
//    cout << "sst————————" << sst << endl;
//	printf("the a is:%f\n",a);

    double RMSE = sqrt(sse /  Cnt);
//    cout << "RMSE————————" << RMSE << endl;

    if(1) {//输出相关系数，取值范围[0,1],越接近１表示拟合效果越好
        if (fabs(a) < 0.05 && sst != 0.0f) {
//        return (1 - sse / Cnt * 0.25);
            return (1 - sse / Cnt * 0.005);
        }
        if (sst != 0.0f) {
            return (1 - sse / sst);
        }
    }else{
        return RMSE;//返回平均拟合偏差，单位mm
    }


    return 0.0f;

}

void getFitCoefficient(int X[], int Y[], const int Cnt, LinePara *EstLinePara) {
    double Y_line[Cnt];
    int Y_total = 0;
    double sse = 0.0;
    double sst = 0.0;
    for (int i = 0; i < Cnt; ++i) {
        Y_total += Y[i];
        Y_line[i] = EstLinePara->a * X[i] + EstLinePara->b;
        sse += (Y[i] - Y_line[i]) * (Y[i] - Y_line[i]);
    }
    double Y_mean = (double) Y_total / (double) (Cnt);
    for (int i = 0; i < Cnt; ++i) {
        sst += (Y[i] - Y_mean) * (Y[i] - Y_mean);
    }
//    cout << "sse————————" << sse << endl;
//    cout << "sst————————" << sst << endl;
//	printf("the a is:%f\n",a);

    EstLinePara->RMSE = sqrt(sse /  Cnt);
//    cout << "RMSE————————  " << EstLinePara->RMSE << endl;
    //输出相关系数，取值范围[0,1],越接近１表示拟合效果越好
//    if (fabs(EstLinePara->a) < 0.05 && sst != 0.0f) {
//       return (1 - sse / Cnt * 0.25);
//        EstLinePara->R = (1 - sse / Cnt * 0.005);
//        EstLinePara->R = 100;
//    }else{
        EstLinePara->R = (1 - sse / sst);
//    }
//    cout << "a———————————— " << EstLinePara->a << endl;
//    cout << "R———————————— " << EstLinePara->R << endl;
}
