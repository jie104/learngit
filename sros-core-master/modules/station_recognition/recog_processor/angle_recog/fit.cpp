//~ #include "OpenRadar.h"
#include<cstring>
#include "fit.h"
#include "WeightedFit.h"

//~ const int DisplayDx = RadarImageWdith/2;
//~ const int DisplayDy = RadarImageHeight*3/4;
const int DisplayRatio = 10; //
int X[100] = {0};
int Y[100] = {0};
long long Err[101][101] = {0};

OpenRadar::OpenRadar(void) {
    RadarState = 0x02;
    corner_line.clear();
    high_thresh = 40;//40mm
    linear_rthresh = 0.01;//表示相关系数
    pointcnt_thresh = 9;//端点被当做断点滤掉了,因此,成为了9个点
    linear_RMSEthresh = 1000;//表示平均拟合偏差RRS,单位mm
}

OpenRadar::~OpenRadar(void) {
}

void OpenRadar::ConvertRho2XY(int *rho, float *sinTheta, float *cosTheta,
                              int Cnt, int *X, int *Y) {
    for (int i = 0; i < Cnt; i++) {
        if (rho[i] < 0) {
            X[i] = -1000000;
            Y[i] = -1000000;
        } else {
            X[i] = floor(rho[i] * cosTheta[i] + 0.5);
            Y[i] = floor(rho[i] * sinTheta[i] + 0.5);
        }
    }
}

int OpenRadar::BreakRadar(int *rho, float *theta, int Cnt) {
    int breakCnt = 0;
    int dis = 0;
    int Dmax = 50;
    RadarCorner.clear();
    iPoint point;
    float theta_thresh=sin(0.00436)/sin(0.170);//临界值,用于识别断点
    for (int i = 1; i < Cnt; i++) {
        dis = abs(rho[i] - rho[i - 1]);
        int thresh=2*rho[i - 1]*theta_thresh+15;
        if (dis > thresh) {
            point.x = rho[i] * cos(theta[i]);
            point.y = rho[i] * sin(theta[i]);
            RadarCorner.push_back(point);
            RadarCorner.push_back(ipoint(rho[i-1] * cos(theta[i-1]),rho[i-1] * sin(theta[i-1])));
            rho[i] = -1;
            rho[i - 1] = -1;
            breakCnt++;
            breakCnt++;
            i++; //
        }
    }
    point.x = rho[0] * cos(theta[0]);
    point.y = rho[0] * sin(theta[0]);//将第一个雷达点识别为断点
    RadarCorner.push_back(point);
    rho[0] = -1;
    point.x = rho[Cnt - 1] * cos(theta[Cnt - 1]);//将最后一个雷达点识别为断点
    point.y = rho[Cnt - 1] * sin(theta[Cnt - 1]);
    RadarCorner.push_back(point);
    rho[Cnt - 1] = -1;
    breakCnt += 2;
    return breakCnt;
}

//找到角点,并筛选直线,用角点与断点分割直线
int OpenRadar::PolyContourFit(int *X, int *Y, int n, float Eps) {
    float dis = sqrt((float) (((X[0] - X[n - 1]) * (X[0] - X[n - 1]))
                              + ((Y[0] - Y[n - 1]) * (Y[0] - Y[n - 1]))));
    float cosTheta = (float) (X[n - 1] - X[0]) / dis;
    float sinTheta = -(float) (Y[n - 1] - Y[0]) / dis;
    float MaxDis = 0;
    int i;
    int MaxDisInd = -1;
    float dbDis;
    for (i = 1; i < n - 1; i++) {
        dbDis = abs((Y[i] - Y[0]) * cosTheta + (X[i] - X[0]) * sinTheta);
        if (dbDis > MaxDis) {
            MaxDis = dbDis;
            MaxDisInd = i;
        }
    }
    if (MaxDis > Eps) {
//		printf("the max is:%f\n",MaxDis);
        return MaxDisInd;
    }//一旦大于临界值,则认为属于角点,返回角点信息.
    return 0;
}

//识别角点,并将每一段连续的点中所有的角点识别出来
int OpenRadar::FindCorners(vector<int> &CornerIndex, int *X, int *Y, int start,
                           int Cnt, float Eps) {

    int N = 0;
    int N1 = 0, N2 = 0;
    N = PolyContourFit(X, Y, Cnt, Eps);
    if (N == 0) {
        return 0;
    } else if (N > 0 && N < Cnt) {
        CornerIndex.push_back(start + N);
        if (N > 20) {
            N1 = FindCorners(CornerIndex, X, Y, start, N, Eps);
        }

        if (Cnt - N > 20) {
            N2 = FindCorners(CornerIndex, X + N, Y + N, start + N, Cnt - N,
                             Eps);
        }
    }
    int temp;
    for (int i = 0; i < static_cast<int>(CornerIndex.size()); i++) {
        for (int j = i + 1; j < static_cast<int>(CornerIndex.size()); j++) {
            if (CornerIndex.at(i) > CornerIndex.at(j)) {
                temp = CornerIndex.at(i);
                CornerIndex.at(i) = CornerIndex.at(j);
                CornerIndex.at(j) = temp;
            }
        }
    }
//	iPoint CornerPoint;
//	for (int i = 0; i < static_cast<int>(CornerIndex.size()); i++) {
//		CornerPoint.x = X[CornerIndex.at(i)];
//		CornerPoint.y = Y[CornerIndex.at(i)];
//		RadarCorner.push_back(CornerPoint);
//	}
    return CornerIndex.size();
}

//将所有的角点识别出来
int OpenRadar::BreakPolyLine(int *X, int *Y, int Cnt, vector<iPoint> &Corners) {

    int pointCnt = 0;
    vector<int> CornerIndex;
    int CornerCnt = 0;
    int lastSepIndex = 0;
    int CurrentSepIndex = 0;
    Corners.clear();
    corner_point.clear();
    int cornercnt_thresh = 2 * pointcnt_thresh;

    for (int i = 0; i < Cnt; i++) {
        if (X[i] <= -1000000) {
            lastSepIndex = CurrentSepIndex;
            CurrentSepIndex = i;
            pointCnt = CurrentSepIndex - lastSepIndex - 1;

            if (pointCnt > cornercnt_thresh)    //一旦超过20个点,就用于识别角点,否则不进行处理
            {
                FindCorners(CornerIndex, X + lastSepIndex + 1,
                            Y + lastSepIndex + 1, 0, pointCnt, high_thresh);
                for (int j = 0; j < CornerIndex.size(); j++) {
                    int corner_size = Corners.size();
                    Corners.push_back(
                            ipoint(X[i - pointCnt + CornerIndex.at(j)],
                                   Y[i - pointCnt + CornerIndex.at(j)]));
                    if (corner_point.find(i - pointCnt + CornerIndex.at(j))
                        == corner_point.end()) {
                        corner_point[i - pointCnt + CornerIndex.at(j)] = ipoint(
                                X[i - pointCnt + CornerIndex.at(j)],
                                Y[i - pointCnt + CornerIndex.at(j)]);
                    }
                    X[i - pointCnt + CornerIndex.at(j)] = -2000000;
                    Y[i - pointCnt + CornerIndex.at(j)] = -2000000;
                }
                CornerCnt += CornerIndex.size();
                CornerIndex.clear();
            }
        }

    }
//    CornerIndex.clear();
    return CornerCnt;
}

//匹配直线,并将直线中,具有角特征的直线识别出来,并对该角进行预处理,用两条直线交点来表示角的顶点
int OpenRadar::FitLine(vector<LinePara> &FittedLine, int *X, int *Y, int Cnt) {
    int pointCnt = 0;
    LinePara tmpLinePara;
    int lastSepIndex = 0;
    int CurrentSepIndex = 0;
    FittedLine.clear();
    line_point.clear();
    corner_line.clear();
    int length = 0;
    for (int i = 0; i < Cnt; i++) {
        if (X[i] <= -1000000) {
            lastSepIndex = CurrentSepIndex;
            CurrentSepIndex = i;
            pointCnt = CurrentSepIndex - lastSepIndex - 1;
            int delta_x = X[lastSepIndex + 1] - X[CurrentSepIndex - 1];
            int delta_y = Y[lastSepIndex + 1] - Y[CurrentSepIndex - 1];
            length = sqrt(delta_x * delta_x + delta_y * delta_y);
            if (pointCnt >= pointcnt_thresh && length > 60) {//一旦超过10个点,在处理过程中,有个断点被滤掉了且距离大于6cm,就进行直线匹配
                vector<iPoint> tmp_line;
                for (i = lastSepIndex + 1; i < CurrentSepIndex; i++) {
                    iPoint ipoint;
                    ipoint.x = X[i];
                    ipoint.y = Y[i];
                    tmp_line.push_back(ipoint);
                }
                line_point.push_back(tmp_line);
                WeightedFit(X + lastSepIndex + 1, Y + lastSepIndex + 1, pointCnt, &tmpLinePara);
                if(1){
                      if ((tmpLinePara.R >= linear_rthresh && fabs(tmpLinePara.a) >= 0.05) || ((fabs(tmpLinePara.a) < 0.05) && tmpLinePara.RMSE < linear_RMSEthresh) ){//对每一段线进行线性相关度判定,如果大于0.8,则认为是直线
//                    if (tmpLinePara.R <= linear_rthresh) {//对每一段线的拟合平均偏差进行判定,如果小于high_thresh,则认为是直线
                        FittedLine.push_back(tmpLinePara);
                        if (X[lastSepIndex] <= -2000000) {
                            decodeCorner(X, Y, tmpLinePara, i - 1, lastSepIndex);
                        }
                        if (X[CurrentSepIndex] <= -2000000) {
                            decodeCorner(X, Y, tmpLinePara, lastSepIndex + 1, CurrentSepIndex);
                        }
                    }
                }
                if(0){
                    FittedLine.push_back(tmpLinePara);
                }
            }
        }
    }
    return 0;
}

void OpenRadar::myStrcpy(string s, char *array) {
    int length = s.length();
    for (int i = 0; i < length; i++) {
        array[i] = s.at(i);
    }
    array[length] = '\0';
}

//获得角特征,对角进行预处理,用两条直线交点表示角的顶点
bool OpenRadar::getCornerLine(int index, LinePara &tmp_line) {
    map<int, vector<LinePara> >::iterator find_corner = corner_line.find(index);
    if (find_corner != corner_line.end()) {
        find_corner->second.push_back(tmp_line);
        if (find_corner->second.size() == 2) {
            LinePara &line1 = find_corner->second[0];
            LinePara &line2 = find_corner->second[1];
            if (line1.a != line2.a) {
                line1.startPoint.x = (line2.b - line1.b) / (line1.a - line2.a);
                line2.startPoint.x = (line2.b - line1.b) / (line1.a - line2.a);
                line1.startPoint.y = (line2.b * line1.a - line1.b * line2.a) / (line1.a - line2.a);
                line2.startPoint.y = (line2.b * line1.a - line1.b * line2.a) / (line1.a - line2.a);
            }
//			printf("the point is:%d,%d\n",find_corner->second[0].startPoint.x,find_corner->second[0].startPoint.y);
        }
    } else {
        vector<LinePara> tmp_line_vector;
        tmp_line_vector.push_back(tmp_line);
        corner_line.insert(pair<int, vector<LinePara> >(index, tmp_line_vector));
    }
    return true;
}

//求两条直线的交点,并标记为顶点
bool OpenRadar::decodeCorner(int *X, int *Y, LinePara tmpline, int endindex, int startindex) {
    map<int, iPoint>::iterator cor_it = corner_point.find(
            startindex);
    if (cor_it != corner_point.end()) {
        //tmpline.b=(float)cor_it->second.y-(float)cor_it->second.x*tmpline.a;
        //如果注释去掉,那么用角点来代替直线交点
        tmpline.startPoint = cor_it->second;
        if (abs(tmpline.a) >= 1) {
            tmpline.endPoint.y = Y[endindex];
            tmpline.endPoint.x = (Y[endindex] - tmpline.b) / tmpline.a;
        } else {
            tmpline.endPoint.x = X[endindex];
            tmpline.endPoint.y = tmpline.a * X[endindex] + tmpline.b;
        }
        getCornerLine(startindex, tmpline);
        return true;
    }
    return false;
}

//求雷达点形成的直线
bool OpenRadar::getLineVector(int *X, int *Y, int Cnt) {
    int pointCnt = 0;
    LinePara tmpLinePara;
    int lastSepIndex = 0;
    int CurrentSepIndex = 0;
    FittedLine.clear();
    line_point.clear();
    corner_line.clear();
    int length = 0;
    for (int i = 0; i < Cnt; i++) {
        if (X[i] <= -1000000) {
//            cout << "断点索引———————— " << i << endl;
            lastSepIndex = CurrentSepIndex;
            CurrentSepIndex = i;
            pointCnt = CurrentSepIndex - lastSepIndex - 1;
//            cout << "点段个数———————— " << pointCnt << endl;
            int delta_x=X[lastSepIndex + 1] - X[CurrentSepIndex - 1];
            int delta_y=Y[lastSepIndex + 1] - Y[CurrentSepIndex - 1];
            length = sqrt(delta_x*delta_x+delta_y*delta_y);
//            cout << "点段长度———————— " << length << endl;
            if (pointCnt >= pointcnt_thresh && length > 60){//一旦超过10个点,在处理过程中,有个断点被滤掉了且距离大于6cm,就进行直线匹配
                WeightedFit(X + lastSepIndex + 1, Y + lastSepIndex + 1,	pointCnt, &tmpLinePara);
//                cout << "点段a———————— " << tmpLinePara.a << endl;
//                cout << "点段b———————— " << tmpLinePara.b << endl;
//                cout << "点段t———————— " << tmpLinePara.theta << endl;
//                cout << "点段R———————— " << tmpLinePara.R << endl;

//                cout<< "____________" << endl;
                bool is_merge_line=false;
//                if (tmpLinePara.R >= linear_rthresh) {//对每一段线进行线性相关度判定,如果大于0.8,则认为是直线
//                if (tmpLinePara.R <= linear_rthresh) {//对每一段线的拟合平均偏差进行判定,如果小于high_thresh,则认为是直线
                if ((tmpLinePara.R >= linear_rthresh && fabs(tmpLinePara.a) >= 0.05) || ((fabs(tmpLinePara.a) < 0.05) && tmpLinePara.RMSE < linear_RMSEthresh)){//对每一段线进行线性相关度判定,如果大于0.8,则认为是直线
                    int line_size=FittedLine.size();
//                    cout << "line_size" << line_size << endl;
                    for (int j = 0; j < line_size; ++j) {
                        if(fabs(fabs(tmpLinePara.theta)-fabs(FittedLine[j].theta))<0.1f){
                            if(fabs(tmpLinePara.b-FittedLine[j].b)<40){
                                int new_size = CurrentSepIndex - lastSepIndex + line_point[j].size();
                                int* RadarX=new int[new_size];
                                int* RadarY=new int[new_size];
                                int count=0;
                                for (i = lastSepIndex + 1; i < CurrentSepIndex; i++) {
                                    RadarX[count] = X[i];
                                    RadarY[count] = Y[i];
                                    count++;
                                }
                                int linepoint_size=line_point[j].size();
                                for (int k = 0; k < linepoint_size; ++k) {
                                    iPoint& ipoint=line_point[j][k];
                                    RadarX[count] = ipoint.x;
                                    RadarY[count] = ipoint.y;
                                    count++;
                                }
                                WeightedFit(RadarX, RadarY,	count, &tmpLinePara);
                                delete [] RadarX;
                                delete [] RadarY;
//                                if (tmpLinePara.R >= linear_rthresh){
                                if ((tmpLinePara.R >= linear_rthresh && fabs(tmpLinePara.a) >= 0.05) || ((fabs(tmpLinePara.a) < 0.05) && tmpLinePara.RMSE < linear_RMSEthresh) ){//对每一段线进行线性相关度判定,如果大于0.8,则认为是直线
                                    tmpLinePara.length=length + FittedLine[j].length;
                                    FittedLine[j]=tmpLinePara;
                                    for (i = lastSepIndex + 1; i < CurrentSepIndex; i++) {
                                        iPoint ipoint;
                                        ipoint.x = X[i];
                                        ipoint.y = Y[i];
                                        line_point[j].push_back(ipoint);
                                    }
                                    is_merge_line = true;
                                    break;
                                  }
                            }
                        }
                    }
                    if(is_merge_line==false){
                        vector<iPoint> tmp_line;
                        for (i = lastSepIndex + 1; i < CurrentSepIndex; i++) {
                            iPoint ipoint;
                            ipoint.x = X[i];
                            ipoint.y = Y[i];
                            tmp_line.push_back(ipoint);
                        }
                        line_point.push_back(tmp_line);
                        tmpLinePara.length = length;
                        FittedLine.push_back(tmpLinePara);
                    }
                }
            }
        }
    }
    if(FittedLine.size()){
        return true;
    }

    return false;
}