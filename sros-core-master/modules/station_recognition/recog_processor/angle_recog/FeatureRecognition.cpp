/*
 * FeatureRecognition.cpp
 *
 *  Created on: 2016年6月2日
 *      Author: hq
 */

#include "FeatureRecognition.h"
#include "WeightedFit.h"

#include <math.h>

namespace slam {

FeatureRecognition::FeatureRecognition() :
        laser_max_dist(30.0f), laser_min_dist(0.1f), RadarDataCnt(0), RadarX(NULL), RadarSinTheta(NULL),
        RadarCosTheta(NULL), RadarRho(NULL), RadarTheta(NULL),is_initialized(false) {
//    RadarX = new int[MAX_ARRAY_COUNT];
//    RadarY = new int[MAX_ARRAY_COUNT];
//    RadarSinTheta = new float[MAX_ARRAY_COUNT];
//    RadarCosTheta = new float[MAX_ARRAY_COUNT];
//    RadarRho = new int[MAX_ARRAY_COUNT];
//    RadarTheta = new float[MAX_ARRAY_COUNT];
//
//    memset(RadarX, 0, MAX_ARRAY_COUNT * sizeof(int));
//    memset(RadarY, 0, MAX_ARRAY_COUNT * sizeof(int));
//    memset(RadarSinTheta, 0, MAX_ARRAY_COUNT * sizeof(float));
//    memset(RadarCosTheta, 0, MAX_ARRAY_COUNT * sizeof(float));
//    memset(RadarRho, 0, MAX_ARRAY_COUNT * sizeof(int));
//    memset(RadarTheta, 0, MAX_ARRAY_COUNT * sizeof(float));

    input_anglefeature.angle = 1.8959;
    input_anglefeature.line1_length = 240; //mm
    input_anglefeature.line2_length = 240; //mm
    input_anglefeature.is_reflexangle_side = false;

#ifdef ROS_ENV
    scan_sub = node_.subscribe("/scan", 5, &FeatureRecognition::scanCallback,this);
    breakpoint_pub = node_.advertise<sensor_msgs::PointCloud>("/breakpoint", 5); //用于发布断点的数据("/breakpoint", 5);
    longline_pub = node_.advertise<sensor_msgs::PointCloud>("/longline", 5); //用于发布断点的数据("/breakpoint", 5);
#endif
}

FeatureRecognition::~FeatureRecognition() {
    if (RadarX)
        delete[] RadarX;
    if (RadarY)
        delete[] RadarY;
    if (RadarSinTheta)
        delete[] RadarSinTheta;
    if (RadarCosTheta)
        delete[] RadarCosTheta;
    if (RadarRho)
        delete[] RadarRho;
    if (RadarTheta)
        delete[] RadarTheta;

    // TODO 释放数组
}

#ifdef ROS_ENV

void FeatureRecognition::scanCallback(
        const sensor_msgs::LaserScan &scan_const) {
    decodeLidar(scan_const);

    openRadar.BreakRadar(RadarRho, RadarTheta, RadarDataCnt);//识别断点
    int corner_size = openRadar.RadarCorner.size();
    laser_cloud.points.clear();
    for (int i = 0; i < corner_size; i++) {
        iPoint point = openRadar.RadarCorner[i];
        geometry_msgs::Point32 geo_point;
        geo_point.x = (float) (point.x) / 1000.0;
        geo_point.y = (float) (point.y) / 1000.0;
        geo_point.z = 0.0;
        laser_cloud.points.push_back(geo_point);
    }
    breakpoint_pub.publish(laser_cloud);
    laser_cloud.points.clear();
    for (int i = 0; i < MAX_ARRAY_COUNT; i++) {
        RadarSinTheta[i] = sin(RadarTheta[i]);
        RadarCosTheta[i] = cos(RadarTheta[i]);
    }
    openRadar.ConvertRho2XY(RadarRho, RadarSinTheta, RadarCosTheta,
                            RadarDataCnt, RadarX, RadarY); //转换到直角坐标系
    openRadar.BreakPolyLine(RadarX, RadarY, RadarDataCnt,
                            openRadar.RadarCorner);
    //识别直线
    Eigen::Vector3f pose(0,0,0);
    laser_cloud.header=scan_const.header;
    if(openRadar.getLineVector(RadarX, RadarY, RadarDataCnt)){
        int line_index;
        LinePara long_line=findLongestLine(openRadar.FittedLine,line_index);
        printf("the theta is:%f\n",long_line.theta);
        if(fabs(long_line.theta)<PI/4)
            pose[2] = -long_line.theta;
        else{
            if(long_line.a == 0.0f)
                pose[2] = -0.0f;
            else{
                pose[2] = -atan(-1.0f/long_line.a);
            }
        }
        laser_cloud.points.clear();
        geometry_msgs::Point32 geo_point;
        for (int i = 0; i <RadarDataCnt ; ++i) {
            geo_point.x=(RadarX[i]*cos(pose[2])-RadarY[i]*sin(pose[2])+pose[0])/1000.0;
            geo_point.y=(RadarX[i]*sin(pose[2])+RadarY[i]*cos(pose[2])+pose[1])/1000.0;
            laser_cloud.points.push_back(geo_point);
        }
        breakpoint_pub.publish(laser_cloud);
        laser_cloud.points.clear();
        vector<iPoint>& longline_point=openRadar.line_point[line_index];
        int longlinesize=longline_point.size();
        for (int i = 0; i <longlinesize ; ++i) {
            geo_point.x=(longline_point[i].x*cos(pose[2])-longline_point[i].y*sin(pose[2])+pose[0])/1000.0;
            geo_point.y=(longline_point[i].x*sin(pose[2])+longline_point[i].y*cos(pose[2])+pose[1])/1000.0;
            laser_cloud.points.push_back(geo_point);
        }
        longline_pub.publish(laser_cloud);

    }

//    if (matchAngle(RadarRho, RadarTheta, RadarDataCnt, input_anglefeature, pose_vector)) {
//        printf("will publish the pose\n");
//        geometry_msgs::PoseArray tmp_pose_arr;
//        tmp_pose_arr.header.frame_id = "scan";
//        tmp_pose_arr.header.stamp = ros::Time::now();
//        int pose_size = pose_vector.size();
//        for (int i = 0; i < pose_size; i++) {
//            geometry_msgs::Pose tmp_pose;
//            Eigen::Vector3f tmppos = pose_vector[i];
//            tmp_pose.position.x = tmppos[0];
//            tmp_pose.position.y = tmppos[1];
//            tmp_pose.orientation.w = cos(tmppos[2] * 0.5);
//            tmp_pose.orientation.z = sin(tmppos[2] * 0.5);
//            tmp_pose_arr.poses_.push_back(tmp_pose);
//        }
//        pose_pub.publish(tmp_pose_arr);
//    }

}

void FeatureRecognition::decodeLidar(const sensor_msgs::LaserScan &scan) {
    memset(RadarRho, 0, MAX_ARRAY_COUNT * sizeof(int));
    memset(RadarTheta, 0, MAX_ARRAY_COUNT * sizeof(float));
    int size = scan.ranges.size();
    printf("the size is:%d\n", size);
    float angle = scan.angle_min;
    float increment = scan.angle_increment;
    laser_cloud.header = scan.header;
    float maxrange = scan.range_max - 0.1f;
    float minrange = scan.range_min;
    int count = 0;
    float dist_min = minrange > laser_min_dist ? minrange : laser_min_dist;
    float dist_max = maxrange < laser_max_dist ? maxrange : laser_max_dist;

    for (int i = 0; i < size; ++i) {
        float dist = scan.ranges[i];

        if ((dist > dist_min) && (dist < dist_max)) {
            RadarRho[count] = floor((dist) * 1000.0 + 0.5);
            RadarTheta[count] = angle;
            count++;
        }
        angle += increment;
    }
    RadarDataCnt = count;
}

#else


bool FeatureRecognition::getInitialPose(const sros::core::LaserScanMsg &scan, Eigen::Vector3f &initialpose) {
    decodeLidar(scan);
    openRadar.BreakRadar(RadarRho, RadarTheta, RadarDataCnt);//识别断点
    int corner_size = openRadar.RadarCorner.size();

    for (int i = 0; i < scan_size; i++) {
        RadarSinTheta[i] = sin(RadarTheta[i]);
        RadarCosTheta[i] = cos(RadarTheta[i]);
    }
    openRadar.ConvertRho2XY(RadarRho, RadarSinTheta, RadarCosTheta,
                            RadarDataCnt, RadarX, RadarY); //转换到直角坐标系
    openRadar.BreakPolyLine(RadarX, RadarY, RadarDataCnt,
                            openRadar.RadarCorner);
    //识别直线
    Eigen::Vector3f pose(0, 0, 0);
    if (openRadar.getLineVector(RadarX, RadarY, RadarDataCnt)) {
        int line_index;
        LinePara long_line = findLongestLine(openRadar.FittedLine, line_index);
        if (long_line.length > 1000.0) {
            printf("the theta is:%f\n", long_line.theta);
            if (fabs(long_line.theta) < PI / 4)
                pose[2] = -long_line.theta;
            else {
                if (long_line.a == 0.0f)
                    pose[2] = -0.0f;
                else {
                    pose[2] = -atan(-1.0f / long_line.a);
                }
            }
            initialpose = pose;
            return true;
        }
    }
    return false;
}

void FeatureRecognition::decodeLidar(const sros::core::LaserScanMsg &scan) {
    if(!is_initialized) {
        initialize(scan);
    }
    int size = scan.ranges.size();
    printf("the size is:%d\n", size);
    float angle = scan.angle_min;
    float increment = scan.angle_increment;
    float maxrange = scan.range_max - 0.1f;
    float minrange = scan.range_min;
    int count = 0;
    float dist_min = minrange > laser_min_dist ? minrange : laser_min_dist;
    float dist_max = maxrange < laser_max_dist ? maxrange : laser_max_dist;

    for (int i = 0; i < size; ++i) {
        float dist = scan.ranges[i];

        if ((dist > dist_min) && (dist < dist_max)) {
            RadarRho[count] = floor((dist) * 1000.0 + 0.5);
            RadarTheta[count] = angle;
            count++;
        }
        angle += increment;
    }
    RadarDataCnt = count;
}

#endif

bool FeatureRecognition::getAngleFeature(
        vector<AngleFeature> &angle_feature_vector) {
    map<int, vector<LinePara> >::iterator it;
    for (it = openRadar.corner_line.begin(); it != openRadar.corner_line.end();
         it++) {
        if (it->second.size() == 2) {
            AngleFeature tmp_anfe;
            LinePara &tmp_line1 = it->second[0];
            LinePara &tmp_line2 = it->second[1];

            tmp_anfe.corner = tmp_line1.startPoint;

            float dist1 = getPointDist(tmp_line1.startPoint, tmp_line1.endPoint);
            float dist2 = getPointDist(tmp_line2.startPoint, tmp_line2.endPoint);
            float dist3 = getPointDist(tmp_line1.endPoint, tmp_line2.endPoint);

            if (dist1 != 0 && dist2 != 0) {
                tmp_anfe.angle = acos((dist1 * dist1 + dist2 * dist2 - dist3 * dist3) / (2 * dist1 * dist2));
            } else {
                tmp_anfe.angle = 0.0;
            }

            tmp_anfe.line1 = tmp_line1;
            tmp_anfe.line2 = tmp_line2;
            tmp_anfe.line1_length = dist1;
            tmp_anfe.line2_length = dist2;

            iPoint vec_ab, vec_ac, vec_ad;
            vec_ab.x = tmp_line1.endPoint.x - tmp_line1.startPoint.x;
            vec_ab.y = tmp_line1.endPoint.y - tmp_line1.startPoint.y;

            vec_ac.x = tmp_line2.endPoint.x - tmp_line2.startPoint.x;
            vec_ac.y = tmp_line2.endPoint.y - tmp_line2.startPoint.y;

            float length_ab = sqrt(vec_ab.x * vec_ab.x + vec_ab.y * vec_ab.y);
            float length_ac = sqrt(vec_ac.x * vec_ac.x + vec_ac.y * vec_ac.y);

            float vec_ad_x, vec_ad_y;
            if (length_ab != 0 && length_ac != 0) {
                vec_ad_x = (float) vec_ab.x / length_ab
                           + (float) vec_ac.x / length_ac;
                vec_ad_y = (float) vec_ab.y / length_ab
                           + (float) vec_ac.y / length_ac;
                if (vec_ad_x != 0) {
                    tmp_anfe.bisector.a = (float) vec_ad_y / (float) vec_ad_x;
                    tmp_anfe.bisector.b = ((float) tmp_anfe.corner.y * vec_ad_x
                                           - (float) tmp_anfe.corner.x * vec_ad_y)
                                          / (float) vec_ad_x;
                } else {
                    tmp_anfe.bisector.a = 1000;
                    tmp_anfe.bisector.b = -tmp_anfe.bisector.a
                                          * (float) tmp_anfe.corner.x;
                }
                if (abs(tmp_anfe.bisector.a) > 1000) {
                    tmp_anfe.bisector.a = 1000 * (tmp_anfe.bisector.a / abs(tmp_anfe.bisector.a));
                    tmp_anfe.bisector.b = -1 * tmp_anfe.bisector.a * (float) tmp_anfe.corner.x;
                }
                tmp_anfe.bisector.startPoint = tmp_anfe.corner;
                tmp_anfe.bisector.endPoint.x = tmp_anfe.corner.x + 1000;
                tmp_anfe.bisector.endPoint.y = tmp_anfe.bisector.endPoint.x * tmp_anfe.bisector.a
                                               + tmp_anfe.bisector.b;
            } else {
                if (length_ab == 0) {
                    tmp_anfe.bisector = tmp_line2;
                } else {
                    tmp_anfe.bisector = tmp_line1;
                }
            }
            angle_feature_vector.push_back(tmp_anfe);
        }
    }
    return true;
}

bool FeatureRecognition::checkInputAngel(
        vector<AngleFeature> &angle_feature_vector,
        vector<AngleFeature> &output_angfeature) {

    int ang_fea_size = angle_feature_vector.size();
    int count = 0;
    int index = 0;

    if (input_anglefeature.line1_length == 0.0f
        || input_anglefeature.line2_length == 0.0f) {
        printf("the length is zero! will return\n");
        return false;
    }

    for (int i = 0; i < ang_fea_size; i++) {
        AngleFeature &tmp_angfeature = angle_feature_vector[i];
        if (fabs(tmp_angfeature.angle - input_anglefeature.angle) < 0.107f) {
            float delta_dis1 = tmp_angfeature.line1_length
                               / input_anglefeature.line1_length;
            if (delta_dis1 > 1.0) {
                delta_dis1 = 1.0f / delta_dis1;
            }
            float delta_dis2 = tmp_angfeature.line1_length
                               / input_anglefeature.line2_length;
            if (delta_dis2 > 1.0) {
                delta_dis2 = 1.0f / delta_dis2;
            }
            float delta_dis3 = tmp_angfeature.line2_length
                               / input_anglefeature.line1_length;
            if (delta_dis3 > 1.0) {
                delta_dis3 = 1.0f / delta_dis3;
            }
            float delta_dis4 = tmp_angfeature.line2_length
                               / input_anglefeature.line2_length;
            if (delta_dis4 > 1.0) {
                delta_dis4 = 1.0f / delta_dis4;
            }
            if (delta_dis1 > 0.8) {
                if (delta_dis4 > 0.8) {
                    count++;
                    output_angfeature.push_back(tmp_angfeature);
                }
            } else if (delta_dis2 > 0.8) {
                if (delta_dis3 > 0.8) {
                    count++;
                    output_angfeature.push_back(tmp_angfeature);
                }
            }
        }
    }
    if (count >= 1) {
        for (int i = 0; i < count; i++) {
            output_angfeature[i].bisector.theta = atan(output_angfeature[i].bisector.a);
            int tmpx = output_angfeature[i].line1.endPoint.x - output_angfeature[i].line1.startPoint.x;
            int tmpy = output_angfeature[i].line1.endPoint.y - output_angfeature[i].line1.startPoint.y;
            iPoint vec_line1 = ipoint(tmpx, tmpy);

            tmpx = output_angfeature[i].line2.endPoint.x - output_angfeature[i].line2.startPoint.x;
            tmpy = output_angfeature[i].line2.endPoint.y - output_angfeature[i].line2.startPoint.y;
            iPoint vec_line2 = ipoint(tmpx, tmpy);

            tmpx = output_angfeature[i].bisector.endPoint.x - output_angfeature[i].bisector.startPoint.x;
            tmpy = output_angfeature[i].bisector.endPoint.y - output_angfeature[i].bisector.startPoint.y;
            iPoint vec_bis = ipoint(tmpx, tmpy);

            if ((vec_bis.x * vec_line1.x + vec_bis.y * vec_line1.y) > 0 &&
                (vec_bis.x * vec_line2.x + vec_bis.y * vec_line2.y) > 0) {
                if (!input_anglefeature.is_reflexangle_side) {
                    output_angfeature[i].bisector.theta += M_PI;
                }
            } else {
                output_angfeature[i].bisector.endPoint.x = output_angfeature[i].corner.x - 1000;
                output_angfeature[i].bisector.endPoint.y = output_angfeature[i].bisector.endPoint.x
                                                           * output_angfeature[i].bisector.a +
                                                           output_angfeature[i].bisector.b;
                if (input_anglefeature.is_reflexangle_side) {
                    output_angfeature[i].bisector.theta += M_PI;
                }
            }
        }

        return true;
    } else {
        printf("the count is zero!\n");
    }
    return false;
}

bool FeatureRecognition::matchAngle(int *radarrho, float *theta, int ridardatacnt,
                                    AngleFeature &inputangle, vector<Eigen::Vector3f> &pose_vec) {
    openRadar.BreakRadar(radarrho, theta, ridardatacnt); //识别断点
    for (int i = 0; i < scan_size; i++) {
        RadarSinTheta[i] = sin(theta[i]);
        RadarCosTheta[i] = cos(theta[i]);
    }
    openRadar.ConvertRho2XY(radarrho, RadarSinTheta, RadarCosTheta,
                            ridardatacnt, RadarX, RadarY); //转换到直角坐标系
    openRadar.BreakPolyLine(RadarX, RadarY, RadarDataCnt,
                            openRadar.RadarCorner);
    //识别直线
    openRadar.FitLine(openRadar.FittedLine, RadarX, RadarY, RadarDataCnt);
    vector<AngleFeature> angle_feature_vector;
    angle_feature_vector.clear();
    int line_size = openRadar.RadarCorner.size();
    if (line_size > 0) {
        getAngleFeature(angle_feature_vector); //将直线与角点信息转化成角信息
        vector<AngleFeature> output_angle_feature;
        input_anglefeature = inputangle;
        if (checkInputAngel(angle_feature_vector, output_angle_feature)) {
            int angle_size = output_angle_feature.size();
            for (int i = 0; i < angle_size; i++) {
                AngleFeature &tmp_angle = output_angle_feature[i];
                Eigen::Vector3f tmp_pose;
                tmp_pose[0] = tmp_angle.corner.x / 1000.0;
                tmp_pose[1] = tmp_angle.corner.y / 1000.0;
                tmp_pose[2] = tmp_angle.bisector.theta;
                pose_vec.push_back(tmp_pose);
            }
            return true;
        }
    }
    return false;
}

#ifdef ROS_ENV

bool FeatureRecognition::matchAngle(const sensor_msgs::LaserScan &scan_const,
                                    AngleFeature &inputangle, Eigen::Vector3f &laser_pose,
                                    vector<Eigen::Vector3f> &pose_vec) {
    decodeLidar(scan_const);
    vector<Eigen::Vector3f> pose_vector;
    if (matchAngle(RadarRho, RadarTheta, RadarDataCnt, inputangle, pose_vector)) {
        int pose_size = pose_vector.size();
        for (int i = 0; i < pose_size; i++) {
            Eigen::Vector3f &tmp_pose = pose_vector[i];
            Eigen::Vector3f out_pose;
            out_pose[0] = tmp_pose[0] * cos(laser_pose[2]) - tmp_pose[1] * sin(laser_pose[2]) + laser_pose[0];
            out_pose[1] = tmp_pose[0] * sin(laser_pose[2]) + tmp_pose[1] * cos(laser_pose[2]) + laser_pose[1];
            out_pose[2] = tmp_pose[2] + laser_pose[2];
            pose_vec.push_back(out_pose);
        }
        return true;
    }
    return false;
}

#else

bool FeatureRecognition::matchAngle(const sros::core::LaserScanMsg &scan_const,
                                    AngleFeature &inputangle, Eigen::Vector3f &laser_pose,
                                    vector<Eigen::Vector3f> &pose_vec) {
    decodeLidar(scan_const);

    vector<Eigen::Vector3f> pose_vector;
    if (matchAngle(RadarRho, RadarTheta, RadarDataCnt, inputangle, pose_vector)) {
        for (auto p : pose_vector) {
            Eigen::Vector3f out_pose;
            out_pose[0] = p[0] * cos(laser_pose[2]) - p[1] * sin(laser_pose[2]) + laser_pose[0];
            out_pose[1] = p[0] * sin(laser_pose[2]) + p[1] * cos(laser_pose[2]) + laser_pose[1];
            out_pose[2] = p[2] + laser_pose[2];
            pose_vec.push_back(out_pose);
        }
        return true;
    }
    return false;
}

sros::core::Pose_Vector FeatureRecognition::matchAngle(const sros::core::LaserScanMsg &scan_const,
                                                       const Eigen::Vector3f &cur_laser_pose) {
    vector<Eigen::Vector3f> out_vector;

    sros::core::Pose_Vector results;

    Eigen::Vector3f laser_pose = cur_laser_pose;

    matchAngle(scan_const, input_anglefeature, laser_pose, out_vector);

    for (auto pose : out_vector) {
        results.push_back(sros::core::Pose(sros::core::Location(pose[0], pose[1]),
                                           sros::core::Rotation(pose[2])));
    }

    return results;
}

void FeatureRecognition::setAngleFeature(double theta, double d1, double d2) {
    if (theta >= M_PI) {
        input_anglefeature.angle = theta - M_PI;
        input_anglefeature.is_reflexangle_side = false;
    } else {
        input_anglefeature.angle = theta;
        input_anglefeature.is_reflexangle_side = true;
    }
    input_anglefeature.line1_length = d1 * 1000;
    input_anglefeature.line2_length = d2 * 1000;
}

#endif

float FeatureRecognition::getPointDist(iPoint &start, iPoint &end) {
    return sqrt(
            (start.x - end.x) * (start.x - end.x)
            + (start.y - end.y) * (start.y - end.y));
}

LinePara FeatureRecognition::findLongestLine(vector<LinePara> &line_vector, int &index) {
    LinePara &tmpline = line_vector[0];
    int size = line_vector.size();
    for (int i = 1; i < size; ++i) {
        if (tmpline.length < line_vector[i].length) {
            index = i;
            tmpline = line_vector[i];
        }
    }
    return tmpline;

}

void FeatureRecognition::initialize(const sros::core::LaserScanMsg& scan) {
    if (!is_initialized) {

        scan_size = scan.ranges.size();
        RadarRho = new int[scan_size];
        memset(RadarRho, 0, scan_size * sizeof(int));
        RadarTheta = new float[scan_size];
        memset(RadarTheta, 0, scan_size * sizeof(float));
        RadarX = new int[scan_size];
        memset(RadarX, 0, scan_size * sizeof(int));
        RadarY = new int[scan_size];
        memset(RadarY, 0, scan_size * sizeof(int));
        RadarSinTheta = new float[scan_size];
        memset(RadarSinTheta, 0, scan_size * sizeof(float));
        RadarCosTheta = new float[scan_size];
        memset(RadarCosTheta, 0, scan_size * sizeof(float));
    }
    is_initialized = true;
}
} /* namespace slam */
