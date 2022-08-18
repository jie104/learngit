#pragma once

#include <memory>
namespace extract
{
    struct ExtractPara
    {
        unsigned int sample_point_size_ = 1400; //降采样点个数1400
        float lambda_ = 0.1745f; //lambda = 10度
        float sigma_ = 0.05f;   //sigma = 0.05m
        float farthest_dist_threshold_ = 0.06f;

        unsigned int line_segment_point_size_threshold_ = 5;    //线段点个数阈值（如果区域内点数少于此阈值，则认为点数过少，不拟合直线）
        float line_segment_length_threshold_ = 0.05f;//0.5f    //线段长度阈值（如果拟合的线段长度小于此阈值，则舍弃）
        float dist_ratio_thre_smear_ = 0.07;// 相邻激光点探测距离之差，与该点实际探测距离的比例阈值，太大判定为断点,0.4
        float angle_thre_smear_ = M_PI / 9;//相邻激光点连线与轴向的夹角的阈值，低于该阈值说明为拖尾M_PI/6
        float min_angle_for_multi_type_corner = 0.1745f*3;//墙角、三角型这种特征的最小夹角为30°

        float articorner_lenth = 0.29f;//边长为29cm
        float articorner_lenth_ratio_thre = 0.4f;
        float articorner_include_angle = 137.0f * 0.01745f;//137度夹角
//        float corner_include_angle_ratio_thre = 0.2f;
        float articorner_include_angle_thre = 15.0f * 0.01745f;//测试发现在-5~10°之间，85%的数据在0~5°之间

        float corner_min_lengh = 0.25f;//
        float corner_include_angle = 90.0f * 0.01745f;
        float corner_include_angle_thre = 10.0f * 0.01745f;

        float triangle_length_ = 0.29f;//三角形最小边长，后面应该修改成该数值上下一定范围
        float triangle_lenth_ratio_thre = 0.4f;
        float triangle_include_angle = (360.0f-137.0f) * 0.01745f;//90°
        float triangle_include_angle_thre = 10.0f * 0.01745f;

        float range_close_threshold_ = 0.2f;//0.2   //范围差阈值0.2（相邻点range差大于此阈值则认为该两点不连续）
        float intensity_weak_threshold_ = 32.f; //反光强度弱阈值(小于等于此阈值被认为噪点),这个是原始非归一化的强度值
        unsigned int strong_count_threshold_ = 4;//4   //特征点数量阈值（特征点数量小于此阈值的圆柱被舍弃）
        unsigned int weak_count_threshold_ = 10; //噪点数量阈值（连续噪点数量大于此阈值的圆柱被舍弃）
        float landmark_distance_threshold_ = 25.f;  //信标特征最大有效距离

    };

    using ExtractPara_ptr = std::shared_ptr<ExtractPara>;
}
