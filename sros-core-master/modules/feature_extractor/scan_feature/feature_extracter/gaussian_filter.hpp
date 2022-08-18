//
// Created by lfc on 2020/7/2.
//

#ifndef SCAN_GAUSSIAN_FILTER_HPP
#define SCAN_GAUSSIAN_FILTER_HPP

namespace extractor{
class GaussianFilter {
 public:
    template <class ScanType>
    static void filter(ScanType& scan){
        static double gaussian_array[5];
        static bool first_process = true;
        static float continous_thresh = 0.1f;
        if (first_process) {
            first_process = false;
            gaussian_array[0] = 1.f / 16.f;
            gaussian_array[1] = 0.25f;
            gaussian_array[2] = 6.f / 16.f;
            gaussian_array[3] = gaussian_array[1];
            gaussian_array[4] = gaussian_array[0];
        }
        int scan_size = scan->ranges.size();
        auto& ranges = scan->ranges;
        int loop_start = 2, loop_end = scan_size - 2;

        auto computeWeightDist = [](float& range_curr,const float &weight,const float & range_ref,const float continous_thresh = 0.1){
            range_curr = ((fabs(range_ref - range_curr) < continous_thresh) ? range_curr : range_ref) * weight;
        };
        std::vector<float> bk_ranges(scan_size,0.0f);
        for (int i = loop_start; i < loop_end; ++i) {
            if (ranges[i] != 0) {
                float range_2_p = ranges[i - 2], range_1_p = ranges[i - 1], range_2_s = ranges[i + 2],
                      range_1_s = ranges[i + 1], range = ranges[i];
                computeWeightDist(range_2_p, gaussian_array[0], range);
                computeWeightDist(range_1_p, gaussian_array[1], range);
                computeWeightDist(range_1_s, gaussian_array[3], range);
                computeWeightDist(range_2_s, gaussian_array[4], range);
                bk_ranges[i] = range_1_p + range_2_p + range * gaussian_array[2] + range_2_s + range_1_s;
            }
        }
        for (int j = 0; j < loop_start; ++j) {
            bk_ranges[j] = scan->ranges[j];
        }
        for (int k = loop_end; k < scan_size; ++k) {
            bk_ranges[k] = scan->ranges[k];
        }
        ranges.swap(bk_ranges);
    }

    template <class ScanType>
    static void filter(const ScanType& scan,std::vector<float>& filtered_ranges){
        static double gaussian_array[5];
        static bool first_process = true;
        static float continous_thresh = 0.1f;
        if (first_process) {
            first_process = false;
            gaussian_array[0] = 1.f / 16.f;
            gaussian_array[1] = 0.25f;
            gaussian_array[2] = 6.f / 16.f;
            gaussian_array[3] = gaussian_array[1];
            gaussian_array[4] = gaussian_array[0];
        }
        int scan_size = scan->ranges.size();
        auto& ranges = scan->ranges;
        int loop_start = 2, loop_end = scan_size - 2;

        auto computeWeightDist = [](float& range_curr,const float &weight,const float & range_ref,const float continous_thresh = 0.1){
            range_curr = ((fabs(range_ref - range_curr) < continous_thresh) ? range_curr : range_ref) * weight;
        };
        std::vector<float> bk_ranges(scan_size,0.0f);
        for (int i = loop_start; i < loop_end; ++i) {
            if (ranges[i] != 0) {
                float range_2_p = ranges[i - 2], range_1_p = ranges[i - 1], range_2_s = ranges[i + 2],
                      range_1_s = ranges[i + 1], range = ranges[i];
                computeWeightDist(range_2_p, gaussian_array[0], range);
                computeWeightDist(range_1_p, gaussian_array[1], range);
                computeWeightDist(range_1_s, gaussian_array[3], range);
                computeWeightDist(range_2_s, gaussian_array[4], range);
                bk_ranges[i] = range_1_p + range_2_p + range * gaussian_array[2] + range_2_s + range_1_s;
            }
        }
        for (int j = 0; j < loop_start; ++j) {
            bk_ranges[j] = scan->ranges[j];
        }
        for (int k = loop_end; k < scan_size; ++k) {
            bk_ranges[k] = scan->ranges[k];
        }
        filtered_ranges.swap(bk_ranges);
    }

    static
    float getGaussianMean(const float &range_1_p,const float &range,const float &range_1_s,const float range_thresh = 0.1){
        float mean_range = range * 0.5f;
        mean_range += (fabs(range_1_p - range) < range_thresh ? range_1_p : range) * 0.25;
        mean_range += (fabs(range_1_s - range) < range_thresh ? range_1_s : range) * 0.25;
        return mean_range;
    }
 private:

};

}

#endif  // SRC_GAUSSIAN_FILTER_HPP
