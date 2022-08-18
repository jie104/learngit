//
// Created by ljh on 2021/12/2.
//

#ifndef PERCEPTION_SOLUTION_DESTACK_DETECTOR_H
#define PERCEPTION_SOLUTION_DESTACK_DETECTOR_H

// INCLUDE
#include "../card_detector/card.hpp"
#include "../card_detector/card_target.h"
#include "decards_detect_param.hpp"
#include "../base/detector.h"
#include "../base/livox_pallet_detect.hpp"
// CODE
namespace perception {

class DecardsDetector : public Detector {
public:
    using CardTarget = perception::CardTarget;

    /** @brief constructor. */
    explicit DecardsDetector();

    /** @brief destructor. */
    ~DecardsDetector() override;

    /** @brief display param. */
    void showParam() override;

    /** @brief set runtime data as default value. */
    void reset() override;

    /**
     * @brief detect target in frame.
     * @param[in] frame 3D camera frame.
     * @return return detect result.
     */
    DetectResult detect(const O3d3xxFrame &frame) override;
    DetectResult detect_test(const O3d3xxFrame &frame);
    /**
     * @brief detect if card stack in image is top
     * @param frame
     * @return
     */
    bool stackIsTop(const O3d3xxFrame &frame);

    /**
     * @brief detect target in frame. 旧拆栈板方法，目前弃用
     * @param frame
     * @param flag
     * @return
     */
    DetectResult detect(const O3d3xxFrame &frame, bool flag);

    /** @brief init detect param and build point handler.
     *  @param[in] param detect param.
     *  @param[in] cards detect cards param.
     *  @return if cards.size = 0 return false, otherwise return ture;
     */
    bool init(const DecardsDetectParam &param, const std::map<int, Card> &cards);


    /**
     * @brief get an amplitude normalize image.
     * @param img output image.
     */
    void getAmplitudeImage(cv::Mat &img) const;

    /**
     * @brief get an configdence normalize image.
     * @param img output image.
     */
    void getConfidenceImage(cv::Mat &img) const;

    /**
     * @brief get an image with a rectangular box with a pallet.
     * @param[out] img output image.
     */
    void getPalletImage(cv::Mat &img) const;

    /**
     * @brief get an image with a rectangular box with a hole.
     * @param[out] img output image.
     */
    void getHoleImage(cv::Mat &img) const;

    /**
     * @brief get an image that deep image
     * @param img out image.
     */
    void getDeepImage(cv::Mat &img) const;

    /**
     * @brief get an xyz threshold image.
     * @param img out image.
     */
    void getXYZThresholdImage(cv::Mat &img) const;

    /**
     *
     * @param param
     */
    void setParam(const DecardsDetectParam& param) {
        this->param_ = param;
    }
    
private:

/**
     * @brief search target in pallet map.
     * @return if found target return true, then return false.
     */
    bool searchTarget() override;

    /**
     * @brief correct target width according to center pallet width.
     * @param[in] param input detect param.
     * @param[in,out] card output detect card.
     * @param[in,out] clusters_map output pallets map.
     * @param[in,out] target output target.
     */
    void correctTargetWidth();

    /**
     * @brief generate target image
     * @param[in] cloud input point cloud.
     * @param[out] target_point_cloud output target point cloud.
     */
    void generateTargetImage(const PointCloudConstPtr &cloud,
                             PointCloudPtr &target_point_cloud);

    /**
     * @brief euclidean segmentation.
     * @param[in] normals input normal cloud.
     * @param[in] cloud input point cloud.
     * @param[in] clusters Multiple clusters point indices of cloud.
     * @param[out] clusters_map segmentation pallet set.
      */
    void filterClusters(const NormalCloudConstPtr &normals,
                        const PointCloudConstPtr &cloud,
                        const std::vector<Indices> &clusters_indices,
                        std::map<int, Cluster> &clusters_map);

    /**
     * @brief found rectangle in image source.
     * @param[in] param input detect param.
     * @param[in] img_src input image.
     * @param[out] rects rectangles found.
     */
    static void findRectangle(
            const DecardsDetectParam &param,
            const cv::Mat &img_src,
            std::vector<cv::Rect> &rects);

    /**
     * @brief check the target is card.if the target is confirmed to be a card, it returns true;
     *        otherwise, it returns false
     * @return if the target is confirmed to be a card, it returns true; otherwise, it returns false
     */
    bool targetIsCard();

    /**
     * @brief correct target height.
     */
    void correctTargetHeight();

    /**
     * @brief calculate card detect result.if card all pallet point cloud fitting plane fail,
     *        return false; else return true;
     * @return if card all pallet point cloud fitting plane fail, return false; else return true;
     */
    bool calculateResult() override;

    /**
     * @brief draw 'card hole' text on card hole image.
     */
    void drawCardHole();

    /**
     * @brief draw 'card pallet' text on card pallet image.
     */
    void drawCardPallet();

    /**
     * @brief draw 'card target' text on card target image.
     */
    void drawTarget();

    /** @brief detect param. */
    DecardsDetectParam param_{};

    /** @brief card type to be detected. */
    std::map<int, Card> cards_map_;

    /** @brief region of interest image. */
    cv::Mat_<uint8_t> roi_img_;

    /** @brief pallet image. */
    cv::Mat pallet_img_;

    /** @brief hole image. */
    cv::Mat hole_img_;

    /** @breif xyz threshold image. */
    cv::Mat xyz_threshold_img_;

    /** @brief range of interest. */
    cv::Rect roi_rect_;

    /** @brief the bounding rectangle of the card board */
    cv::Rect card_rect_;

    /** @brief detect target */
    CardTarget target_;

    /** @brief mask filtered point cloud of top card. */
    PointCloudPtr top_card_cloud_;

    /** @brief mask filtered point cloud of bottom card. */
    PointCloudPtr bottom_card_cloud_;

    /** @brief top card 's point cloud after median filter. */
    PointCloudPtr top_median_cloud_;

    /** @brief bottom card 's point cloud after median filter. */
    PointCloudPtr bottom_median_cloud_;


public:
    typedef std::shared_ptr<DecardsDetector> Ptr;
    typedef std::shared_ptr<const DecardsDetector> ConstPtr;
};

}


#endif //PERCEPTION_SOLUTION_DESTACK_DETECTOR_H
