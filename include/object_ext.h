#ifndef _OBJECT_EXT_H_
#define _OBJECT_EXT_H_

#include "defines.h"
#include "user_param.h"
#include "cloud_frame.h"
#include "timer.h"
#include <opencv2/opencv.hpp>

class MaplessDynamic;
class UserParam;

class ObjectExt
{
private:
    int img_height_;
    int img_width_;

    int thr_object_;
    float alpha_;
    float beta_;
    float coef_accum_w_[2];

    float weight_factor_;
    float seg_deg_;
public:

    std::vector<int> idx_row_;
    std::vector<int> idx_col_;
    std::vector<int> check_;

    std::vector<int> object_row_;
    std::vector<int> object_col_;
    std::vector<float> object_rho_roi_;

    // std::vector<float> max_his_object_rho_roi_;

    std::vector<int> disconti_row_;
    std::vector<int> disconti_col_;

    std::vector<int> diff_object_area_conti_row_;
    std::vector<int> diff_object_area_conti_col_;

    std::vector<float> diff_z_;

    cv::MatND histogram_;

public:
    ObjectExt(const std::unique_ptr<UserParam> &user_param);
    ~ObjectExt();

    void filterOutAccumdR(std::unique_ptr<CloudFrame>& CloudFrame_next, std::unique_ptr<CloudFrame>& CloudFrame_cur_warped,
                             cv::Mat &accumulated_dRdt, cv::Mat &accumulated_dRdt_score, cv::Mat &residual);

    void extractObjectCandidate(cv::Mat &accumulated_dRdt, std::unique_ptr<CloudFrame>& CloudFrame_next, float object_factor);

    void checkSegment(cv::Mat &accumulated_dRdt, std::unique_ptr<CloudFrame>& CloudFrame_next, cv::Mat &groundPtsIdx_next);

    void updateAccum(cv::Mat &accumulated_dRdt, cv::Mat &accumulated_dRdt_score);

    void updateAccumdRdt(std::unique_ptr<CloudFrame>& CloudFrame_next, cv::Mat &accumulated_dRdt, cv::Mat &accumulated_dRdt_score, cv::Mat& dRdt, cv::Mat& groundPtsIdx_next);

    void updateAccumScore(cv::Mat &accumulated_dRdt, cv::Mat &accumulated_dRdt_score);
};

#endif