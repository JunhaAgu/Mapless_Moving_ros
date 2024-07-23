#ifndef _CLOUD_FRAME_H_
#define _CLOUD_FRAME_H_

#include "defines.h"
#include "user_param.h"
#include "timer.h"

#include "immintrin.h" // AVX, AVX2 ...


class MaplessDynamic;
class UserParam;

class CloudFrame
{
    friend class dRCalc;
    
    private:
        std::string dataset_name_;

        float azimuth_res_;
        float az_step_;
        int n_horizontal_;
        int n_vertical_;
        int n_pts_;
        std::vector<float> v_angle_;

        float lidar_elevation_criteria_[4];
        float lidar_elevation_line0_[2];
        float lidar_elevation_line1_[2];

        int img_height_;
        int img_width_;

    public:
        std::shared_ptr<StrRhoPts> str_rhopts_;

    public:
        CloudFrame(const std::unique_ptr<UserParam>& user_param);
        ~CloudFrame();

        void genRangeImages(PointCloudwithTime::Ptr pcl_in, bool cur_next);

        void genRangeImages_dR(PointCloudwithTime::Ptr pcl_in, bool cur_next);

        void genRangeImages_noComp(PointCloudwithTime::Ptr pcl_in, bool cur_next);

        void calcuateRho(PointCloudwithTime::Ptr pcl_in, bool cur_next);
        
        void calcuateRho_SIMD(PointCloudwithTime::Ptr pcl_in, bool cur_next);

        void makeRangeImageAndPtsPerPixel(bool cur_next);

        void makeRangeImageAndPtsPerPixel_dR(bool cur_next);
        
        void interpRangeImage(bool cur_next);

        void interpRangeImage_dR(bool cur_next);

        void interpPts(PointCloudwithTime::Ptr pcl_in, bool cur_next);

        void reset();
};


#endif