#ifndef ICP_CONNECTODOMETRY_H_
#define ICP_CONNECTODOMETRY_H_

#include "OdometryProvider.h"

class ICP_ConnectOdometry : public OdometryProvider
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ICP_ConnectOdometry(std::vector<Eigen::Vector3f> & tvecs_,
                    std::vector<Eigen::Matrix<float, 3, 3, Eigen::RowMajor> > & rmats_,
                    std::vector<DeviceArray2D<float> > & vmaps_g_prev_,
                    std::vector<DeviceArray2D<float> > & nmaps_g_prev_,
                    std::vector<DeviceArray2D<float> > & vmaps_curr_,
                    std::vector<DeviceArray2D<float> > & nmaps_curr_,
                    std::vector<DeviceArray2D<float> > & nsmaps_g_prev_,
                    std::vector<DeviceArray2D<float> > & nsmaps_curr_,
                    Intr & intr,
                    float distThresh = 0.10f,
                    float angleThresh = sin(20.f * 3.14159254f / 180.f));

        virtual ~ICP_ConnectOdometry();

        CloudSlice::Odometry getIncrementalTransformation(Eigen::Vector3f & trans,
                                                          Eigen::Matrix<float, 3, 3, Eigen::RowMajor> & rot,
                                                          const DeviceArray2D<unsigned short> & depth,
                                                          const DeviceArray2D<PixelRGB> & image,
                                                          uint64_t timestamp,
                                                          unsigned char * rgbImage,
                                                          unsigned short * depthData);

        Eigen::MatrixXd getCovariance();

        void reset();

        static const int LEVELS = 4;

    private:
        int icp_iterations_[LEVELS];

        std::vector<Eigen::Vector3f> & tvecs_;
        std::vector<Eigen::Matrix<float, 3, 3, Eigen::RowMajor> > & rmats_;

        std::vector<DeviceArray2D<float> > & vmaps_g_prev_;
        std::vector<DeviceArray2D<float> > & nmaps_g_prev_;

        std::vector<DeviceArray2D<float> > & vmaps_curr_;
        std::vector<DeviceArray2D<float> > & nmaps_curr_;

        std::vector<DeviceArray2D<float> > & nsmaps_g_prev_;
        std::vector<DeviceArray2D<float> > & nsmaps_curr_;

        Intr & intr;

        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> lastA;

        DeviceArray<JtJJtrSE3> sumDataSE3;
        DeviceArray<JtJJtrSE3> outDataSE3;

        float distThres_;
        float angleThres_;
};

#endif /* ICP_CONNECTODOMETRY_H_ */
