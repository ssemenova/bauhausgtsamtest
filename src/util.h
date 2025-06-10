
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

namespace gtsam
{
    class State
    {
    public:
        Pose3 pose_;                 // Rotation from global to IMU, Position of IMU in global
        Vector3 velocity_;           // Velocity of IMU in global
        imuBias::ConstantBias bias_; // Bias of IMU

        State(const Pose3 &pose, const Vector3 &velocity, const imuBias::ConstantBias &bias)
        {
            this->pose_ = pose;
            this->velocity_ = velocity;
            this->bias_ = bias;
        }
    };

    class ImuMeasurement
    {
    public:
        Vector3 measuredAcc;
        Vector3 measuredOmega;
        double dt;

        ImuMeasurement(Vector3 measuredAcc, Vector3 measuredOmega, double dt)
        {
            this->measuredAcc = measuredAcc;
            this->measuredOmega = measuredOmega;
            this->dt = dt;
        }
    };
}

typedef std::vector<std::tuple<gtsam::Point2, int>> TrackedFeatures;
typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;

std::vector<cv::Point2f> get_tracked_features_as_vector_of_point2f(
    const TrackedFeatures &tracked_features
)
{
    std::vector<cv::Point2f> features;
    for (const auto &feature : tracked_features)
    {
        features.push_back(cv::Point2f(std::get<0>(feature).x(), std::get<0>(feature).y()));
    }
    return features;
}

void print_features(const TrackedFeatures &features)
{
    std::cout << "Features: ";
    for (const auto &feature : features)
    {
        std::cout << "(" << std::get<0>(feature).x() << ", " << std::get<0>(feature).y() << ", " << std::get<1>(feature) << "), ";
    }
    std::cout << std::endl;
}