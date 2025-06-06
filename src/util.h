
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

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
