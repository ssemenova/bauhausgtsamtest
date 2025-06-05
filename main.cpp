// #include <gtsam/geometry/Pose2.h>
// #include <gtsam/geometry/Point2.h>
// #include <gtsam/inference/Symbol.h>
// #include <gtsam/slam/PriorFactor.h>
// #include <gtsam/slam/BetweenFactor.h>
// #include <gtsam/slam/BearingRangeFactor.h>
// #include <gtsam/nonlinear/NonlinearFactorGraph.h>
// #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// #include <gtsam/nonlinear/Marginals.h>
// #include <gtsam/nonlinear/Values.h>
// #include <gtsam/nonlinear/NonlinearFactorGraph.h>
// #include <gtsam/inference/Symbol.h>
// #include <gtsam/nonlinear/Values.h>
// #include <gtsam/nonlinear/ISAM2.h>

// // Factors
// #include <gtsam/slam/PriorFactor.h>
// #include <gtsam/navigation/CombinedImuFactor.h>
// #include <gtsam/slam/ProjectionFactor.h>
// #include <gtsam/slam/SmartProjectionPoseFactor.h>

// Graphs
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

// Factors
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>


using namespace std;
using namespace gtsam;

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


//* CONSTANTS *//
const double FX = 458.654;
const double FY = 457.296;
const double S = 0.0;
const double CX = 367.215;
const double CY = 248.375;
const double SIGMA_CAMERA = 0.306555403;
const Pose3 tbc = gtsam::Pose3(
    gtsam::Quaternion(0.7123014606689537, -0.0077071797555374275, 0.010499323370587278, 0.7017528002919717),
    gtsam::Point3(-0.0216401454975, -0.064676986768, 0.00981073058949));

// ... Initialization
const double INITIAL_ROLL_PITCH_SIGMA = 0.174533;
const double INITIAL_YAW_SIGMA = 0.00174533;
const double INITIAL_POSITION_SIGMA = 1e-05;
const double INITIAL_VELOCITY_SIGMA = 0.001;
const double INITIAL_ACC_BIAS_SIGMA = 0.1;
const double INITIAL_GYRO_BIAS_SIGMA = 0.01;
const double GYRO_NOISE_DENSITY = 1.7e-4;
const double ACCEL_NOISE_DENSITY = 2.0000e-3;
const double IMU_INTEGRATION_SIGMA = 1.0e-8;
const double IMU_BIAS_INIT_SIGMA = 1e-3;
const double ACCEL_RANDOM_WALK = 3.0000e-2;
const double GYRO_RANDOM_WALK = 1.9393e-05;

// * INPUTS *//
const gtsam::Pose3 initial_pose = gtsam::Pose3(gtsam::Quaternion(0.5308, -0.1365, -0.8329, -0.0761), gtsam::Point3(4.6331, -1.8072, 0.8306));
const gtsam::Vector3 initial_velocity = gtsam::Vector3(-0.060768, 0.054005, 0.617824);
const gtsam::imuBias::ConstantBias initial_bias = gtsam::imuBias::ConstantBias(
    gtsam::Vector3(-0.024348, 0.144441, 0.06754),
    gtsam::Vector3(-0.002535, 0.021162, 0.07717));

const std::vector<gtsam::ImuMeasurement> imu_measurements = {
    gtsam::ImuMeasurement(
        gtsam::Vector3(9.536967277526855, -0.1062387079000473, -4.012554168701172),
        gtsam::Vector3(-0.04188790172338486, 0.4663519859313965, 0.012566370889544487),
        0.010000228881835938),
    gtsam::ImuMeasurement(
        gtsam::Vector3(9.569655895233154, -0.2206496223807335, -4.073845863342285),
        gtsam::Vector3(-0.030368728563189507, 0.47856926918029785, 0.027576201595366),
        0.004999876022338867),
    gtsam::ImuMeasurement(
        gtsam::Vector3(9.48793363571167, -0.3064578175544739, -4.1228790283203125),
        gtsam::Vector3(-0.018151423893868923, 0.4925318956375122, 0.03735004551708698),
        0.004999876022338867),
    gtsam::ImuMeasurement(
        gtsam::Vector3(9.304059028625488, -0.34323275089263916, -4.126965045928955),
        gtsam::Vector3(-0.008726646425202489, 0.5064945220947266, 0.04642575792968273),
        0.00500035285949707),
    gtsam::ImuMeasurement(
        gtsam::Vector3(9.042548656463623, -0.3064578101038933, -4.102448463439941),
        gtsam::Vector3(0.0006981317419558764, 0.5124286413192749, 0.051661744713783264),
        0.004999876022338867),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.8178129196167, -0.17978858575224876, -4.061587572097778),
        gtsam::Vector3(0.007679448928683996, 0.514523059129715, 0.04991641640663147),
        0.004999876022338867),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.629851818084717, -0.08580818586051464, -4.024812698364258),
        gtsam::Vector3(0.009424778167158365, 0.5127777457237244, 0.0478220209479332),
        0.004999876022338867),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.556302070617676, 0.008172208443284035, -4.008468151092529),
        gtsam::Vector3(0.008377580437809229, 0.5089380145072937, 0.045727625489234924),
        0.00500035285949707),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.580819129943848, 0.09398039430379868, -4.000295877456665),
        gtsam::Vector3(0.007330382941290736, 0.5099852085113525, 0.045378560200333595),
        0.004999876022338867),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.593077182769775, 0.11849702149629593, -3.983951449394226),
        gtsam::Vector3(0.0059341194573789835, 0.5159193277359009, 0.04712388850748539),
        0.004999876022338867),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.556302070617676, 0.17570247873663902, -3.9635210037231445),
        gtsam::Vector3(0.002443460834911093, 0.5201081335544586, 0.04921828396618366),
        0.004999876022338867),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.54813003540039, 0.25333844870328903, -3.9676071405410767),
        gtsam::Vector3(-0.004188790364423767, 0.5232497155666351, 0.04956735111773014),
        0.00500035285949707),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.613507747650146, 0.2901133894920349, -3.943090558052063),
        gtsam::Vector3(-0.009075712412595749, 0.5270894169807434, 0.048171088099479675),
        0.004999876022338867),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.699316024780273, 0.2615106701850891, -3.9185739755630493),
        gtsam::Vector3(-0.011868238914757967, 0.5260422229766846, 0.04712389037013054),
        0.004999876022338867),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.756521224975586, 0.17570248246192932, -3.8981434106826782),
        gtsam::Vector3(-0.015707962680608034, 0.5222025215625763, 0.045727627351880074),
        0.004999876022338867),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.821898937225342, 0.07763598021119833, -3.873626708984375),
        gtsam::Vector3(-0.015358896926045418, 0.5197590589523315, 0.045029494911432266),
        0.00500035285949707),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.854588031768799, -0.032688834704458714, -3.869540572166443),
        gtsam::Vector3(-0.010122909676283598, 0.5190609097480774, 0.048171086236834526),
        0.004999876022338867),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.940395832061768, -0.08172208443284035, -3.869540572166443),
        gtsam::Vector3(-0.0041887902189046144, 0.5204571783542633, 0.05654866807162762),
        0.004999876022338867),
    gtsam::ImuMeasurement(
        gtsam::Vector3(8.960826396942139, -0.08989429101347923, -3.824593424797058),
        gtsam::Vector3(-0.00034906581277027726, 0.5204571783542633, 0.06457718275487423),
        0.005000114440917969)
};

std::vector<gtsam::Point2> feature_points = {
    gtsam::Point2(8.960826396942139, -0.089894291013479223)
};



void initialize(NonlinearFactorGraph &graph, PreintegratedCombinedMeasurements *& preint, Values & values)
{
    // Create prior factor and add it to the graph

    // Sofiya: quaternion is wxyz here but xyzw in bauhaus
    gtsam::State prior_state = gtsam::State(initial_pose, initial_velocity, initial_bias);

    std::cout << "INITIALIZATION!" << std::endl;
    std::cout << "\t Translation: " << initial_pose.translation().transpose() << std::endl;
    std::cout << "\t Rotation: " << initial_pose.rotation().toQuaternion() << std::endl;
    std::cout << "\t Velocity: " << initial_velocity.transpose() << std::endl;
    std::cout << "\t Bias: " << initial_bias << std::endl;

    // Set initial pose uncertainty: constrain mainly position and global yaw.
    // roll and pitch is observable, therefore low variance.
    Matrix6 pose_prior_covariance = Matrix6::Zero();
    pose_prior_covariance.diagonal()[0] = INITIAL_ROLL_PITCH_SIGMA * INITIAL_ROLL_PITCH_SIGMA;
    pose_prior_covariance.diagonal()[1] = INITIAL_ROLL_PITCH_SIGMA * INITIAL_ROLL_PITCH_SIGMA;
    pose_prior_covariance.diagonal()[2] = INITIAL_YAW_SIGMA * INITIAL_YAW_SIGMA;
    pose_prior_covariance.diagonal()[3] = INITIAL_POSITION_SIGMA * INITIAL_POSITION_SIGMA;
    pose_prior_covariance.diagonal()[4] = INITIAL_POSITION_SIGMA * INITIAL_POSITION_SIGMA;
    pose_prior_covariance.diagonal()[5] = INITIAL_POSITION_SIGMA * INITIAL_POSITION_SIGMA;

    // Rotate initial uncertainty into local frame, where the uncertainty is
    // specified.
    auto B_Rot_W = initial_pose.rotation().matrix();
    pose_prior_covariance.topLeftCorner(3, 3) =
        B_Rot_W * pose_prior_covariance.topLeftCorner(3, 3) * B_Rot_W.transpose();

    // Add pose prior.
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        gtsam::Symbol('x', 0),
        prior_state.pose_,
        gtsam::noiseModel::Gaussian::Covariance(pose_prior_covariance)));

    // Add initial velocity priors.
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(
        gtsam::Symbol('v', 0),
        prior_state.velocity_,
        gtsam::noiseModel::Isotropic::Sigma(3, INITIAL_VELOCITY_SIGMA) // initialVelocitySigma
        ));

    // Add initial bias priors:
    gtsam::Vector6 b_noise = gtsam::Vector6::Zero();
    b_noise.head<3>().setConstant(INITIAL_ACC_BIAS_SIGMA);
    b_noise.tail<3>().setConstant(INITIAL_GYRO_BIAS_SIGMA);

    gtsam::SharedNoiseModel imu_bias_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(b_noise, true);

    graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        gtsam::Symbol('b', 0),
        prior_state.bias_,
        imu_bias_prior_noise
    ));

    // Add initial state to the graph
    values.insert(gtsam::Symbol('x', 0), prior_state.pose_);
    values.insert(gtsam::Symbol('v', 0), prior_state.velocity_);
    values.insert(gtsam::Symbol('b', 0), prior_state.bias_);

    // Create GTSAM preintegration parameters for use with Foster's version
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
        params;
    params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(); // Z-up navigation frame: gravity points along negative Z-axis !!!

    params->setGyroscopeCovariance(gtsam::I_3x3 * GYRO_NOISE_DENSITY * GYRO_NOISE_DENSITY);
    params->setAccelerometerCovariance(gtsam::I_3x3 * ACCEL_NOISE_DENSITY * ACCEL_NOISE_DENSITY);
    params->setIntegrationCovariance(gtsam::I_3x3 * IMU_INTEGRATION_SIGMA * IMU_INTEGRATION_SIGMA);
    params->biasAccOmegaInt = IMU_BIAS_INIT_SIGMA * gtsam::Matrix66::Identity(6, 6);
    params->biasAccCovariance = ACCEL_RANDOM_WALK * ACCEL_RANDOM_WALK * gtsam::Matrix33::Identity(3, 3);
    params->biasOmegaCovariance = GYRO_RANDOM_WALK * GYRO_RANDOM_WALK * gtsam::Matrix33::Identity(3, 3); // gyroscope_random_walk

    // Actually create the GTSAM preintegration
    preint = new gtsam::PreintegratedCombinedMeasurements(params, prior_state.bias_);
}

gtsam::State predict_state(PreintegratedCombinedMeasurements &preint, Values &values, int ct_state)
{
    // Get the current state (t=k)
    gtsam::State state_k = gtsam::State(
        values.at(gtsam::Symbol('x', ct_state)).cast<Pose3>(),
        values.at(gtsam::Symbol('v', ct_state)).cast<Vector3>(),
        values.at(gtsam::Symbol('b', ct_state)).cast<imuBias::ConstantBias>());

    // std::cout << "Current state used for imu-based prediction: " << std::endl;
    // std::cout << "Pose: " << state_k.pose_ << std::endl;
    // std::cout << "Velocity: " << state_k.velocity_ << std::endl;
    // std::cout << "Bias: " << state_k.bias_ << std::endl;

    // From this we should predict where we will be at the next time (t=K+1)
    NavState state_k1 = preint.predict(
        gtsam::NavState(
            state_k.pose_,
            state_k.velocity_),
        state_k.bias_);

    gtsam::State predicted = gtsam::State(
        state_k1.pose(),
        state_k1.velocity(),
        state_k.bias_);

    return predicted;
}

void preintegrate(PreintegratedCombinedMeasurements *&preint)
{
    for (int i = 0; i < imu_measurements.size(); i++)
    {
        preint->integrateMeasurement(imu_measurements[i].measuredAcc, imu_measurements[i].measuredOmega, imu_measurements[i].dt);
    }
}

void create_imu_factor(PreintegratedCombinedMeasurements & preint, NonlinearFactorGraph & graph)
{
    CombinedImuFactor imu_factor = CombinedImuFactor(
        gtsam::Symbol('x', 0), // pose at time t
        gtsam::Symbol('v', 0), // velocity at time t
        gtsam::Symbol('x', 1), // pose at time t+1
        gtsam::Symbol('v', 1), // velocity at time t+1
        gtsam::Symbol('b', 0), // bias at time t
        gtsam::Symbol('b', 1), // bias at time t+1
        preint);
    graph.add(imu_factor);
}

void process_smart_features(NonlinearFactorGraph & graph)
{
    for (int i = 0; i < feature_points.size(); i++)
    {
        // Sofiya: For this example, removed all the code that checks if the feature is already in the graph
        // because we are only running this once

        // Create a smart factor for the new feature
        noiseModel::Isotropic::shared_ptr measurement_noise = noiseModel::Isotropic::Sigma(2, SIGMA_CAMERA, true); // sigma_camera
        gtsam::Cal3_S2 K = gtsam::Cal3_S2(FX, FY, S, CX, CY);

        // Note (frames): Transformation from camera frame to imu frame, i.e., pose of imu frame in camera frame
        SmartProjectionPoseFactor<gtsam::Cal3_S2> factor = SmartProjectionPoseFactor<gtsam::Cal3_S2>(
            measurement_noise,
            boost::make_shared<gtsam::Cal3_S2>(K), // calibration
            boost::optional<gtsam::Pose3>(tbc)); // body_P_sensor

        // Insert measurements to a smart factor
        factor.add(feature_points[i], gtsam::Symbol('x', 1)); // Add measurement

        // Add smart factor to FORSTER2 model
        graph.add(factor);
    }
}

void optimize(ISAM2 & isam2, NonlinearFactorGraph & graph, Values & values)
{
    // Perform smoothing update
    ISAM2Result result = isam2.update(graph, values);
    values = isam2.calculateEstimate();
    // std::cout << "SOFIYA! ESTIMATE: ";
    // values.print();

    // Sofiya: Not doing the following because we are only running the optimization once

    // // Remove the used up nodes
    // self.values_new.clear();

    // // Use the optimized bias to reset integration
    // if self
    //     .values_initial.exists(&Symbol::new (b 'b', self.ct_state + 1))
    //     {
    //         self.preint_gtsam.reset_integration_and_set_bias(
    //             &self.values_initial.get_constantbias(&Symbol::new (b 'b', self.ct_state + 1)).unwrap().into());
    //     }
    // else
    // {
    //     warn !("Bias wasn't optimized?");
    // }
}

int main(int argc, char** argv)
{

    // Create a factor graph
    NonlinearFactorGraph graph;
    PreintegratedCombinedMeasurements *preint_gtsam;
    Values values;

    initialize(graph, preint_gtsam, values);

    preintegrate(preint_gtsam);
    create_imu_factor(*preint_gtsam, graph);
    gtsam::State new_state = predict_state(*preint_gtsam, values, 0);

    values.insert(gtsam::Symbol('x', 1), new_state.pose_);
    values.insert(gtsam::Symbol('v', 1), new_state.velocity_);
    values.insert(gtsam::Symbol('b', 1), new_state.bias_);

    process_smart_features(graph);

    std::cout << "IMU POSE ESTIMATE: " << std::endl;
    std::cout << "\t Translation: " << new_state.pose_.translation().transpose() << std::endl;
    std::cout << "\t Rotation: " << new_state.pose_.rotation().toQuaternion() << std::endl;
    std::cout << "\t Velocity: " << new_state.velocity_.transpose() << std::endl;
    std::cout << "\t Bias: " << new_state.bias_ << std::endl;

    ISAM2 isam2 = ISAM2();
    optimize(isam2, graph, values);

    gtsam::Pose3 updated_pose = values.at(gtsam::Symbol('x', 1)).cast<gtsam::Pose3>();
    Vector3 updated_velocity = values.at(gtsam::Symbol('v', 1)).cast<Vector3>();
    imuBias::ConstantBias updated_bias = values.at(gtsam::Symbol('b', 1)).cast<imuBias::ConstantBias>();

    std::cout << "OPTIMIZATION!" << std::endl;
    std::cout << "\t Translation: " << updated_pose.translation().transpose() << std::endl;
    std::cout << "\t Rotation: " << updated_pose.rotation().toQuaternion() << std::endl;
    std::cout << "\t Velocity: " << updated_velocity.transpose() << std::endl;
    std::cout << "\t Bias: " << updated_bias << std::endl;

    return 0;
}
