// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

// Opencv
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

#include "inputs.cpp"

using namespace std;
using namespace gtsam;

typedef std::vector<std::tuple<gtsam::Point2, int>> TrackedFeatures;

void initialize(NonlinearFactorGraph &graph, PreintegratedCombinedMeasurements *&preint, Values &values)
{
    // Create prior factor and add it to the graph

    // Sofiya: quaternion is wxyz here but xyzw in bauhaus
    gtsam::State prior_state = gtsam::State(INITIAL_POSE, INITIAL_VELOCITY, INITIAL_BIAS);

    std::cout << "INITIALIZATION!" << std::endl;
    std::cout << "\t Translation: " << INITIAL_POSE.translation().transpose() << std::endl;
    std::cout << "\t Rotation: " << INITIAL_POSE.rotation().toQuaternion() << std::endl;
    std::cout << "\t Velocity: " << INITIAL_VELOCITY.transpose() << std::endl;
    std::cout << "\t Bias: " << INITIAL_BIAS << std::endl;

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
    auto B_Rot_W = INITIAL_POSE.rotation().matrix();
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
        imu_bias_prior_noise));

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
    for (int i = 0; i < IMU_MEASUREMENTS.size(); i++)
    {
        preint->integrateMeasurement(IMU_MEASUREMENTS[i].measuredAcc, IMU_MEASUREMENTS[i].measuredOmega, IMU_MEASUREMENTS[i].dt);
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

void process_smart_features(
    NonlinearFactorGraph & graph,
    TrackedFeatures & features_f1,
    TrackedFeatures & features_f2)
{
    unordered_map<int, gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>> measurement_smart_lookup_left;

    // Sofiya: First frame's features. All features should be new
    for (int i = 0; i < features_f1.size(); i++)
    {
        // Create a smart factor for the new feature
        noiseModel::Isotropic::shared_ptr measurement_noise = noiseModel::Isotropic::Sigma(2, SIGMA_CAMERA, true); // sigma_camera
        gtsam::Cal3_S2 K = gtsam::Cal3_S2(FX, FY, S, CX, CY);

        // Note (frames): Transformation from camera frame to imu frame, i.e., pose of imu frame in camera frame
        SmartProjectionPoseFactor<gtsam::Cal3_S2> factor = SmartProjectionPoseFactor<gtsam::Cal3_S2>(
            measurement_noise,
            boost::make_shared<gtsam::Cal3_S2>(K), // calibration
            boost::optional<gtsam::Pose3>(tbc)); // body_P_sensor

        int feature_id = std::get<1>(features_f1[i]);
        gtsam::Point2 point = std::get<0>(features_f1[i]);

        // Insert measurements to a smart factor
        factor.add(point, gtsam::Symbol('x', 0));

        // Add smart factor to FORSTER2 model
        graph.add(factor);

        measurement_smart_lookup_left.insert({feature_id, factor}); // Store the factor for later use
    }

    // Sofiya: Matches to second frame's features. All features should match to the previous frame's
    for (int i = 0; i < features_f2.size(); i++)
    {
        int feature_id = std::get<1>(features_f2[i]);
        gtsam::Point2 point = std::get<0>(features_f2[i]);

        // Insert measurements to a smart factor
        auto factor = measurement_smart_lookup_left[feature_id];
        factor.add(point, gtsam::Symbol('x', 1)); // Add measurement
    }
}

void optimize(ISAM2 & isam2, NonlinearFactorGraph & graph, Values & values)
{
    // Perform smoothing update
    ISAM2Result result = isam2.update(graph, values);
    values = isam2.calculateEstimate();
}

std::tuple<TrackedFeatures, TrackedFeatures> optical_flow(cv::Mat &f1, cv::Mat &f2, bool should_visualize=false)
{
    // Good features to track
    vector<cv::Point2f> p1;
    cv::goodFeaturesToTrack(f1, p1, 200, 0.01, 10, cv::Mat(), 3, false, 0.04);
    TrackedFeatures features_f1;
    for (int i = 0; i < p1.size(); i++)
    {
        features_f1.push_back(std::make_tuple(gtsam::Point2(p1[i].x, p1[i].y), i));
        // std::cout << "gtsam::Point2(" << p1[i].x << ", " << p1[i].y << "), " << i << std::endl;
    }

    // Optical flow
    vector<cv::Point2f> p2;
    vector<uchar> status;
    vector<float> err;
    cv::TermCriteria criteria = cv::TermCriteria(3, 30, 0.01);
    cv::calcOpticalFlowPyrLK(f1, f2, p1, p2, status, err, cv::Size(21, 21), 4, criteria);

    TrackedFeatures features_f2;
    int total_tracked = 0;
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] == 1) {
            features_f2.push_back(std::make_tuple(gtsam::Point2(p2[i].x, p2[i].y), i));
            // std::cout << "gtsam::Point2(" << p2[i].x << ", " << p2[i].y << "), " << i << std::endl;
            total_tracked += 1;
        }
    }

    std::cout << "Optical flow total tracked features: " << total_tracked << std::endl;

    if (should_visualize)
    {
        vector<cv::Scalar> colors;
        cv::RNG rng;
        for (int i = 0; i < 100; i++)
        {
            int r = rng.uniform(0, 256);
            int g = rng.uniform(0, 256);
            int b = rng.uniform(0, 256);
            colors.emplace_back(r, g, b);
        }

        cv::Mat f2_rgb;
        cvtColor(f2, f2_rgb, cv::COLOR_BGR2RGB);

        cv::Mat mask = cv::Mat::zeros(f2_rgb.size(), f2_rgb.type());

        for (uint i = 0; i < p1.size(); i++)
        {
            if (status[i] == 1)
            {
                line(mask, p2[i], p1[i], colors[i], 2);
                circle(f2_rgb, p2[i], 2, colors[i], -1);
            }
        }

        cv::Mat img;
        cv::add(f2_rgb, mask, img);
        cv::imwrite("optical_flow.png", img);
    }

    return std::make_tuple(features_f1, features_f2);
}

cv::Mat read_image_and_resize(const std::string &image_path)
{
    cv::Mat im = imread(image_path, cv::IMREAD_UNCHANGED);
    cv::Mat im_resize;
    cv::resize(im, im_resize, cv::Size(600, 350));
    return im_resize;
}

int main(int argc, char** argv)
{
    // Initialize factor graph
    NonlinearFactorGraph graph;
    PreintegratedCombinedMeasurements *preint_gtsam;
    Values values;
    initialize(graph, preint_gtsam, values);

    // IMU
    preintegrate(preint_gtsam);
    create_imu_factor(*preint_gtsam, graph);
    gtsam::State new_state = predict_state(*preint_gtsam, values, 0);

    values.insert(gtsam::Symbol('x', 1), new_state.pose_);
    values.insert(gtsam::Symbol('v', 1), new_state.velocity_);
    values.insert(gtsam::Symbol('b', 1), new_state.bias_);

    std::cout << "IMU POSE ESTIMATE: " << std::endl;
    std::cout << "\t Translation: " << new_state.pose_.translation().transpose() << std::endl;
    std::cout << "\t Rotation: " << new_state.pose_.rotation().toQuaternion() << std::endl;
    std::cout << "\t Velocity: " << new_state.velocity_.transpose() << std::endl;
    std::cout << "\t Bias: " << new_state.bias_ << std::endl;

    // Vision
    // Run with the optical flow values from Bauhaus
    // process_smart_features(graph, FEATURES_F1, FEATURES_F2);
    // Or run with optical flow performed here
    cv::Mat f1 = read_image_and_resize(IMAGE1_PATH);
    cv::Mat f2 = read_image_and_resize(IMAGE2_PATH);
    std::tuple<TrackedFeatures, TrackedFeatures> feature_tracks = optical_flow(f1, f2, true);
    TrackedFeatures features_f1 = std::get<0>(feature_tracks);
    TrackedFeatures features_f2 = std::get<1>(feature_tracks);
    process_smart_features(graph, features_f1, features_f1);

    // Optimization
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
