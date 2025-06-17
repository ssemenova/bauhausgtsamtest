// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>

// Opencv
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

#include "inputs.cpp"

using namespace std;
using namespace gtsam;

void initialize(NonlinearFactorGraph &graph, PreintegratedCombinedMeasurements *&preint, Values &current_values, Values &all_values)
{
    // const gtsam::Rot3 huh = gtsam::Rot3(0.5308, -0.1365, -0.8329, -0.0761);
    // const gtsam::Point3 huh2 = gtsam::Point3(4.6331, -1.8072, 0.8306);

    // std::cout << "huh: " << huh << std::endl;
    // std::cout << "huh2: " << huh2 << std::endl;

    // Sofiya: quaternion is wxyz here but xyzw in bauhaus
    gtsam::State prior_state = gtsam::State(INITIAL_POSE, INITIAL_VELOCITY, INITIAL_BIAS);

    std::cout << "Initialization:" << std::endl;
    std::cout << "\t Translation: " << INITIAL_POSE.translation().transpose() << std::endl;
    std::cout << "\t Rotation matrix: " << INITIAL_POSE.rotation() << std::endl;
    std::cout << "\t Rotation quaternion: " << INITIAL_POSE.rotation().toQuaternion() << std::endl;
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

    // std::cout << "Pose prior covariance: " << pose_prior_covariance << std::endl;
    // std::cout << " INITIAL_ROLL_PITCH_SIGMA: " << INITIAL_ROLL_PITCH_SIGMA << std::endl;
    // std::cout << " INITIAL_YAW_SIGMA: " << INITIAL_YAW_SIGMA << std::endl;
    // std::cout << " INITIAL_POSITION_SIGMA: " << INITIAL_POSITION_SIGMA << std::endl;
    // std::cout << " INITIAL_VELOCITY_SIGMA: " << INITIAL_VELOCITY_SIGMA << std::endl;

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
    current_values.insert(gtsam::Symbol('x', 0), prior_state.pose_);
    current_values.insert(gtsam::Symbol('v', 0), prior_state.velocity_);
    current_values.insert(gtsam::Symbol('b', 0), prior_state.bias_);
    all_values.insert(gtsam::Symbol('x', 0), prior_state.pose_);
    all_values.insert(gtsam::Symbol('v', 0), prior_state.velocity_);
    all_values.insert(gtsam::Symbol('b', 0), prior_state.bias_);

    // Create GTSAM preintegration parameters for use with Foster's version
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
        params;
    params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(); // Z-up navigation frame: gravity points along negative Z-axis !!!

    params->setGyroscopeCovariance(gtsam::I_3x3 * GYRO_NOISE_DENSITY * GYRO_NOISE_DENSITY);
    std::cout << "set_gyroscope_covariance: " << GYRO_NOISE_DENSITY * GYRO_NOISE_DENSITY << std::endl;
    params->setAccelerometerCovariance(gtsam::I_3x3 * ACCEL_NOISE_DENSITY * ACCEL_NOISE_DENSITY);
    std::cout << "set_accelerometer_covariance: " << ACCEL_NOISE_DENSITY * ACCEL_NOISE_DENSITY << std::endl;
    params->setIntegrationCovariance(gtsam::I_3x3 * IMU_INTEGRATION_SIGMA * IMU_INTEGRATION_SIGMA);
    std::cout << "set_integration_covariance: " << IMU_INTEGRATION_SIGMA * IMU_INTEGRATION_SIGMA << std::endl;
    params->biasAccOmegaInt = IMU_BIAS_INIT_SIGMA * gtsam::Matrix66::Identity(6, 6);
    std::cout << "bias_acc_omega_int: " << IMU_BIAS_INIT_SIGMA << std::endl;
    params->biasAccCovariance = ACCEL_RANDOM_WALK * ACCEL_RANDOM_WALK * gtsam::Matrix33::Identity(3, 3);
    std::cout << "bias_acc_covariance: " << ACCEL_RANDOM_WALK * ACCEL_RANDOM_WALK << std::endl;
    params->biasOmegaCovariance = GYRO_RANDOM_WALK * GYRO_RANDOM_WALK * gtsam::Matrix33::Identity(3, 3); // gyroscope_random_walk
    std::cout << "bias_omega_covariance: " << GYRO_RANDOM_WALK * GYRO_RANDOM_WALK << std::endl;

    std::cout << "Params: " << params << std::endl;
    // Actually create the GTSAM preintegration
    preint = new gtsam::PreintegratedCombinedMeasurements(params, prior_state.bias_);
}

gtsam::State predict_state(PreintegratedCombinedMeasurements &preint, Values &values, const int state_id)
{
    // Get the current state (t=k)
    gtsam::State state_k = gtsam::State(
        values.at(gtsam::Symbol('x', state_id)).cast<Pose3>(),
        values.at(gtsam::Symbol('v', state_id)).cast<Vector3>(),
        values.at(gtsam::Symbol('b', state_id)).cast<imuBias::ConstantBias>());

    std::cout << "Current state used for imu-based prediction: " << std::endl;
    std::cout << "Pose: " << state_k.pose_ << std::endl;
    std::cout << "Velocity: " << state_k.velocity_ << std::endl;
    std::cout << "Bias: " << state_k.bias_ << std::endl;

    // From this we should predict where we will be at the next time (t=K+1)
    NavState state_k1 = preint.predict(
        gtsam::NavState(
            state_k.pose_,
            state_k.velocity_),
        state_k.bias_);

    std::cout << "C++ predicted state: " << state_k1.t() << std::endl;
    std::cout << "C++ predicted velocity: " << state_k1.velocity().transpose() << std::endl;

    gtsam::State predicted = gtsam::State(
        state_k1.pose(),
        state_k1.velocity(),
        state_k.bias_);

    return predicted;
}

void preintegrate(PreintegratedCombinedMeasurements *&preint, vector<gtsam::ImuMeasurement> &imu_msmts)
{
    for (int i = 0; i < imu_msmts.size(); i++)
    {
        preint->integrateMeasurement(imu_msmts[i].measuredAcc, imu_msmts[i].measuredOmega, imu_msmts[i].dt);

        preint->print("Preint meas cov: ");
        std::cout << " Preintegrating " << i << " measurement: "
                  << " Acc: " << imu_msmts[i].measuredAcc.transpose()
                  << ", Omega: " << imu_msmts[i].measuredOmega.transpose()
                  << ", dt: " << imu_msmts[i].dt << std::endl;
    }
}

void create_imu_factor(PreintegratedCombinedMeasurements & preint, NonlinearFactorGraph & graph, int state_id)
{
    CombinedImuFactor imu_factor = CombinedImuFactor(
        gtsam::Symbol('x', state_id),     // pose at time t
        gtsam::Symbol('v', state_id),     // velocity at time t
        gtsam::Symbol('x', state_id + 1), // pose at time t+1
        gtsam::Symbol('v', state_id + 1), // velocity at time t+1
        gtsam::Symbol('b', state_id),     // bias at time t
        gtsam::Symbol('b', state_id + 1),            // bias at time t+1
        preint);
    graph.add(imu_factor);
}

void process_smart_features(
    NonlinearFactorGraph &graph,
    const TrackedFeatures &features,
    unordered_map<int, SmartFactor::shared_ptr> &smart_factors_lookup,
    const int state_id)
{
    for (int i = 0; i < features.size(); i++)
    {
        gtsam::Point2 point = std::get<0>(features[i]);
        int feature_id = std::get<1>(features[i]);
        // std::cout << "Processing feature id: " << feature_id << " at point: " << point.transpose() << std::endl;

        if (smart_factors_lookup.find(feature_id) != smart_factors_lookup.end())
        {
            // Insert measurements to a smart factor
            auto factor = smart_factors_lookup[feature_id];
            factor->add(point, gtsam::Symbol('x', state_id)); // Add measurement
            std::cout << "Added measurement to existing smart factor for feature id: " << feature_id << std::endl;
        } 
        else
        {
            // If we know it is not in the graph
            // Create a smart factor for the new feature
            noiseModel::Isotropic::shared_ptr measurement_noise = noiseModel::Isotropic::Sigma(2, SIGMA_CAMERA, true);
            gtsam::Cal3_S2 K = gtsam::Cal3_S2(FX, FY, S, CX, CY);

            // Note (frames): Transformation from camera frame to imu frame, i.e., pose of imu frame in camera frame
            SmartFactor::shared_ptr factor(new SmartFactor(
                measurement_noise,
                boost::make_shared<gtsam::Cal3_S2>(K), // calibration
                boost::optional<gtsam::Pose3>(TBC))); // body_P_sensor);

            // Insert measurements to a smart factor
            factor->add(point, gtsam::Symbol('x', state_id));

            // Add smart factor to FORSTER2 model
            graph.add(factor);

            smart_factors_lookup.insert({feature_id, factor}); // Store the factor for later use

            std::cout << "Added new smart factor for feature id: " << feature_id << std::endl;
        }
    }
}

void extract_features(const cv::Mat frame, TrackedFeatures &features)
{
    int num_features_to_find = 200 - features.size();
    if (num_features_to_find <= 0)
    {
        std::cout << "Have enough features, not extracting more." << std::endl;
        return;
    }

    cv::Mat mask = cv::Mat(frame.rows, frame.cols, CV_8U, cv::Scalar(255));
    for (int i = 0; i < features.size(); i++)
    {
        cv::Point p1 = cv::Point(std::get<0>(features[i]).x(), std::get<0>(features[i]).y());
        cv::circle(mask, p1, 20, cv::Scalar(0.0), -1, 8, 0);
    }

    vector<cv::Point2f> points;
    cv::goodFeaturesToTrack(frame, points, num_features_to_find, 0.01, 7, mask, 7, false, 0.04);

    int last_tracked_features_id;
    if (!features.empty())
    {
        last_tracked_features_id = std::get<1>(features.back()) + 1;
    } else {
        last_tracked_features_id = 0;
    }

    std::cout << "Extracted features: " << points.size() << std::endl;
    for (int i = 0; i < points.size(); i++)
    {
        features.push_back(std::make_tuple(gtsam::Point2(points[i].x, points[i].y), last_tracked_features_id + i));
        std::cout << "(" << points[i].x << ", " << points[i].y << ", " << last_tracked_features_id + i << "), ";
    }
    std::cout << std::endl;
}

TrackedFeatures optical_flow(const cv::Mat &frame1, const cv::Mat &frame2, const TrackedFeatures features1, bool should_visualize=false)
{
    vector<cv::Point2f> p1 = get_tracked_features_as_vector_of_point2f(features1);

    vector<cv::Point2f> p2;
    vector<uchar> status;
    vector<float> err;
    cv::TermCriteria criteria = cv::TermCriteria(3, 30, 0.01);
    cv::calcOpticalFlowPyrLK(frame1, frame2, p1, p2, status, err, cv::Size(21, 21), 4, criteria);

    TrackedFeatures features2;
    int total_tracked = 0;
    std::cout << "Tracked features: " << std::endl;
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] == 1) {
            features2.push_back(std::make_tuple(gtsam::Point2(p2[i].x, p2[i].y), i));
            // std::cout << "gtsam::Point2(" << p2[i].x << ", " << p2[i].y << "), " << i << std::endl;
            total_tracked += 1;
            std::cout << "(" << p2[i].x << ", " << p2[i].y << ", " << i << "), ";
        }
    }
    std::cout << std::endl;

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

        cv::Mat frame2_rgb;
        cvtColor(frame2, frame2_rgb, cv::COLOR_BGR2RGB);

        cv::Mat mask = cv::Mat::zeros(frame2_rgb.size(), frame2_rgb.type());

        for (uint i = 0; i < p1.size(); i++)
        {
            if (status[i] == 1)
            {
                line(mask, p2[i], p1[i], colors[i], 2);
                circle(frame2_rgb, p2[i], 2, colors[i], -1);
            }
        }

        cv::Mat img;
        cv::add(frame2_rgb, mask, img);
        cv::imwrite("optical_flow.png", img);
    }

    return features2;
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
    NonlinearFactorGraph graph;
    PreintegratedCombinedMeasurements *preint_gtsam;
    Values values; // deleted after every iteration
    Values all_values; // all values stored here
    ISAM2 isam2 = ISAM2();

    vector<string> image_paths = {IMAGE1_PATH, IMAGE2_PATH, IMAGE3_PATH};
    vector<vector<gtsam::ImuMeasurement>> all_imu_measurements = {IMU_MEASUREMENTS1, IMU_MEASUREMENTS2};

    // Initialize 
    initialize(graph, preint_gtsam, values, all_values);
    cv::Mat frame1 = read_image_and_resize(image_paths[0]);
    TrackedFeatures features1;
    extract_features(frame1, features1);
    unordered_map<int, SmartFactor::shared_ptr> smart_factors_lookup = {};
    process_smart_features(graph, features1, smart_factors_lookup, 0);

    // print_features(features1);

    for (int i = 0; i < image_paths.size() - 1; i++)
    {
        std::cout << "\n \nWORKING ON IMAGE " << i << std::endl;

        string image_path2 = image_paths[i + 1];

        //****** IMU */
        preintegrate(preint_gtsam, all_imu_measurements[i]);
        create_imu_factor(*preint_gtsam, graph, i);
        gtsam::State new_state = predict_state(*preint_gtsam, all_values, i);

        values.insert(gtsam::Symbol('x', i + 1), new_state.pose_);
        values.insert(gtsam::Symbol('v', i + 1), new_state.velocity_);
        values.insert(gtsam::Symbol('b', i + 1), new_state.bias_);
        all_values.insert(gtsam::Symbol('x', i + 1), new_state.pose_);
        all_values.insert(gtsam::Symbol('v', i + 1), new_state.velocity_);
        all_values.insert(gtsam::Symbol('b', i + 1), new_state.bias_);

        std::cout << "IMU pose estimate: " << std::endl;
        std::cout << "\t Translation: " << new_state.pose_.translation().transpose() << std::endl;
        std::cout << "\t Rotation: " << new_state.pose_.rotation() << std::endl;

        std::cout << "\t Velocity: " << new_state.velocity_.transpose() << std::endl;
        std::cout << "\t Bias: " << new_state.bias_ << std::endl;

        //****** Vision */
        cv::Mat frame2 = read_image_and_resize(image_path2);

        // Optical flow
        TrackedFeatures features2 = optical_flow(frame1, frame2, features1, true);
        // Feature extraction
        extract_features(frame2, features2);

        std::cout << "Features2:";
        print_features(features2);

        process_smart_features(graph, features2, smart_factors_lookup, i + 1);

        // print_features(features2);
        // print_features(features2);

        //****** Optimization */
        std::cout << "Before update, isam2 has: ";
        isam2.getLinearizationPoint().print();

        std::cout << "Initial values: ";
        values.print();
        std::cout << std::endl << "Graph: ";
        graph.print();

        ISAM2Result result = isam2.update(graph, values);
        std::cout << "After update, isam2 has: ";
        isam2.getLinearizationPoint().print();

        values = isam2.calculateEstimate();
        std::cout << "SOFIYA! ESTIMATE: ";
        values.print();

        gtsam::Pose3 updated_pose = values.at(gtsam::Symbol('x', i + 1)).cast<gtsam::Pose3>();
        Vector3 updated_velocity = values.at(gtsam::Symbol('v', i + 1)).cast<Vector3>();
        imuBias::ConstantBias updated_bias = values.at(gtsam::Symbol('b', i + 1)).cast<imuBias::ConstantBias>();

        std::cout << "Optimization:" << std::endl;
        std::cout << "\t Translation: " << updated_pose.translation().transpose() << std::endl;
        std::cout << "\t Rotation: " << updated_pose.rotation().toQuaternion() << std::endl;
        std::cout << "\t Velocity: " << updated_velocity.transpose() << std::endl;
        std::cout << "\t Bias: " << updated_bias << std::endl;

        // std::cout << "After update, isam2 has: ";
        // isam2.getLinearizationPoint().print();

        // Clear values for next iteration
        values.clear();
        // graph.resize(0);
        // smart_factors_lookup = {};

        // Update previous frame data for next iteration
        features1 = features2;
        frame1 = frame2;
    }

    return 0;
}
