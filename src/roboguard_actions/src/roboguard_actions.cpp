#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "control_msgs/msg/joint_jog.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/bool.hpp"
#include "capra_control_msgs/msg/flippers.hpp"
#include "capra_control_msgs/msg/tracks.hpp"

using namespace std::chrono_literals;

#define RAD2REV(rad) (rad / M_2_PI)
#define REV2RAD(rev) (rev * M_2_PI)

#define FLIPPER_MOVE_OFFSET REV2RAD(50)

class StateBool
{
private:
    bool _state = false;
    bool _lastState = false;
public:
    StateBool(bool state = false) : _state{state} {}
    operator bool() const { return _state; }
    StateBool& operator=(bool state) 
    {
        _lastState = _state;
        _state = state;
        return *this;
    }
    bool latched() const { return !_lastState && _state; }
    bool unlatched() const { return _lastState && !_state; }
};

struct DriveState
{
    double position;
    double setPosition;
    bool initialized = false;
};

using DriveStatePtr = DriveState*;

class Instruction
{
protected:
    DriveStatePtr* drives;
    size_t count;
    double offset;
    double setOffset;
public:
    bool isInitialized() const 
    {
        for (size_t i = 0; i < count; ++i)
        {
            if (!drives[i]->initialized) return false;
        }
        return true;
    }
    double getOffset() const { return offset; }
    double getSetOffset() const { return setOffset; }
    virtual void update() = 0;
    virtual void compute(double command, bool enabled) = 0;
};

class SingleInstruction : public Instruction
{
private:
    StateBool singleControl{};
public:
    SingleInstruction(DriveState& drive)
    {
        drives = new DriveStatePtr[1] {&drive};
        count = 1;
    }

    void update() override
    {
        if (!isInitialized()) return;
        offset = drives[0]->position - (drives[0]->setPosition - setOffset);
    }

    void compute(double command, bool enabled) override
    {
        if (!isInitialized()) return;
        singleControl = command != 0 && enabled;

        if (singleControl)
        {
            setOffset = offset + (command > 0 ? FLIPPER_MOVE_OFFSET : -FLIPPER_MOVE_OFFSET);
        }
        else if (singleControl.unlatched())
        {
            setOffset = offset;
        }
    }
};

class PairInstruction : public Instruction
{
private:
    StateBool pairControl{};
public:
    PairInstruction(DriveState& left, DriveState& right)
    {
        drives = new DriveStatePtr[2] {&left, &right};
        count = 2;
    }

    void update() override
    {
        if (!isInitialized()) return;
        double cumul = 0.0;
        for (size_t i = 0; i < count; ++i)
        {
            cumul += drives[i]->position - (drives[i]->setPosition - setOffset);
        }
        offset = cumul / count;
    }

    void compute(double command, bool enabled) override
    {
        if (!isInitialized()) return;
        pairControl = command != 0 && enabled;

        if (pairControl)
        {
            setOffset = offset + (command > 0 ? FLIPPER_MOVE_OFFSET : -FLIPPER_MOVE_OFFSET);
        }
        else if (pairControl.unlatched())
        {
            setOffset = offset;
        }
    }
};

class AllInstruction : public Instruction
{
private:
    StateBool allControl{};
public:
    AllInstruction(DriveState& frontLeft, DriveState& rearLeft, DriveState& frontRight, DriveState& rearRight)
    {
        drives = new DriveStatePtr[4] {&frontLeft, &rearLeft, &frontRight, &rearRight};
        count = 4;
    }

    void update() override
    {
        if (!isInitialized()) return;
        double cumul = 0.0;
        for (size_t i = 0; i < count; ++i)
        {
            cumul += drives[i]->position - (drives[i]->setPosition - setOffset);
        }
        offset = cumul / count;
    }

    void compute(double command, bool enabled) override
    {
        if (!isInitialized()) return;
        allControl = command != 0 && enabled;

        if (allControl)
        {
            setOffset = offset + (command > 0 ? FLIPPER_MOVE_OFFSET : -FLIPPER_MOVE_OFFSET);
        }
        else if (allControl.unlatched())
        {
            setOffset = offset;
        }
    }
};

class ActionsNode : public rclcpp::Node
{
public:
    using JointJog = control_msgs::msg::JointJog;
    using JointState = sensor_msgs::msg::JointState;
    using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
    using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
    using Header = std_msgs::msg::Header;
    using Bool = std_msgs::msg::Bool;
    using Flippers = capra_control_msgs::msg::Flippers;
    using Tracks = capra_control_msgs::msg::Tracks;

    ActionsNode()
        : Node("roboguard_actions")
    {
        // Declare parameters
        leftJointNames = this->declare_parameter<std::vector<std::string>>("tracks.left_joint_names");
        rightJointNames = this->declare_parameter<std::vector<std::string>>("tracks.right_joint_names");
        wheelRadius = this->declare_parameter<double>("tracks.wheel_radius", 1.0);
        wheelSeparation = this->declare_parameter<double>("tracks.wheel_separation", 1.0);
        tracksGearRatio = this->declare_parameter<double>("tracks.gear_ratio", 1.0);
        tracksMaxSpeed = this->declare_parameter<double>("tracks.max_speed", 1.0);

        flipperFrontLeft = this->declare_parameter<std::string>("flippers.front_left_name");
        flipperRearLeft = this->declare_parameter<std::string>("flippers.rear_left_name");
        flipperFrontRight = this->declare_parameter<std::string>("flippers.front_right_name");
        flipperRearRight = this->declare_parameter<std::string>("flippers.rear_right_name");
        flipperGearRatio = this->declare_parameter<double>("flippers.gear_ratio", 1.0);
        flipperMaxSpeed = this->declare_parameter<double>("flippers.max_speed", 1.0);

        updateRate = this->declare_parameter<double>("diff_drive.update_rate");
        odomFrameID = this->declare_parameter<std::string>("diff_drive.odom_frame_id");
        baseFrameID = this->declare_parameter<std::string>("diff_drive.base_frame_id");
        poseCovarianceDiagonal = this->declare_parameter<std::vector<double>>("diff_drive.pose_covariance_diagonal");
        twistCovarianceDiagonal = this->declare_parameter<std::vector<double>>("diff_drive.twist_covariance_diagonal");
        openLoop = this->declare_parameter<bool>("diff_drive.open_loop");
        enableOdomTF = this->declare_parameter<bool>("diff_drive.enable_odom_tf");

        // Create subcriptions
        enableSub = this->create_subscription<Bool>(
            "enable", 1, std::bind(&ActionsNode::onEnable, this, std::placeholders::_1));
        tracksSub = this->create_subscription<Tracks>(
            "tracks_cmd", 1, std::bind(&ActionsNode::onTracks, this, std::placeholders::_1));
        flippersSub = this->create_subscription<Flippers>(
            "flippers_cmd", 1, std::bind(&ActionsNode::onFlippers, this, std::placeholders::_1));
        jointStateSub = this->create_subscription<JointState>(
            "joint_state", 1, std::bind(&ActionsNode::onJointState, this, std::placeholders::_1));

        // Create publishers
        jointTrajectoryPub = this->create_publisher<JointTrajectory>("trajectory", 1);
        jointJogPub = this->create_publisher<JointJog>("jog", 1);

        // Create drives
        drives = new DriveState[2] {};

        // Create instructions
        frontLeftFlipperInstruction = new SingleInstruction(drives[0]);
        frontRightFlipperInstruction = new SingleInstruction(drives[1]);
        frontPairInstruction = new PairInstruction(drives[0], drives[1]);
    }

private:
    double mps2tracks(float speed)
    {
        return speed / wheelRadius / tracksGearRatio;
    }

    void onEnable(Bool::SharedPtr enable)
    {
        this->enable = enable->data;
    }

    void onTracks(Tracks::SharedPtr tracks)
    {
        JointJog res{};
        res.header.frame_id = baseFrameID;
        res.header.stamp = get_clock()->now();
        res.duration = 1.0;
        res.joint_names.resize(leftJointNames.size() + rightJointNames.size(), "");
        res.displacements.resize(res.joint_names.size(), 0.0);
        res.velocities.resize(res.joint_names.size(), 0.0);

        for (size_t i = 0; i < leftJointNames.size(); ++i)
        {
            res.joint_names[i] = leftJointNames[i];
            res.velocities[i] = mps2tracks(tracks->left);
        }
        for (size_t i = leftJointNames.size(); i < res.joint_names.size(); ++i)
        {
            res.joint_names[i] = rightJointNames[i];
            res.velocities[i] = mps2tracks(tracks->right);
        }

        jointJogPub->publish(res);
    }

    float clamp(float value) { return std::min(1.0f, std::max(-1.0f, value)); }

    void onFlippers(Flippers::SharedPtr flippers)
    {
        // Update instructions
        frontLeftFlipperInstruction->update();
        frontRightFlipperInstruction->update();
        frontPairInstruction->update();

        // Clamp commands
        float fl = clamp(flippers->front_left);
        float fr = clamp(flippers->front_right);

        // Find selected
        bool frontSelected = fl != 0 && fr != 0;
        bool frontLeftSelected = !frontSelected && fl != 0;
        bool frontRightSelected = !frontSelected && fr != 0;

        // Compute
        frontLeftFlipperInstruction->compute(fl, frontLeftSelected);
        frontRightFlipperInstruction->compute(fr, frontRightSelected);
        frontPairInstruction->compute((fl + fr) /2.0, frontSelected);

        // Publish
        JointTrajectory traj{};
        traj.header.frame_id = baseFrameID;
        traj.header.stamp = get_clock()->now();
        traj.joint_names = {flipperFrontLeft, flipperFrontRight};
        JointTrajectoryPoint point{};
        point.positions = {
            frontLeftFlipperInstruction->getSetOffset() + frontPairInstruction->getSetOffset(),
            frontRightFlipperInstruction->getSetOffset() + frontPairInstruction->getSetOffset(),
        };
        point.velocities = {
            flipperMaxSpeed * std::abs(fl),
            flipperMaxSpeed * std::abs(fr),
        };
        point.accelerations = {0, 0};
        point.effort = {0, 0};

        traj.points.push_back(point);
        
        jointTrajectoryPub->publish(traj);
    }

    void onJointState(JointState::SharedPtr state)
    {
        for (size_t i = 0; i < state->name.size(); ++i)
        {
            if (state->name[i] == flipperFrontLeft)
            {
                drives[0].initialized = true;
                drives[0].position = state->position[i];
            }
            if (state->name[i] == flipperFrontRight)
            {
                drives[1].initialized = true;
                drives[1].position = state->position[i];
            }
        }
    }

    bool enable{};
    DriveState* drives{};

    // Parameters
    // Tracks
    std::vector<std::string> leftJointNames{};
    std::vector<std::string> rightJointNames{};
    double wheelRadius{};
    double wheelSeparation{};
    double tracksGearRatio{};
    double tracksMaxSpeed{};

    // Flippers
    std::string flipperFrontLeft{};
    std::string flipperRearLeft{};
    std::string flipperFrontRight{};
    std::string flipperRearRight{};
    double flipperGearRatio{};
    double flipperMaxSpeed{};

    // Diff drive
    double updateRate{};
    std::string odomFrameID{};
    std::string baseFrameID{};
    std::vector<double> poseCovarianceDiagonal{};
    std::vector<double> twistCovarianceDiagonal{};
    bool openLoop{};
    bool enableOdomTF{};

    // Instructions
    SingleInstruction* frontLeftFlipperInstruction{};
    SingleInstruction* frontRightFlipperInstruction{};
    PairInstruction* frontPairInstruction{};

    rclcpp::TimerBase::SharedPtr timer;

    rclcpp::Subscription<Bool>::SharedPtr enableSub;
    rclcpp::Subscription<Flippers>::SharedPtr flippersSub;
    rclcpp::Subscription<Tracks>::SharedPtr tracksSub;
    rclcpp::Subscription<JointState>::SharedPtr jointStateSub;

    rclcpp::Publisher<JointTrajectory>::SharedPtr jointTrajectoryPub;
    rclcpp::Publisher<JointJog>::SharedPtr jointJogPub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionsNode>());
    rclcpp::shutdown();
    return 0;
}