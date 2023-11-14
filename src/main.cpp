#include "main.h"

#include "odom.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

std::shared_ptr<pros::MotorGroup> leftMotors =
    std::make_shared<pros::MotorGroup>(std::initializer_list<pros::Motor> {
        pros::Motor(-19, pros::E_MOTOR_GEAR_BLUE),
        pros::Motor(-20, pros::E_MOTOR_GEAR_BLUE)
    });
std::shared_ptr<pros::MotorGroup> rightMotors =
    std::make_shared<pros::MotorGroup>(std::initializer_list<pros::Motor> {
        pros::Motor(11, pros::E_MOTOR_GEAR_BLUE),
        pros::Motor(12, pros::E_MOTOR_GEAR_BLUE)
    });
    

std::shared_ptr<pros::IMU> imu = std::make_shared<pros::IMU>(2);
std::shared_ptr<Gyro> gyro = std::make_shared<V5Gyro>(imu);

std::shared_ptr<TrackingWheel> leftTracker = std::make_shared<MotorTracker>(leftMotors, 3.25, 400, -6.1);
std::shared_ptr<TrackingWheel> rightTracker = std::make_shared<MotorTracker>(rightMotors, 3.25, 400, 6.1);

std::shared_ptr<pros::Rotation> rotation = std::make_shared<pros::Rotation>(3, true);

std::shared_ptr<TrackingWheel> horzTracker = std::make_shared<RotationTracker>(rotation, 2.744, 1, -2.2);

std::shared_ptr<Odom> odom = std::make_shared<TwoEncoderImuOdom>(rightTracker, horzTracker, gyro);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    odom->calibrate();
    odom->setPose({0, 0, 0});

    while (true) {
        odom->update();
        Pose currentPose = odom->getPose();
        pros::lcd::print(1, "X: %f", currentPose.x);
        pros::lcd::print(2, "Y: %f", currentPose.y);
        pros::lcd::print(3, "Theta: %f", currentPose.theta);
        pros::delay(20);
    }
    // pros::lcd::print(1, "Calibrating...");
    // gyro->calibrate();

    // constexpr float TURNS = 10;
    
    // float gyroPos;
    
    // leftMotors->move_velocity(50);
    // rightMotors->move_velocity(-50);

    // do {
    //     gyroPos = gyro->getRotation();
    //     pros::lcd::print(1, "Percent completed: %f", gyroPos / (360 * TURNS) * 100);
    //     pros::delay(20);
    // } while (gyroPos < 360 * TURNS);

    // leftMotors->move_velocity(0);
    // rightMotors->move_velocity(0);

    // const float POSITION_TO_RADIUS = gyro->getRotation() / 180 * M_PI;

    // pros::lcd::print(1, "%f", (leftTracker->getPosition() - rightTracker->getPosition()) / POSITION_TO_RADIUS); // track width
    // pros::lcd::print(2, "%f", horzTracker->getPosition() / POSITION_TO_RADIUS); // tracking point to horz tracker
}