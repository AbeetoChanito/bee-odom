#include "main.h"

#include "control.hpp"
#include "odom.hpp"

#include "Graphy/Grapher.hpp"
#include "okapi/api/units/QTime.hpp"
#include "pros/colors.h"

#include <memory>

pros::Controller controller(pros::E_CONTROLLER_MASTER);

std::shared_ptr<pros::MotorGroup> leftMotors = std::make_shared<pros::MotorGroup>(std::initializer_list<pros::Motor> {
    pros::Motor(-19, pros::E_MOTOR_GEAR_BLUE), pros::Motor(-20, pros::E_MOTOR_GEAR_BLUE)});
std::shared_ptr<pros::MotorGroup> rightMotors = std::make_shared<pros::MotorGroup>(std::initializer_list<pros::Motor> {
    pros::Motor(11, pros::E_MOTOR_GEAR_BLUE), pros::Motor(12, pros::E_MOTOR_GEAR_BLUE)});

std::shared_ptr<pros::IMU> imu = std::make_shared<pros::IMU>(2);
std::shared_ptr<Gyro> gyro = std::make_shared<V5Gyro>(imu);

constexpr float HALF_TRACK = 11.725 / 2;

std::shared_ptr<TrackingWheel> leftTracker = std::make_shared<MotorTracker>(leftMotors, 3.25, 400, -HALF_TRACK);
std::shared_ptr<TrackingWheel> rightTracker = std::make_shared<MotorTracker>(rightMotors, 3.25, 400, HALF_TRACK);

std::shared_ptr<pros::Rotation> rotation = std::make_shared<pros::Rotation>(3, true);

std::shared_ptr<TrackingWheel> horzTracker = std::make_shared<RotationTracker>(rotation, 2.744, 1, -8.476);

std::shared_ptr<Odom> odom = std::make_shared<TwoEncoderImuOdom>(rightTracker, horzTracker, gyro);

std::unique_ptr<TimeSettler> lateralTimeSettler = std::make_unique<TimeSettler>(0, 0, 0, 0, 10000);
std::unique_ptr<PID> lateralPID = std::make_unique<PID>(0, 0, 0, 0, std::move(lateralTimeSettler));

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() { pros::lcd::initialize(); }

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
void autonomous() {}

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
    graphy::AsyncGrapher grapher("Lateral PID");
    
    grapher.setRefreshRate(10 * okapi::millisecond);
    grapher.addDataType("Target", COLOR_ORANGE);
    grapher.addDataType("Actual", COLOR_AQUAMARINE);
    grapher.startTask();

    leftTracker->tare();
    rightTracker->tare();

    float target = 24;

    while (true) {
        std::optional<float> pidOutput = lateralPID->update(target - (leftTracker->getPosition() + rightTracker->getPosition()) / 2);

        if (!pidOutput.has_value()) {
            break;
        }

        grapher.update("Target", target);
        grapher.update("Actual", pidOutput.value());

        leftMotors->move(pidOutput.value());
        rightMotors->move(pidOutput.value());
        pros::delay(10);
    }

    leftMotors->move(0);
    rightMotors->move(0);
}