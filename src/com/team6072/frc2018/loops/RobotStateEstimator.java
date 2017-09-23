package com.team6072.frc2018.loops;

import com.team6072.frc2018.Kinematics;
import com.team6072.frc2018.RobotState;
import com.team6072.frc2018.subsystems.Drive;
import com.team6072.lib.util.math.Rotation2d;
import com.team6072.lib.util.math.Twist2d;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimator implements Loop {
    static RobotStateEstimator instance_ = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    RobotStateEstimator() {
    }

    RobotState robot_state_ = RobotState.getInstance();
    Drive drive_ = Drive.getInstance();
    double left_encoder_prev_distance_ = 0;
    double right_encoder_prev_distance_ = 0;

    @Override
    public synchronized void onStart(double timestamp) {

    }

    @Override
    public synchronized void onLoop(double timestamp) {
        final Rotation2d gyro_angle = drive_.getGyroAngle();
    }

    @Override
    public void onStop(double timestamp) {
        // no-op
    }

}
