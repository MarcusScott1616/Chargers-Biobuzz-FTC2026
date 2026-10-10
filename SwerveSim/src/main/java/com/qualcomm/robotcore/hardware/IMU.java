package com.qualcomm.robotcore.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * FAKE IMU (gyro) for SwerveSim. The simulator tells it which way the robot is
 * facing, and SwerveDrive reads it back as yaw - just like the real gyro.
 * Yaw is counter-clockwise positive, same as the real FTC IMU.
 */
public class IMU {
    public static class Parameters {
        public final RevHubOrientationOnRobot imuOrientationOnRobot;

        public Parameters(RevHubOrientationOnRobot imuOrientationOnRobot) {
            this.imuOrientationOnRobot = imuOrientationOnRobot;
        }
    }

    private double headingRadians = 0; // true heading, set by the simulator
    private double yawZeroRadians = 0; // where resetYaw() was last called

    public boolean initialize(Parameters parameters) { return true; }

    public void resetYaw() { yawZeroRadians = headingRadians; }

    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        double yaw = Math.atan2(Math.sin(headingRadians - yawZeroRadians), Math.cos(headingRadians - yawZeroRadians));
        return new YawPitchRollAngles(AngleUnit.RADIANS, yaw, 0, 0, System.nanoTime());
    }

    /** Simulator only: tell the fake gyro which way the robot is facing. */
    public void simSetHeading(double radians) { headingRadians = radians; }
}
