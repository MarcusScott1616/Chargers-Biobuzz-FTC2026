package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {

    public static SwerveConfig driveConfig = new SwerveConfig(
            c -> {
                c.zeroPowerBehavior.set(SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
                c.manualBrakeMode.set(true);
                c.voltageCompensation.set(false);
            }
    );

    public static CoaxialPodConfig yellowPod = new CoaxialPodConfig(
            c -> {
                c.name.set("yellow");
                c.motorName.set("yellowMotor");
                c.servoName.set("yellowServo");
                c.servoEncoderName.set("yellowServoEncoder");
                c.turnController.set(Controller.pid(0.3, 0, 0.005)
                        .plus(Controller.proportionalFeedforward(0)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );

    public static CoaxialPodConfig redPod = new CoaxialPodConfig(
            c -> {
                c.name.set("red");
                c.motorName.set("redMotor");
                c.servoName.set("redServo");
                c.servoEncoderName.set("redServoEncoder");
                c.turnController.set(Controller.pid(0.3, 0, 0.005)
                        .plus(Controller.proportionalFeedforward(0)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );

    public static CoaxialPodConfig bluePod = new CoaxialPodConfig(
            c -> {
                c.name.set("blue");
                c.motorName.set("blueMotor");
                c.servoName.set("blueServo");
                c.servoEncoderName.set("blueServoEncoder");
                c.turnController.set(Controller.pid(0.3, 0, 0.005)
                        .plus(Controller.proportionalFeedforward(0)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );

    public static CoaxialPodConfig greenPod = new CoaxialPodConfig(
            c -> {
                c.name.set("green");
                c.motorName.set("greenMotor");
                c.servoName.set("greenServo");
                c.servoEncoderName.set("greenServo Encoder");
                c.turnController.set(Controller.pid(0.3, 0, 0.0086)
                        .plus(Controller.proportionalFeedforward(0)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );
}
