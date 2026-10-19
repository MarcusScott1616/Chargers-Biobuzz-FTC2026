package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPod;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

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

                c.angleOffsetRad.set(Math.toRadians(0));
                c.podOffset.set(Vector2D.cartesian(-6, 6));
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

                c.angleOffsetRad.set(Math.toRadians(0));
                c.podOffset.set(Vector2D.cartesian(6, 6));

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

                c.angleOffsetRad.set(Math.toRadians(0));
                c.podOffset.set(Vector2D.cartesian(-6, -6));

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

                c.angleOffsetRad.set(Math.toRadians(0));
                c.podOffset.set(Vector2D.cartesian(6, -6));

            }
    );

    public static SwerveConfig driveConfig = new SwerveConfig(
            c -> {
                c.zeroPowerBehavior.set(SwerveConfig.ZeroPowerBehavior.X_LOCK);
                c.manualBrakeMode.set(true);
                c.voltageCompensation.set(false);
            }
    );

    public static Follower create(HardwareMap h) {
        CoaxialPod yellow = new CoaxialPod(h, yellowPod);
        CoaxialPod red = new CoaxialPod(h, redPod);
        CoaxialPod blue = new CoaxialPod(h, bluePod);
        CoaxialPod green = new CoaxialPod(h, greenPod);


        return new Follower(new PinpointLocalizer(h, new PinpointConfig() ), new Swerve(h, driveConfig,
                blue, yellow, green, red), new Foresight( foresightConfig ));

   // changed this up a bit, but it still has errors. unsure what it will turn out to be.

    }
}
