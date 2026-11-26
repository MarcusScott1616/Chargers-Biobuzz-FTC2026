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
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SwerveCode.SwerveDriveV2;
import java.util.List;

public class Constants {

    // Odometry pod offsets. Pinpoint takes doubles, so these are doubles too.
    public static double xOffset = 4; //Odometry pods.
    // Distance from Center point in mm. To the Left would be positive for X offset
    public static double yOffset = 4; //Odometry pods.
    // Distance from Center point in mm. To the Front would be positive for Y offset.

    public static CoaxialPodConfig yellowPod = new CoaxialPodConfig(
            c -> {
                c.name.set("yellow");
                c.motorName.set("yellowMotor");
                c.servoName.set("yellowServo");
                c.servoEncoderName.set("yellowServoEncoder");
                c.turnController.set(Controller.pid(0.3, 0, 0.005)
                        .plus(Controller.proportionalFeedforward(0)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(CRServo.Direction.REVERSE);

                c.angleOffsetRad.set(Math.toRadians(0));
                c.podOffset.set(Vector2D.cartesian(-8.75, -8.25)); // Back-Right (-x, -y in Pedro frame)
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
                c.servoDirection.set(CRServo.Direction.REVERSE);
                c.encoderReversed.set(true); // Red ticks decrease when spun CW (increases CCW)

                c.angleOffsetRad.set(Math.toRadians(0));
                c.podOffset.set(Vector2D.cartesian(8.75, 8.25)); // Front-Left (+x, +y in Pedro frame)
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
                c.servoDirection.set(CRServo.Direction.REVERSE);

                c.angleOffsetRad.set(Math.toRadians(0));
                c.podOffset.set(Vector2D.cartesian(8.75, -8.25)); // Front-Right (+x, -y in Pedro frame)
            }
    );

    public static CoaxialPodConfig greenPod = new CoaxialPodConfig(
            c -> {
                c.name.set("green");
                c.motorName.set("greenMotor");
                c.servoName.set("greenServo");
                c.servoEncoderName.set("greenServoEncoder");
                c.turnController.set(Controller.pid(0.3, 0, 0.0086)
                        .plus(Controller.proportionalFeedforward(0)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(CRServo.Direction.REVERSE);

                c.angleOffsetRad.set(Math.toRadians(0));
                c.podOffset.set(Vector2D.cartesian(-8.75, 8.25)); // Back-Left (-x, +y in Pedro frame)
            }
    );

    public static SwerveConfig driveConfig = new SwerveConfig(
            c -> {
                c.zeroPowerBehavior.set(SwerveConfig.ZeroPowerBehavior.X_LOCK);
                c.manualBrakeMode.set(true);
                c.voltageCompensation.set(false);
            }
    );

    public static ThroughBoreCoaxialPod yellow, red, blue, green;

    public static Follower create(HardwareMap h) {
        // Enable Lynx bulk caching for optimal Pedro follower loop frequencies
        try {
            List<LynxModule> allHubs = h.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        } catch (Exception ignored) {}

        OctoQuad octoquad = null;
        if (SwerveDriveV2.USE_OCTOQUAD) {
            try {
                octoquad = h.get(OctoQuad.class, SwerveDriveV2.OCTOQUAD_NAME);
                octoquad.setCachingMode(OctoQuad.CachingMode.AUTO);
            } catch (Exception ignored) {
                // OctoQuad not configured or found; fall back to Control Hub motor encoder ports
                octoquad = null;
            }
        }

        yellow = new ThroughBoreCoaxialPod(h, yellowPod, octoquad, SwerveDriveV2.YELLOW_ENCODER_CHANNEL);
        red    = new ThroughBoreCoaxialPod(h, redPod,    octoquad, SwerveDriveV2.RED_ENCODER_CHANNEL);
        blue   = new ThroughBoreCoaxialPod(h, bluePod,   octoquad, SwerveDriveV2.BLUE_ENCODER_CHANNEL);
        green  = new ThroughBoreCoaxialPod(h, greenPod,  octoquad, SwerveDriveV2.GREEN_ENCODER_CHANNEL);

        return new Follower(new PinpointLocalizer(h, pinpointConfig), new Swerve(h, driveConfig,
                blue, yellow, green, red), new Foresight(foresightConfig));
    }

    /** True if autonomous pods are currently reading steering encoders from an OctoQuad. */
    public static boolean isUsingOctoQuad() {
        return yellow != null && yellow.isUsingOctoQuad();
    }

    /**
     * Re-zeros all four pod encoders and enables servo PWM. Call on start() so that any
     * straightening of the wheels during init/init_loop is captured as the true zero position.
     */
    public static void resetPodEncoders() {
        if (yellow != null) { yellow.resetEncoder(); yellow.enablePwm(); }
        if (red    != null) { red.resetEncoder();    red.enablePwm(); }
        if (blue   != null) { blue.resetEncoder();   blue.enablePwm(); }
        if (green  != null) { green.resetEncoder();  green.enablePwm(); }
    }
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                c.parametricTConstraint.set(0.025);
                c.velocityConstraint.set(0.1);
                c.translationalConstraint.set(0.1);
                c.headingConstraint.set(0.007);
                c.timeoutConstraint.set(100.0);
                // Other Foresight constants...
            }
    );

    // Odometry computer (goBILDA Pinpoint). Everything the Pinpoint needs is set here,
    // inside the config lambda, instead of calling methods on the driver directly.
    public static PinpointConfig pinpointConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint"); // must match the device name in the Robot Configuration
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.xPodOffset.set(xOffset);
                c.yPodOffset.set(yOffset);
                c.offsetUnits.set(DistanceUnit.MM); // the offsets above are in mm
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }
    );

}
