package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.SwervePod;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import org.firstinspires.ftc.teamcode.SwerveCode.SwerveDriveV2;

import java.util.HashMap;
import java.util.Map;

/**
 * ThroughBoreCoaxialPod is an implementation of `SwervePod` for Pedro Pathing
 * that reads REV Through-Bore encoders connected to REV Hub motor encoder ports (DcMotor)
 * instead of AnalogInput.
 */
public class ThroughBoreCoaxialPod implements SwervePod {

    public static final double ENCODER_TICKS_PER_REV = SwerveDriveV2.ENCODER_TICKS_PER_REV;
    /** Gear ratio between pod rotation and encoder shaft (16T servo shaft to 48T pod = 3.0). */
    public static double STEER_GEAR_RATIO = SwerveDriveV2.STEER_GEAR_RATIO;

    private final String name;
    private final SwerveDriveV2.SteeringEncoder turnEncoder;
    private final boolean usingOctoQuad;
    private final CRServo turnServo;
    private final DcMotorEx driveMotor;
    private final CoaxialPodConfig config;
    private int zeroOffsetTicks = 0;

    private double lastDrivePower = 0;
    private double lastTurnPower = 0;
    private boolean pwmEnabled = true;

    public ThroughBoreCoaxialPod(HardwareMap hardwareMap, CoaxialPodConfig config) {
        this(hardwareMap, config, null, -1);
    }

    public ThroughBoreCoaxialPod(HardwareMap hardwareMap, CoaxialPodConfig config,
                                 OctoQuad octoquad, int octoquadChannel) {
        this.config = config;
        this.name = config.name.get();

        this.driveMotor = hardwareMap.get(DcMotorEx.class, config.motorName.get());
        this.turnServo = hardwareMap.get(CRServo.class, config.servoName.get());

        if (octoquad != null && octoquadChannel >= 0) {
            this.turnEncoder = new SwerveDriveV2.OctoQuadEncoder(octoquad, octoquadChannel);
            this.usingOctoQuad = true;
        } else {
            this.turnEncoder = new SwerveDriveV2.MotorPortEncoder(
                    hardwareMap.get(DcMotor.class, config.servoEncoderName.get()));
            this.usingOctoQuad = false;
        }

        // Software zero offset
        this.zeroOffsetTicks = this.turnEncoder.getPosition();

        setMotorToFloat();

        this.driveMotor.setDirection(config.driveDirection.get());
        this.turnServo.setDirection(config.servoDirection.get());

        // Keep servo unpowered on init so pods can be aligned freely by hand
        disablePwm();
    }

    public boolean isUsingOctoQuad() {
        return usingOctoQuad;
    }

    /** Enables PWM output on this pod's steering servo if currently disabled. */
    public void enablePwm() {
        if (!pwmEnabled && this.turnServo instanceof PwmControl) {
            ((PwmControl) this.turnServo).setPwmEnable();
            pwmEnabled = true;
        }
    }

    /** Disables PWM output on this pod's steering servo so it can be moved freely by hand. */
    public void disablePwm() {
        if (pwmEnabled && this.turnServo instanceof PwmControl) {
            ((PwmControl) this.turnServo).setPwmDisable();
        }
        pwmEnabled = false;
    }

    @Override
    public Vector2D getOffset() {
        return config.podOffset.get();
    }

    @Override
    public double getAngle() {
        return getAngleAfterOffsetRad();
    }

    public void setServoPower(double power) {
        enablePwm();
        lastTurnPower = power;
        turnServo.setPower(power);
    }

    public void setMotorPower(double power) {
        lastDrivePower = power;
        driveMotor.setPower(power);
    }

    @Override
    public void setToFloat() {
        setMotorToFloat();
    }

    @Override
    public void setToBreak() {
        setMotorToBreak();
    }

    public void setMotorToFloat() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setMotorToBreak() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean isEncoderReversed() {
        return config.encoderReversed.get();
    }

    public void resetEncoder() {
        this.zeroOffsetTicks = this.turnEncoder.getPosition();
    }

    @Override
    public double adjustThetaForEncoder(double wheelTheta) {
        double t = config.encoderReversed.get() ? wheelTheta : (2 * Math.PI - wheelTheta);
        t += Math.PI / 2.0;
        return Angle.normalize(t);
    }

    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        boolean encoderReversed = config.encoderReversed.get();

        double actualRad = getAngleAfterOffsetRad();
        actualRad = Angle.normalize(actualRad);

        double desiredRad = encoderReversed ? targetAngleRad : (2 * Math.PI - targetAngleRad);
        desiredRad += Math.PI / 2.0;
        desiredRad = Angle.normalize(desiredRad);

        double mag = Angle.smallestDifference(actualRad, desiredRad);
        double dir = Angle.turnDirection(actualRad, desiredRad);
        double signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;
        double errorRad = signedRad;

        // Minimize rotation: flip + invert drive if > 90 deg
        if (Math.abs(errorRad) > (Math.PI / 2.0)) {
            desiredRad = Angle.normalize(desiredRad + Math.PI);
            drivePower = -drivePower;

            mag = Angle.smallestDifference(actualRad, desiredRad);
            dir = Angle.turnDirection(actualRad, desiredRad);
            signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;
            errorRad = signedRad;
        }

        double turnPower;
        if (Math.abs(errorRad) < (2.0 * Math.PI / 180.0)) {
            turnPower = Utils.clamp(config.turnController.get().calculate(0, errorRad), -1.0, 1.0);
        } else {
            turnPower = Utils.clamp(
                    config.turnController.get().calculate(
                            Angle.turnDirection(actualRad, desiredRad), errorRad
                    ),
                    -1.0,
                    1.0
            );
        }

        enablePwm();

        double servoCachingThreshold = config.servoCachingThreshold.get();
        double motorCachingThreshold = config.motorCachingThreshold.get();

        if (ignoreAngleChanges) {
            lastTurnPower = 0;
            turnServo.setPower(0);
        } else if (Math.abs(turnPower - lastTurnPower) > servoCachingThreshold || (turnPower == 0 && lastTurnPower != 0)) {
            lastTurnPower = turnPower;
            turnServo.setPower(turnPower);
        }

        // Cosine scaling: scale drive power by wheel alignment to prevent scrubbing/darting while steering
        double scaledDrivePower = drivePower * Math.cos(errorRad);

        if (Math.abs(scaledDrivePower - lastDrivePower) > motorCachingThreshold || (scaledDrivePower == 0 && lastDrivePower != 0)) {
            lastDrivePower = scaledDrivePower;
            driveMotor.setPower(scaledDrivePower);
        }
    }

    public double getAngleAfterOffsetRad() {
        return getRawAngleRad() - config.angleOffsetRad.get();
    }

    public double getRawAngleRad() {
        double ticks = turnEncoder.getPosition() - zeroOffsetTicks;
        double ticksPerPodRev = ENCODER_TICKS_PER_REV * STEER_GEAR_RATIO;
        if (ticksPerPodRev == 0) return 0;
        double rotations = (ticks % ticksPerPodRev) / ticksPerPodRev;
        if (rotations < 0) {
            rotations += 1.0;
        }
        return rotations * (2.0 * Math.PI);
    }

    public double getOffsetAngleRad() {
        double rad = getRawAngleRad() - config.angleOffsetRad.get();
        return Angle.normalize(rad);
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public Map<String, Object> debug() {
        double rawAngleRad = getRawAngleRad();
        double offsetAngleRad = getAngleAfterOffsetRad();

        Map<String, Object> map = new HashMap<>();
        map.put("servoName", config.servoName.get());
        map.put("rawAngleRad", rawAngleRad);
        map.put("rawAngleDeg", Math.toDegrees(rawAngleRad));
        map.put("angleAfterOffsetRad", offsetAngleRad);
        map.put("angleAfterOffsetDeg", Math.toDegrees(offsetAngleRad));
        map.put("servoPower", turnServo.getPower());
        map.put("drivePower", driveMotor.getPower());
        return map;
    }
}
