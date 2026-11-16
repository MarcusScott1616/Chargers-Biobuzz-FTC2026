package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.SwervePod;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

import java.util.HashMap;
import java.util.Map;

/**
 * ThroughBoreCoaxialPod is an implementation of `SwervePod` for Pedro Pathing
 * that reads REV Through-Bore encoders connected to REV Hub motor encoder ports (DcMotor)
 * instead of AnalogInput.
 */
public class ThroughBoreCoaxialPod implements SwervePod {

    public static final double ENCODER_TICKS_PER_REV = 8192.0;
    public static double STEER_GEAR_RATIO = 1.0;

    private final String name;
    private final DcMotor turnEncoder;
    private final CRServo turnServo;
    private final DcMotorEx driveMotor;
    private final CoaxialPodConfig config;
    private int zeroOffsetTicks = 0;

    private double lastDrivePower = 0;
    private double lastTurnPower = 0;

    public ThroughBoreCoaxialPod(HardwareMap hardwareMap, CoaxialPodConfig config) {
        this.config = config;
        this.name = config.name.get();

        this.driveMotor = hardwareMap.get(DcMotorEx.class, config.motorName.get());
        this.turnServo = hardwareMap.get(CRServo.class, config.servoName.get());
        this.turnEncoder = hardwareMap.get(DcMotor.class, config.servoEncoderName.get());

        // Software zero offset
        this.turnEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.zeroOffsetTicks = this.turnEncoder.getCurrentPosition();

        setMotorToFloat();

        this.driveMotor.setDirection(config.driveDirection.get());
        this.turnServo.setDirection(config.servoDirection.get());

        // Keep servo unpowered on init so pods can be aligned freely by hand
        if (this.turnServo instanceof PwmControl) {
            ((PwmControl) this.turnServo).setPwmDisable();
        }
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
        if (this.turnServo instanceof PwmControl) {
            ((PwmControl) this.turnServo).setPwmEnable();
        }
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
        this.zeroOffsetTicks = this.turnEncoder.getCurrentPosition();
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

        double servoCachingThreshold = config.servoCachingThreshold.get();
        double motorCachingThreshold = config.motorCachingThreshold.get();

        if (ignoreAngleChanges) {
            lastTurnPower = 0;
            turnServo.setPower(0);
        } else if (Math.abs(turnPower - lastTurnPower) > servoCachingThreshold || (turnPower == 0 && lastTurnPower != 0)) {
            lastTurnPower = turnPower;
            turnServo.setPower(turnPower);
        }

        if (Math.abs(drivePower - lastDrivePower) > motorCachingThreshold || (drivePower == 0 && lastDrivePower != 0)) {
            lastDrivePower = drivePower;
            driveMotor.setPower(drivePower);
        }
    }

    public double getAngleAfterOffsetRad() {
        return getRawAngleRad() - config.angleOffsetRad.get();
    }

    public double getRawAngleRad() {
        double ticks = turnEncoder.getCurrentPosition() - zeroOffsetTicks;
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
