package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/*
 * TwoZoneSwerveDrive - field-centric coaxial swerve drive using a simple
 * overshoot-proof Two-Zone Approach steering controller (no PID / no kD needed).
 *
 * Steering zones:
 *   1. Far Zone (|error| >= SLOW_ZONE_DEG): runs at steady FAST_TURN_POWER.
 *   2. Approach Zone (STEER_TOLERANCE_DEG < |error| < SLOW_ZONE_DEG):
 *      smoothly ramps down from APPROACH_POWER to CREEP_POWER so every pod
 *      enters the target at a gentle, predictable speed without overshooting.
 *   3. Settled Band (|error| <= STEER_TOLERANCE_DEG): cuts power to 0 and stays
 *      quiet until error exceeds STEER_WAKE_DEG (Schmitt-trigger deadband).
 *
 * Shares hardware config, OctoQuad/MotorPort encoders, and polarity flags with SwerveDriveV2.
 */
public class TwoZoneSwerveDrive {

    // ==================== Robot Dimensions ====================

    public double WheelbaseLength = 17.5; // front-to-back length (inches)
    public double WheelbaseWidth  = 16.5; // left-to-right width (inches)

    // ==================== Two-Zone Steering Settings ====================

    /** Servo power when far from target (|error| >= SLOW_ZONE_DEG). */
    public static double FAST_TURN_POWER = 0.45;

    /** Angle (degrees) from target where pods switch from fast turn into the gentle approach ramp. */
    public static double SLOW_ZONE_DEG = 25.0;

    /** Servo power right as the pod enters SLOW_ZONE_DEG. */
    public static double APPROACH_POWER = 0.22;

    /** Gentle minimum creep power right outside STEER_TOLERANCE_DEG so pods land cleanly without overshoot. */
    public static double CREEP_POWER = 0.08;

    /** Stop steering once within this angle (degrees) of target. */
    public static double STEER_TOLERANCE_DEG = 2.5;

    /** Stay off after settling until error exceeds this angle (degrees) so backlash/coast never hunts. */
    public static double STEER_WAKE_DEG = 5.0;

    /** Hysteresis thresholds around 90 deg so pods don't chatter reversing drive & steer direction. */
    private static final double FLIP_ENTER_RAD = Math.toRadians(95.0);
    private static final double FLIP_EXIT_RAD  = Math.toRadians(85.0);

    /** Minimum change in servo power before sending a new PWM write (0 is always sent immediately). */
    private static final double SERVO_CACHE_THRESHOLD = 0.002;

    // ==================== Module Class ====================

    /** One corner of the drivetrain: steering CRServo, drive motor, and steering encoder. */
    public static class Module {
        public final String name;
        public final CRServo servo;
        public final DcMotor motor;
        public final SwerveDriveV2.SteeringEncoder encoderSource;
        public final double x, y; // wheel position from robot center: +x = right, +y = forward
        public double offsetRad;
        public boolean encoderReversed;
        private int zeroOffsetTicks = 0;

        private double targetAngle = 0;
        private boolean lastFlipped = false;
        private boolean steerSettled = false;
        private double currentErrorDeg = 0;
        private double currentServoPower = 0;
        private double lastSentServoPower = Double.NaN;
        private double currentMotorPower = 0;
        private boolean pwmEnabled = true;

        public Module(String name, CRServo servo, DcMotor motor, SwerveDriveV2.SteeringEncoder encoderSource,
                      double x, double y, double offsetRad, boolean encoderReversed) {
            this.name = name;
            this.servo = servo;
            this.motor = motor;
            this.encoderSource = encoderSource;
            this.x = x;
            this.y = y;
            this.offsetRad = offsetRad;
            this.encoderReversed = encoderReversed;
        }

        /** Returns encoder ticks relative to zero. */
        public int getTicks() {
            return encoderSource.getPosition() - zeroOffsetTicks;
        }

        /** Resets this pod's encoder to 0 and clears steering state. */
        public void resetEncoder() {
            zeroOffsetTicks = encoderSource.getPosition();
            lastFlipped = false;
            steerSettled = false;
            targetAngle = 0;
            currentErrorDeg = 0;
        }

        /** Reads raw angle in radians [0, 2*PI) from the encoder ticks. */
        public double getRawAngleRad() {
            double ticks = getTicks();
            double ticksPerPodRev = SwerveDriveV2.ENCODER_TICKS_PER_REV * SwerveDriveV2.STEER_GEAR_RATIO;
            if (ticksPerPodRev == 0) return 0;
            double rotations = (ticks % ticksPerPodRev) / ticksPerPodRev;
            if (rotations < 0) {
                rotations += 1.0;
            }
            return rotations * 2.0 * Math.PI;
        }

        /** Current pod angle relative to robot forward (0 rad = forward, + = right) in [-PI, PI]. */
        public double getCurrentAngleRad() {
            double raw = getRawAngleRad();
            if (encoderReversed) {
                raw = (2.0 * Math.PI) - raw;
            }
            return SwerveDriveV2.normalizeAngle(raw - offsetRad);
        }

        public double getCurrentAngleDeg() { return Math.toDegrees(getCurrentAngleRad()); }
        public double getTargetAngleDeg()  { return Math.toDegrees(targetAngle); }
        public double getErrorDeg()        { return currentErrorDeg; }
        public double getServoPower()      { return currentServoPower; }
        public double getMotorPower()      { return currentMotorPower; }
        public String getActiveZone() {
            if (!pwmEnabled) return "LIMP";
            if (steerSettled) return "SETTLED";
            return Math.abs(currentErrorDeg) >= SLOW_ZONE_DEG ? "FAST" : "APPROACH";
        }

        /** Two-Zone Approach steering power calculation (overshoot-proof, no kD). */
        private double calculateTwoZonePower(double errorRad, boolean flipped) {
            currentErrorDeg = Math.toDegrees(errorRad);
            if (flipped != lastFlipped) {
                steerSettled = false;
            }
            lastFlipped = flipped;

            double absDeg = Math.abs(currentErrorDeg);
            double wakeDeg = Math.max(STEER_WAKE_DEG, STEER_TOLERANCE_DEG * 1.8);

            if (steerSettled) {
                if (absDeg <= wakeDeg) {
                    return 0.0;
                }
                steerSettled = false;
            }
            if (absDeg <= STEER_TOLERANCE_DEG) {
                steerSettled = true;
                return 0.0;
            }

            double mag;
            if (absDeg >= SLOW_ZONE_DEG) {
                mag = FAST_TURN_POWER;
            } else {
                double span = Math.max(1.0, SLOW_ZONE_DEG - STEER_TOLERANCE_DEG);
                double t = (absDeg - STEER_TOLERANCE_DEG) / span; // 0 at tolerance -> 1 at SLOW_ZONE_DEG
                mag = CREEP_POWER + t * (APPROACH_POWER - CREEP_POWER);
            }

            double turnPower = Math.signum(errorRad) * Math.max(0.0, Math.min(1.0, mag));
            if (SwerveDriveV2.SERVO_REVERSED) {
                turnPower = -turnPower;
            }
            return Math.max(-1.0, Math.min(1.0, turnPower));
        }

        private boolean shouldFlip(double errorRad) {
            double threshold = lastFlipped ? FLIP_EXIT_RAD : FLIP_ENTER_RAD;
            return Math.abs(errorRad) > threshold;
        }

        private void sendServoPower(double power) {
            enablePwm();
            if (Double.isNaN(lastSentServoPower)
                    || (power == 0 && lastSentServoPower != 0)
                    || Math.abs(power - lastSentServoPower) >= SERVO_CACHE_THRESHOLD) {
                servo.setPower(power);
                lastSentServoPower = power;
            }
            currentServoPower = power;
        }

        /** Update steering (Two-Zone) and drive motor power. */
        public void update(double commandedAngle, double commandedPower) {
            this.targetAngle = commandedAngle;
            double error = SwerveDriveV2.normalizeAngle(commandedAngle - getCurrentAngleRad());

            double drivePower = commandedPower;
            boolean flipped = shouldFlip(error);
            if (flipped) {
                error = error > 0 ? error - Math.PI : error + Math.PI;
                drivePower = -drivePower;
            }

            double turnPower = calculateTwoZonePower(error, flipped);

            double cosScale = Math.max(0.0, Math.cos(error));
            double alignedDrivePower = drivePower * cosScale;
            double finalDrivePower = Math.abs(alignedDrivePower) < SwerveDriveV2.DEADZONE ? 0 : alignedDrivePower;

            sendServoPower(turnPower);
            motor.setPower(finalDrivePower);
            currentMotorPower = finalDrivePower;
        }

        /** Update steering only (for diagnostic testing without spinning drive motors). */
        public void updateSteering(double commandedAngle) {
            this.targetAngle = commandedAngle;
            double error = SwerveDriveV2.normalizeAngle(commandedAngle - getCurrentAngleRad());

            boolean flipped = shouldFlip(error);
            if (flipped) {
                error = error > 0 ? error - Math.PI : error + Math.PI;
            }

            double turnPower = calculateTwoZonePower(error, flipped);
            sendServoPower(turnPower);
        }

        /** Directly set continuous rotation servo power (respects SERVO_REVERSED). */
        public void setDirectServoPower(double power) {
            double clamped = Math.max(-1.0, Math.min(1.0, power));
            if (SwerveDriveV2.SERVO_REVERSED) {
                clamped = -clamped;
            }
            sendServoPower(clamped);
        }

        /** Disables PWM output so pod can be freely turned by hand. */
        public void disablePwm() {
            if (pwmEnabled) {
                if (Double.isNaN(lastSentServoPower) || lastSentServoPower != 0) {
                    servo.setPower(0);
                }
                if (servo instanceof PwmControl) {
                    ((PwmControl) servo).setPwmDisable();
                }
            }
            pwmEnabled = false;
            lastSentServoPower = Double.NaN;
            currentServoPower = 0;
            currentErrorDeg = 0;
            lastFlipped = false;
            steerSettled = false;
        }

        /** Enables PWM output so steering servo can receive power commands. */
        public void enablePwm() {
            if (!pwmEnabled) {
                if (servo instanceof PwmControl) {
                    ((PwmControl) servo).setPwmEnable();
                }
                pwmEnabled = true;
                lastSentServoPower = Double.NaN;
            }
        }

        public void stopSteer() {
            if (Double.isNaN(lastSentServoPower) || lastSentServoPower != 0) {
                servo.setPower(0);
                lastSentServoPower = 0;
            }
            lastFlipped = false;
            steerSettled = false;
            currentServoPower = 0;
            currentErrorDeg = 0;
        }

        public void stopMotor() {
            motor.setPower(0);
            currentMotorPower = 0;
        }

        public void stop() {
            stopSteer();
            stopMotor();
        }
    }

    // ==================== Drivetrain State ====================

    private Module[] modules;
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    private double yawOffsetRad = 0.0;
    private boolean usingOctoQuad = false;

    public void init(HardwareMap hMap) {
        try {
            List<LynxModule> allHubs = hMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        } catch (Exception ignored) {}

        try {
            pinpoint = hMap.get(GoBildaPinpointDriver.class, "pinpoint");
        } catch (Exception ignored) {
            pinpoint = null;
        }

        try {
            imu = hMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            imu.resetYaw();
        } catch (Exception ignored) {
            imu = null;
        }
        resetYaw();

        double halfL = WheelbaseLength / 2.0;
        double halfW = WheelbaseWidth / 2.0;

        SwerveDriveV2.SteeringEncoder redEnc = null, blueEnc = null, greenEnc = null, yellowEnc = null;
        usingOctoQuad = false;
        if (SwerveDriveV2.USE_OCTOQUAD) {
            try {
                OctoQuad oq = hMap.get(OctoQuad.class, SwerveDriveV2.OCTOQUAD_NAME);
                oq.setCachingMode(OctoQuad.CachingMode.AUTO);
                redEnc    = new SwerveDriveV2.OctoQuadEncoder(oq, SwerveDriveV2.RED_ENCODER_CHANNEL);
                blueEnc   = new SwerveDriveV2.OctoQuadEncoder(oq, SwerveDriveV2.BLUE_ENCODER_CHANNEL);
                greenEnc  = new SwerveDriveV2.OctoQuadEncoder(oq, SwerveDriveV2.GREEN_ENCODER_CHANNEL);
                yellowEnc = new SwerveDriveV2.OctoQuadEncoder(oq, SwerveDriveV2.YELLOW_ENCODER_CHANNEL);
                usingOctoQuad = true;
            } catch (Exception ignored) {
                usingOctoQuad = false;
            }
        }

        if (redEnc == null) {
            redEnc    = new SwerveDriveV2.MotorPortEncoder(hMap.tryGet(DcMotor.class, "redServoEncoder"));
            blueEnc   = new SwerveDriveV2.MotorPortEncoder(hMap.tryGet(DcMotor.class, "blueServoEncoder"));
            greenEnc  = new SwerveDriveV2.MotorPortEncoder(hMap.tryGet(DcMotor.class, "greenServoEncoder"));
            yellowEnc = new SwerveDriveV2.MotorPortEncoder(hMap.tryGet(DcMotor.class, "yellowServoEncoder"));
        }

        modules = new Module[]{
                new Module("red",
                        hMap.get(CRServo.class, "redServo"),
                        hMap.get(DcMotor.class, "redMotor"),
                        redEnc,
                        -halfW,  halfL, Math.toRadians(SwerveDriveV2.RED_OFFSET_DEG),
                        SwerveDriveV2.ENCODER_REVERSED ^ SwerveDriveV2.RED_ENCODER_REVERSED),
                new Module("blue",
                        hMap.get(CRServo.class, "blueServo"),
                        hMap.get(DcMotor.class, "blueMotor"),
                        blueEnc,
                         halfW,  halfL, Math.toRadians(SwerveDriveV2.BLUE_OFFSET_DEG),
                        SwerveDriveV2.ENCODER_REVERSED ^ SwerveDriveV2.BLUE_ENCODER_REVERSED),
                new Module("green",
                        hMap.get(CRServo.class, "greenServo"),
                        hMap.get(DcMotor.class, "greenMotor"),
                        greenEnc,
                        -halfW, -halfL, Math.toRadians(SwerveDriveV2.GREEN_OFFSET_DEG),
                        SwerveDriveV2.ENCODER_REVERSED ^ SwerveDriveV2.GREEN_ENCODER_REVERSED),
                new Module("yellow",
                        hMap.get(CRServo.class, "yellowServo"),
                        hMap.get(DcMotor.class, "yellowMotor"),
                        yellowEnc,
                         halfW, -halfL, Math.toRadians(SwerveDriveV2.YELLOW_OFFSET_DEG),
                        SwerveDriveV2.ENCODER_REVERSED ^ SwerveDriveV2.YELLOW_ENCODER_REVERSED),
        };

        for (Module m : modules) {
            m.motor.setDirection(DcMotor.Direction.FORWARD);
            m.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.servo.setDirection(CRServo.Direction.FORWARD);
            m.resetEncoder();
            m.disablePwm();
        }
    }

    public void resetEncoders() {
        if (modules != null) {
            for (Module m : modules) m.resetEncoder();
        }
    }

    public void disablePwm() {
        if (modules != null) {
            for (Module m : modules) m.disablePwm();
        }
    }

    public void enablePwm() {
        if (modules != null) {
            for (Module m : modules) m.enablePwm();
        }
    }

    public void setPinpoint(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
        resetYaw();
    }

    public GoBildaPinpointDriver getPinpoint() { return pinpoint; }
    public boolean isUsingPinpoint()          { return pinpoint != null; }
    public boolean isUsingOctoQuad()          { return usingOctoQuad; }
    public Module[] getModules()              { return modules; }

    private double readPinpointYawSafe() {
        if (pinpoint == null) return Double.NaN;
        try {
            pinpoint.update();
            double raw = pinpoint.getHeading(AngleUnit.RADIANS);
            return Double.isFinite(raw) ? raw : Double.NaN;
        } catch (Exception ignored) {
            return Double.NaN;
        }
    }

    public double getHeadingRad() {
        double rawYaw = readPinpointYawSafe();
        if (!Double.isFinite(rawYaw)) {
            if (imu != null) {
                try {
                    rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                } catch (Exception ignored) {
                    rawYaw = 0.0;
                }
            } else {
                rawYaw = 0.0;
            }
            return SwerveDriveV2.normalizeAngle(rawYaw);
        }
        return SwerveDriveV2.normalizeAngle(rawYaw - yawOffsetRad);
    }

    public void resetYaw() {
        double pinpointYaw = readPinpointYawSafe();
        if (Double.isFinite(pinpointYaw)) {
            yawOffsetRad = pinpointYaw;
        } else if (imu != null) {
            try {
                imu.resetYaw();
            } catch (Exception ignored) {}
            yawOffsetRad = 0.0;
        } else {
            yawOffsetRad = 0.0;
        }
    }

    public void stop() {
        if (modules != null) {
            for (Module m : modules) {
                if (SwerveDriveV2.RELAX_SERVOS_WHEN_IDLE) {
                    m.stopMotor();
                    m.disablePwm();
                } else {
                    m.stop();
                }
            }
        }
    }

    /** Field-centric drive. */
    public void move(float strafeX, float strafeY, float rotate) {
        if (modules == null) return;

        double cleanX = Math.abs(strafeX) < SwerveDriveV2.DEADZONE ? 0 : strafeX;
        double cleanY = Math.abs(strafeY) < SwerveDriveV2.DEADZONE ? 0 : strafeY;
        double cleanRotate = Math.abs(rotate) < SwerveDriveV2.DEADZONE ? 0 : rotate;

        if (cleanX == 0 && cleanY == 0 && cleanRotate == 0) {
            stop();
            return;
        }

        double yaw = getHeadingRad();
        double driveX = cleanX * Math.cos(-yaw) - cleanY * Math.sin(-yaw);
        double driveY = cleanX * Math.sin(-yaw) + cleanY * Math.cos(-yaw);

        driveWithRobotVector(driveX, driveY, cleanRotate);
    }

    /** Robot-centric drive. */
    public void moveRobotCentric(float strafeX, float strafeY, float rotate) {
        if (modules == null) return;

        double cleanX = Math.abs(strafeX) < SwerveDriveV2.DEADZONE ? 0 : strafeX;
        double cleanY = Math.abs(strafeY) < SwerveDriveV2.DEADZONE ? 0 : strafeY;
        double cleanRotate = Math.abs(rotate) < SwerveDriveV2.DEADZONE ? 0 : rotate;

        if (cleanX == 0 && cleanY == 0 && cleanRotate == 0) {
            stop();
            return;
        }

        driveWithRobotVector(cleanX, cleanY, cleanRotate);
    }

    private void driveWithRobotVector(double driveX, double driveY, double cleanRotate) {
        enablePwm();

        double halfDiagonal = Math.hypot(WheelbaseLength, WheelbaseWidth) / 2.0;
        double[] power = new double[modules.length];
        double[] angle = new double[modules.length];
        double maxPower = 1.0;

        for (int i = 0; i < modules.length; i++) {
            Module m = modules[i];
            double pushX = driveX + cleanRotate * (m.y / halfDiagonal);
            double pushY = driveY - cleanRotate * (m.x / halfDiagonal);
            power[i] = Math.hypot(pushX, pushY);
            angle[i] = Math.atan2(pushX, pushY);
            maxPower = Math.max(maxPower, power[i]);
        }

        for (int i = 0; i < modules.length; i++) {
            modules[i].update(angle[i], power[i] / maxPower);
        }
    }
}
