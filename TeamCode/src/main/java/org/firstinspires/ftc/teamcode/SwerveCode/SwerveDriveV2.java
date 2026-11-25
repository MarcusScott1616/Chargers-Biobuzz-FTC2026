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
 * SwerveDriveV2 - field-centric coaxial swerve drive with continuous rotation servos
 * and REV Through-Bore encoders (in quadrature mode on Control Hub motor encoder ports).
 *
 * Architecture:
 *   - Steering: Continuous Rotation Servos (CRServo) driven by a closed-loop PID.
 *   - Feedback: REV Through-Bore encoders (8192 ticks/rev) connected to Control Hub motor encoder ports.
 *   - Drive: DcMotors with BRAKE mode (connected to Expansion Hub motor ports).
 *   - Optimization: Never turns more than 90 degrees; reverses drive motor if target is > 90 deg away.
 */
public class SwerveDriveV2 {

    /*
     * RED--------BLU      front of robot is at the top (+y)
     *  |          |
     *  |          |
     * GRN--------YLW
     * -x         +x
     */

    /** Distance between front and back wheels, and between left and right wheels (inches). */
    public double WheelbaseLength = 17.5; // front-to-back length
    public double WheelbaseWidth = 16.5;  // left-to-right width

    /** Stick inputs smaller than this are treated as zero. */
    public static double DEADZONE = 0.05;

    /** PID Gains for Pod Steering. */
    public static double kP = 0.45;
    public static double kI = 0.0;
    public static double kD = 0.005;
    public static double MAX_INTEGRAL = 0.2;

    /** Minimum servo power to overcome static friction (disabled = 0.0). */
    public static double kStatic = 0.0;

    /** Angular tolerance (radians) within which steering is considered "on target" (~2 degrees). */
    public static double STEER_TOLERANCE = Math.toRadians(2.0);

    /** When true, CRServos are powered down (0 power) when joysticks are idle to prevent jitter and heating. */
    public static boolean RELAX_SERVOS_WHEN_IDLE = true;

    /** REV Through-Bore Encoder ticks per revolution in 4x quadrature decoding. */
    public static final double ENCODER_TICKS_PER_REV = 8192.0;

    /** Gear ratio between pod rotation and encoder shaft (16T servo shaft to 48T pod = 3.0). */
    public static double STEER_GEAR_RATIO = 48.0 / 16.0;

    /** Zero-angle offsets (in degrees) for each module when pointing straight forward (0 rad). */
    public static double RED_OFFSET_DEG = 0.0;
    public static double BLUE_OFFSET_DEG = 0.0;
    public static double GREEN_OFFSET_DEG = 0.0;
    public static double YELLOW_OFFSET_DEG = 0.0;

    /** Set to true if an encoder's ticks decrease when the pod turns to the right (+ angle). */
    public static boolean ENCODER_REVERSED = false;

    /** Set to true if turning power needs to be flipped for all steering servos. */
    public static boolean SERVO_REVERSED = false;

    // ==================== OctoQuad Configuration ====================
    // Set USE_OCTOQUAD to true once you have the OctoQuad wired and configured.
    // When false, encoders are read from Control Hub motor encoder ports (the current setup).
    // When true, encoders are read from the OctoQuad I2C board.

    /** Set to true to read steering encoders from an OctoQuad instead of motor encoder ports. */
    public static boolean USE_OCTOQUAD = true;

    /** Hardware name of the OctoQuad in the robot configuration (I2C device). */
    public static String OCTOQUAD_NAME = "octoquad";

    /** OctoQuad channel assignments for each steering encoder (0-7). */
    public static int RED_ENCODER_CHANNEL = 0;
    public static int BLUE_ENCODER_CHANNEL = 1;
    public static int GREEN_ENCODER_CHANNEL = 2;
    public static int YELLOW_ENCODER_CHANNEL = 3;

    // ==================== Encoder Abstraction ====================

    /** Interface for reading steering encoder ticks, regardless of source (motor port or OctoQuad). */
    public interface SteeringEncoder {
        /** Returns the current raw tick count. */
        int getPosition();
    }

    /** Reads encoder ticks from a Control Hub / Expansion Hub motor encoder port. */
    public static class MotorPortEncoder implements SteeringEncoder {
        private final DcMotor motor;
        public MotorPortEncoder(DcMotor motor) {
            this.motor = motor;
            if (motor != null) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        @Override public int getPosition() {
            return motor != null ? motor.getCurrentPosition() : 0;
        }
        /** Exposes the underlying DcMotor for SwerveSim physics. */
        public DcMotor getMotor() { return motor; }
    }

    /** Reads encoder ticks from a single channel on an OctoQuad I2C encoder board. */
    public static class OctoQuadEncoder implements SteeringEncoder {
        private final OctoQuad octoquad;
        private final int channel;
        public OctoQuadEncoder(OctoQuad octoquad, int channel) {
            this.octoquad = octoquad;
            this.channel = channel;
        }
        @Override public int getPosition() { return octoquad.readSinglePosition_Caching(channel); }
    }

    /** One corner of the drivetrain: a steering CRServo, a drive motor, and a steering encoder. */
    public static class Module {
        public final String name;
        public final CRServo servo;
        public final DcMotor motor;
        public final SteeringEncoder encoderSource;
        public final double x, y;          // wheel position from robot center: +x = right, +y = forward
        public double offsetRad;
        public boolean encoderReversed;
        private int zeroOffsetTicks = 0;

        private double targetAngle = 0;
        private double lastError = 0;
        private double integral = 0;
        private long lastTimeNs = 0;
        private boolean lastFlipped = false;
        private double currentServoPower = 0;
        private double currentMotorPower = 0;
        private boolean pwmEnabled = true;

        public Module(String name, CRServo servo, DcMotor motor, SteeringEncoder encoderSource,
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

        /** Resets this pod's encoder to 0 (zeroes the current position) and clears PID state. */
        public void resetEncoder() {
            zeroOffsetTicks = encoderSource.getPosition();
            integral = 0;
            lastError = 0;
            lastFlipped = false;
            lastTimeNs = 0;  // forces PID to use default dt on next call
            targetAngle = 0;
        }

        /** Reads raw angle in radians [0, 2*PI) from the encoder ticks. */
        public double getRawAngleRad() {
            double ticks = getTicks();
            double ticksPerPodRev = ENCODER_TICKS_PER_REV * STEER_GEAR_RATIO;
            if (ticksPerPodRev == 0) return 0;
            double rotations = (ticks % ticksPerPodRev) / ticksPerPodRev;
            if (rotations < 0) {
                rotations += 1.0;
            }
            return rotations * 2.0 * Math.PI;
        }

        /** Current pod angle relative to robot forward (0 rad = forward, + = right, - = left) in [-PI, PI]. */
        public double getCurrentAngleRad() {
            double raw = getRawAngleRad();
            if (encoderReversed) {
                raw = (2.0 * Math.PI) - raw;
            }
            double angle = raw - offsetRad;
            return normalizeAngle(angle);
        }

        public double getCurrentAngleDeg() {
            return Math.toDegrees(getCurrentAngleRad());
        }

        public double getTargetAngleDeg() {
            return Math.toDegrees(targetAngle);
        }

        public double getServoPower() {
            return currentServoPower;
        }

        public double getMotorPower() {
            return currentMotorPower;
        }

        /**
         * Update module steering with PID and command motor power.
         *
         * @param commandedAngle target wheel angle in radians (0 = forward, + = right)
         * @param commandedPower target drive power [-1.0, 1.0]
         */
        public void update(double commandedAngle, double commandedPower) {
            this.targetAngle = commandedAngle;
            double currentAngle = getCurrentAngleRad();

            // Shortest angular difference between target and current [-PI, PI]
            double error = normalizeAngle(commandedAngle - currentAngle);

            // Swerve angle optimization: if error > 90 deg, flip motor direction and turn <= 90 deg.
            // Track whether we flipped this frame vs last frame so the derivative stays meaningful
            // across flip-state changes (the main cause of turn+strafe instability).
            double drivePower = commandedPower;
            boolean flipped = Math.abs(error) > Math.PI / 2.0;
            if (flipped) {
                error = error > 0 ? error - Math.PI : error + Math.PI;
                drivePower = -drivePower;
            }

            // Delta time calculation
            long nowNs = System.nanoTime();
            double dt = (lastTimeNs > 0) ? (nowNs - lastTimeNs) / 1e9 : 0.02;
            lastTimeNs = nowNs;
            if (dt <= 0 || dt > 0.5) {
                dt = 0.02;
                // Time gap too large — reset derivative and integral to avoid stale state
                integral = 0;
                lastError = error;
                lastFlipped = flipped;
            }

            // If the flip state changed since last frame, the stored lastError is in a different
            // frame of reference — reset it to avoid a massive derivative spike.
            if (flipped != lastFlipped) {
                lastError = error;
            }
            lastFlipped = flipped;

            // PID calculation
            integral += error * dt;
            integral = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integral));

            double derivative = (error - lastError) / dt;
            lastError = error;

            // Prevent servo chatter: if error is within STEER_TOLERANCE, cut power and reset integral.
            double turnPower;
            if (Math.abs(error) <= STEER_TOLERANCE) {
                turnPower = 0;
                integral = 0;
            } else {
                turnPower = (kP * error) + (kI * integral) + (kD * derivative);
                // Static friction feedforward: add a minimum kick in the direction of error
                if (kStatic > 0) {
                    turnPower += Math.signum(error) * kStatic;
                }
            }

            if (SERVO_REVERSED) {
                turnPower = -turnPower;
            }

            // Clamp servo power to [-1.0, 1.0]
            turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

            // Cosine scaling: scale drive power by wheel alignment to prevent driving sideways while turning
            double alignedDrivePower = drivePower * Math.cos(error);
            double finalDrivePower = Math.abs(alignedDrivePower) < DEADZONE ? 0 : alignedDrivePower;

            enablePwm();
            servo.setPower(turnPower);
            motor.setPower(finalDrivePower);
            currentServoPower = turnPower;
            currentMotorPower = finalDrivePower;
        }

        /**
         * Update steering PID only without driving the wheel (used for diagnostic and holding pod angle).
         *
         * @param commandedAngle target wheel angle in radians (0 = forward, + = right)
         */
        public void updateSteering(double commandedAngle) {
            this.targetAngle = commandedAngle;
            double currentAngle = getCurrentAngleRad();

            // Shortest angular difference between target and current [-PI, PI]
            double error = normalizeAngle(commandedAngle - currentAngle);

            // Swerve angle optimization: turn <= 90 deg.
            // Reset lastError when flip state changes to avoid derivative spike.
            boolean flipped = Math.abs(error) > Math.PI / 2.0;
            if (flipped) {
                error = error > 0 ? error - Math.PI : error + Math.PI;
            }

            // Delta time calculation
            long nowNs = System.nanoTime();
            double dt = (lastTimeNs > 0) ? (nowNs - lastTimeNs) / 1e9 : 0.02;
            lastTimeNs = nowNs;
            if (dt <= 0 || dt > 0.5) {
                dt = 0.02;
                integral = 0;
                lastError = error;
                lastFlipped = flipped;
            }

            if (flipped != lastFlipped) {
                lastError = error;
            }
            lastFlipped = flipped;

            // PID calculation
            integral += error * dt;
            integral = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integral));

            double derivative = (error - lastError) / dt;
            lastError = error;

            // Prevent servo chatter: cut power and clear integral when within STEER_TOLERANCE
            double turnPower;
            if (Math.abs(error) <= STEER_TOLERANCE) {
                turnPower = 0;
                integral = 0;
            } else {
                turnPower = (kP * error) + (kI * integral) + (kD * derivative);
                // Static friction feedforward: add a minimum kick in the direction of error
                if (kStatic > 0) {
                    turnPower += Math.signum(error) * kStatic;
                }
            }

            if (SERVO_REVERSED) {
                turnPower = -turnPower;
            }

            turnPower = Math.max(-1.0, Math.min(1.0, turnPower));
            enablePwm();
            servo.setPower(turnPower);
            currentServoPower = turnPower;
        }

        /** Directly set continuous rotation servo power (bypassing PID, for open-loop hardware testing). */
        public void setDirectServoPower(double power) {
            double clamped = Math.max(-1.0, Math.min(1.0, power));
            enablePwm();
            servo.setPower(clamped);
            currentServoPower = clamped;
        }

        /** Disables PWM output to this servo, completely de-energizing it so it can be moved freely by hand. */
        public void disablePwm() {
            servo.setPower(0);
            if (pwmEnabled && servo instanceof PwmControl) {
                ((PwmControl) servo).setPwmDisable();
            }
            pwmEnabled = false;
            currentServoPower = 0;
            integral = 0;
            lastError = 0;
            lastTimeNs = 0;
        }

        /** Enables PWM output to this servo so it can receive power commands. */
        public void enablePwm() {
            if (!pwmEnabled && servo instanceof PwmControl) {
                ((PwmControl) servo).setPwmEnable();
            }
            pwmEnabled = true;
        }

        public void stopSteer() {
            servo.setPower(0);
            integral = 0;
            lastError = 0;
            lastFlipped = false;
            lastTimeNs = 0;  // reset so next PID call doesn't get a stale time gap
            currentServoPower = 0;
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

    private Module[] modules;
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    private double yawOffsetRad = 0.0;
    private boolean usingOctoQuad = false;

    public void init(HardwareMap hMap) {
        // Enable Lynx bulk caching mode across all REV Hubs for ~6x faster loop speeds (~8ms)
        try {
            List<LynxModule> allHubs = hMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        } catch (Exception ignored) {
            // Graceful fallback for simulation or non-Lynx hardware
        }

        // Auto-detect goBILDA Pinpoint driver for drift-free, high-precision IMU yaw
        try {
            pinpoint = hMap.get(GoBildaPinpointDriver.class, "pinpoint");
        } catch (Exception ignored) {
            pinpoint = null;
        }

        // Initialize REV Hub IMU as fallback
        try {
            imu = hMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            imu.resetYaw();
        } catch (Exception ignored) {
            imu = null;
        }
        yawOffsetRad = 0.0;

        double halfL = WheelbaseLength / 2.0, halfW = WheelbaseWidth / 2.0;

        // Build encoder sources: OctoQuad (I2C) or motor encoder ports
        SteeringEncoder redEnc = null, blueEnc = null, greenEnc = null, yellowEnc = null;
        usingOctoQuad = false;
        if (USE_OCTOQUAD) {
            try {
                OctoQuad oq = hMap.get(OctoQuad.class, OCTOQUAD_NAME);
                oq.setCachingMode(OctoQuad.CachingMode.AUTO);
                redEnc    = new OctoQuadEncoder(oq, RED_ENCODER_CHANNEL);
                blueEnc   = new OctoQuadEncoder(oq, BLUE_ENCODER_CHANNEL);
                greenEnc  = new OctoQuadEncoder(oq, GREEN_ENCODER_CHANNEL);
                yellowEnc = new OctoQuadEncoder(oq, YELLOW_ENCODER_CHANNEL);
                usingOctoQuad = true;
            } catch (Exception ignored) {
                // If OctoQuad is not configured or found, gracefully fall back to motor encoder ports
                usingOctoQuad = false;
            }
        }

        if (redEnc == null) {
            redEnc    = new MotorPortEncoder(hMap.tryGet(DcMotor.class, "redServoEncoder"));
            blueEnc   = new MotorPortEncoder(hMap.tryGet(DcMotor.class, "blueServoEncoder"));
            greenEnc  = new MotorPortEncoder(hMap.tryGet(DcMotor.class, "greenServoEncoder"));
            yellowEnc = new MotorPortEncoder(hMap.tryGet(DcMotor.class, "yellowServoEncoder"));
        }

        modules = new Module[]{
                new Module("red",
                        hMap.get(CRServo.class, "redServo"),
                        hMap.get(DcMotor.class, "redMotor"),
                        redEnc,
                        -halfW,  halfL, Math.toRadians(RED_OFFSET_DEG), ENCODER_REVERSED),
                new Module("blue",
                        hMap.get(CRServo.class, "blueServo"),
                        hMap.get(DcMotor.class, "blueMotor"),
                        blueEnc,
                         halfW,  halfL, Math.toRadians(BLUE_OFFSET_DEG), ENCODER_REVERSED),
                new Module("green",
                        hMap.get(CRServo.class, "greenServo"),
                        hMap.get(DcMotor.class, "greenMotor"),
                        greenEnc,
                        -halfW, -halfL, Math.toRadians(GREEN_OFFSET_DEG), ENCODER_REVERSED),
                new Module("yellow",
                        hMap.get(CRServo.class, "yellowServo"),
                        hMap.get(DcMotor.class, "yellowMotor"),
                        yellowEnc,
                         halfW, -halfL, Math.toRadians(YELLOW_OFFSET_DEG), ENCODER_REVERSED),
        };

        for (Module m : modules) {
            m.motor.setDirection(DcMotor.Direction.FORWARD);
            m.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.servo.setDirection(CRServo.Direction.FORWARD);
            m.resetEncoder();
            m.disablePwm(); // Keep servos completely unpowered on INIT so pods can be freely aligned
        }
    }

    /** Re-zeros all pod steering encoders (sets current wheel angles to 0 rad). */
    public void resetEncoders() {
        if (modules != null) {
            for (Module m : modules) {
                m.resetEncoder();
            }
        }
    }

    /** Disables PWM output to all steering servos (makes them completely limp). */
    public void disablePwm() {
        if (modules != null) {
            for (Module m : modules) {
                m.disablePwm();
            }
        }
    }

    /** Enables PWM output to all steering servos. */
    public void enablePwm() {
        if (modules != null) {
            for (Module m : modules) {
                m.enablePwm();
            }
        }
    }

    /** Sets an explicit Pinpoint driver instance (e.g. from PinpointOdometry). */
    public void setPinpoint(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
    }

    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    /** True if currently using the goBILDA Pinpoint IMU for heading. */
    public boolean isUsingPinpoint() {
        return pinpoint != null;
    }

    /** True if steering encoders are currently being read from an OctoQuad I2C board. */
    public boolean isUsingOctoQuad() {
        return usingOctoQuad;
    }

    /**
     * Returns the robot yaw heading in radians [-PI, PI], prioritizing goBILDA Pinpoint IMU.
     * Subtracts any driver reset offset so field-centric zero is preserved without wiping odometry.
     */
    public double getHeadingRad() {
        double rawYaw = 0.0;
        if (pinpoint != null) {
            try {
                pinpoint.update();
            } catch (Exception ignored) {}
            rawYaw = pinpoint.getHeading(AngleUnit.RADIANS);
        } else if (imu != null) {
            rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        return normalizeAngle(rawYaw - yawOffsetRad);
    }

    /**
     * Resets field-centric "forward" to current robot heading.
     * When using Pinpoint IMU, this zeroes the drive yaw WITHOUT wiping the robot's (X, Y) field position!
     */
    public void resetYaw() {
        if (pinpoint != null) {
            try {
                pinpoint.update();
            } catch (Exception ignored) {}
            yawOffsetRad = pinpoint.getHeading(AngleUnit.RADIANS);
        } else if (imu != null) {
            imu.resetYaw();
            yawOffsetRad = 0.0;
        }
    }

    public Module[] getModules() {
        return modules;
    }

    public void stop() {
        if (modules != null) {
            for (Module m : modules) {
                if (RELAX_SERVOS_WHEN_IDLE) {
                    m.stopMotor();
                    m.disablePwm();
                } else {
                    m.stop();
                }
            }
        }
    }

    /**
     * Drives the robot, field-centric (pushing "forward" always drives away from the driver).
     *
     * @param strafeX left (-1) to right (+1)
     * @param strafeY back (-1) to forward (+1)
     * @param rotate  counter-clockwise (-1) to clockwise (+1)
     */
    public void move(float strafeX, float strafeY, float rotate) {
        if (modules == null) return;

        // Individual axis deadzones to prevent stick drift on rotation or strafe from turning pods
        double cleanX = Math.abs(strafeX) < DEADZONE ? 0 : strafeX;
        double cleanY = Math.abs(strafeY) < DEADZONE ? 0 : strafeY;
        double cleanRotate = Math.abs(rotate) < DEADZONE ? 0 : rotate;

        // No stick input: stop the motors and relax servos
        if (cleanX == 0 && cleanY == 0 && cleanRotate == 0) {
            if (RELAX_SERVOS_WHEN_IDLE) {
                for (Module m : modules) {
                    m.stopMotor();
                    m.disablePwm();
                }
            } else {
                for (Module m : modules) m.stop();
            }
            return;
        }

        enablePwm();

        // Field-centric: turn the stick direction by the opposite of the robot's heading (via Pinpoint IMU).
        double yaw = getHeadingRad();
        double driveX = cleanX * Math.cos(-yaw) - cleanY * Math.sin(-yaw);
        double driveY = cleanX * Math.sin(-yaw) + cleanY * Math.cos(-yaw);

        // Calculate push vector at each module
        double halfDiagonal = Math.hypot(WheelbaseLength, WheelbaseWidth) / 2.0;
        double[] power = new double[modules.length];
        double[] angle = new double[modules.length];
        double maxPower = 1.0;

        for (int i = 0; i < modules.length; i++) {
            Module m = modules[i];
            double pushX = driveX + cleanRotate * (m.y / halfDiagonal);
            double pushY = driveY - cleanRotate * (m.x / halfDiagonal);
            power[i] = Math.hypot(pushX, pushY);
            angle[i] = Math.atan2(pushX, pushY); // 0 = forward, + = right
            maxPower = Math.max(maxPower, power[i]);
        }

        // Normalize wheel powers so no wheel exceeds 1.0
        for (int i = 0; i < modules.length; i++) {
            modules[i].update(angle[i], power[i] / maxPower);
        }
    }

    /**
     * Drives the robot, robot-centric (pushing "forward" pushes in the direction the robot is facing).
     * Used for visual tracking / chasing objects on the ground relative to the robot's intake.
     *
     * @param strafeX left (-1) to right (+1)
     * @param strafeY back (-1) to forward (+1)
     * @param rotate  counter-clockwise (-1) to clockwise (+1)
     */
    public void moveRobotCentric(float strafeX, float strafeY, float rotate) {
        if (modules == null) return;

        double cleanX = Math.abs(strafeX) < DEADZONE ? 0 : strafeX;
        double cleanY = Math.abs(strafeY) < DEADZONE ? 0 : strafeY;
        double cleanRotate = Math.abs(rotate) < DEADZONE ? 0 : rotate;

        if (cleanX == 0 && cleanY == 0 && cleanRotate == 0) {
            if (RELAX_SERVOS_WHEN_IDLE) {
                for (Module m : modules) {
                    m.stopMotor();
                    m.disablePwm();
                }
            } else {
                for (Module m : modules) m.stop();
            }
            return;
        }

        enablePwm();

        double driveX = cleanX;
        double driveY = cleanY;

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

    /** Normalizes an angle in radians to [-PI, PI]. */
    public static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }
}
