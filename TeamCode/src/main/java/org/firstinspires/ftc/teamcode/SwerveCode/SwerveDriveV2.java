package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
    public double WheelbaseLength = 17;
    public double WheelbaseWidth = 17;

    /** Stick inputs smaller than this are treated as zero. */
    public static double DEADZONE = 0.05;

    /** PID Gains for Pod Steering. */
    public static double kP = 0.45;
    public static double kI = 0.0;
    public static double kD = 0.005;
    public static double MAX_INTEGRAL = 0.2;

    /** Minimum servo power to overcome static friction. Applied whenever error is above STEER_TOLERANCE. */
    public static double kStatic = 0.06;

    /** Angular tolerance (radians) within which steering is considered "on target" (~2 degrees). */
    public static double STEER_TOLERANCE = Math.toRadians(2.0);

    /** When true, CRServos are powered down (0 power) when joysticks are idle to prevent jitter and heating. */
    public static boolean RELAX_SERVOS_WHEN_IDLE = true;

    /** REV Through-Bore Encoder ticks per revolution in 4x quadrature decoding. */
    public static final double ENCODER_TICKS_PER_REV = 8192.0;

    /** Gear ratio between pod rotation and encoder shaft (pod_rotations / encoder_rotations). 1.0 if 1:1. */
    public static double STEER_GEAR_RATIO = 1.0;

    /** Zero-angle offsets (in degrees) for each module when pointing straight forward (0 rad). */
    public static double RED_OFFSET_DEG = 0.0;
    public static double BLUE_OFFSET_DEG = 0.0;
    public static double GREEN_OFFSET_DEG = 0.0;
    public static double YELLOW_OFFSET_DEG = 0.0;

    /** Set to true if an encoder's ticks decrease when the pod turns to the right (+ angle). */
    public static boolean ENCODER_REVERSED = false;

    /** Set to true if turning power needs to be flipped for all steering servos. */
    public static boolean SERVO_REVERSED = true;

    // ==================== OctoQuad Configuration ====================
    // Set USE_OCTOQUAD to true once you have the OctoQuad wired and configured.
    // When false, encoders are read from Control Hub motor encoder ports (the current setup).
    // When true, encoders are read from the OctoQuad I2C board.

    /** Set to true to read steering encoders from an OctoQuad instead of motor encoder ports. */
    public static boolean USE_OCTOQUAD = false;

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
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        @Override public int getPosition() { return motor.getCurrentPosition(); }
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
        @Override public int getPosition() { return octoquad.readSinglePosition(channel); }
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
        private double currentServoPower = 0;
        private double currentMotorPower = 0;

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

            // Swerve angle optimization: if error > 90 deg, invert motor direction and turn <= 90 deg
            double drivePower = commandedPower;
            if (Math.abs(error) > Math.PI / 2.0) {
                if (error > 0) {
                    error -= Math.PI;
                } else {
                    error += Math.PI;
                }
                drivePower = -drivePower;
            }

            // Delta time calculation
            long nowNs = System.nanoTime();
            double dt = (lastTimeNs > 0) ? (nowNs - lastTimeNs) / 1e9 : 0.02;
            lastTimeNs = nowNs;
            if (dt <= 0 || dt > 0.5) {
                dt = 0.02;
                // Time gap too large — don't trust old derivative/integral
                integral = 0;
                lastError = error;
            }

            // PID calculation
            integral += error * dt;
            integral = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integral));

            double derivative = (error - lastError) / dt;
            lastError = error;

            double turnPower = (kP * error) + (kI * integral) + (kD * derivative);

            // Add static friction compensation: minimum power to get the servo moving
            if (Math.abs(error) > STEER_TOLERANCE) {
                turnPower += Math.signum(turnPower) * kStatic;
            }

            if (SERVO_REVERSED) {
                turnPower = -turnPower;
            }

            // Clamp servo power to [-1.0, 1.0]
            turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

            double finalDrivePower = Math.abs(drivePower) < DEADZONE ? 0 : drivePower;
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

            // Swerve angle optimization: turn <= 90 deg
            if (Math.abs(error) > Math.PI / 2.0) {
                if (error > 0) {
                    error -= Math.PI;
                } else {
                    error += Math.PI;
                }
            }

            // Delta time calculation
            long nowNs = System.nanoTime();
            double dt = (lastTimeNs > 0) ? (nowNs - lastTimeNs) / 1e9 : 0.02;
            lastTimeNs = nowNs;
            if (dt <= 0 || dt > 0.5) {
                dt = 0.02;
                integral = 0;
                lastError = error;
            }

            // PID calculation
            integral += error * dt;
            integral = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integral));

            double derivative = (error - lastError) / dt;
            lastError = error;

            double turnPower = (kP * error) + (kI * integral) + (kD * derivative);

            if (Math.abs(error) > STEER_TOLERANCE) {
                turnPower += Math.signum(turnPower) * kStatic;
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
            if (servo instanceof PwmControl) {
                ((PwmControl) servo).setPwmDisable();
            }
            currentServoPower = 0;
        }

        /** Enables PWM output to this servo so it can receive power commands. */
        public void enablePwm() {
            if (servo instanceof PwmControl) {
                ((PwmControl) servo).setPwmEnable();
            }
        }

        public void stopSteer() {
            servo.setPower(0);
            integral = 0;
            lastError = 0;
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

    public void init(HardwareMap hMap) {
        imu = hMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        imu.resetYaw();

        double halfL = WheelbaseLength / 2.0, halfW = WheelbaseWidth / 2.0;

        // Build encoder sources: OctoQuad (I2C) or motor encoder ports
        SteeringEncoder redEnc, blueEnc, greenEnc, yellowEnc;
        if (USE_OCTOQUAD) {
            OctoQuad oq = hMap.get(OctoQuad.class, OCTOQUAD_NAME);
            redEnc    = new OctoQuadEncoder(oq, RED_ENCODER_CHANNEL);
            blueEnc   = new OctoQuadEncoder(oq, BLUE_ENCODER_CHANNEL);
            greenEnc  = new OctoQuadEncoder(oq, GREEN_ENCODER_CHANNEL);
            yellowEnc = new OctoQuadEncoder(oq, YELLOW_ENCODER_CHANNEL);
        } else {
            redEnc    = new MotorPortEncoder(hMap.get(DcMotor.class, "redServoEncoder"));
            blueEnc   = new MotorPortEncoder(hMap.get(DcMotor.class, "blueServoEncoder"));
            greenEnc  = new MotorPortEncoder(hMap.get(DcMotor.class, "greenServoEncoder"));
            yellowEnc = new MotorPortEncoder(hMap.get(DcMotor.class, "yellowServoEncoder"));
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

    public void resetYaw() {
        if (imu != null) imu.resetYaw();
    }

    public Module[] getModules() {
        return modules;
    }

    public void stop() {
        if (modules != null) {
            for (Module m : modules) {
                m.stop();
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
        // No stick input: stop the motors and relax servos
        if (Math.abs(strafeX) < DEADZONE && Math.abs(strafeY) < DEADZONE && Math.abs(rotate) < DEADZONE) {
            for (Module m : modules) m.stop();
            return;
        }

        enablePwm();

        // Field-centric: turn the stick direction by the opposite of the robot's heading.
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double driveX = strafeX * Math.cos(-yaw) - strafeY * Math.sin(-yaw);
        double driveY = strafeX * Math.sin(-yaw) + strafeY * Math.cos(-yaw);

        // Calculate push vector at each module
        double halfDiagonal = Math.hypot(WheelbaseLength, WheelbaseWidth) / 2.0;
        double[] power = new double[modules.length];
        double[] angle = new double[modules.length];
        double maxPower = 1.0;

        for (int i = 0; i < modules.length; i++) {
            Module m = modules[i];
            double pushX = driveX + rotate * (m.y / halfDiagonal);
            double pushY = driveY - rotate * (m.x / halfDiagonal);
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
        if (Math.abs(strafeX) < DEADZONE && Math.abs(strafeY) < DEADZONE && Math.abs(rotate) < DEADZONE) {
            for (Module m : modules) m.stop();
            return;
        }

        enablePwm();

        double driveX = strafeX;
        double driveY = strafeY;

        double halfDiagonal = Math.hypot(WheelbaseLength, WheelbaseWidth) / 2.0;
        double[] power = new double[modules.length];
        double[] angle = new double[modules.length];
        double maxPower = 1.0;

        for (int i = 0; i < modules.length; i++) {
            Module m = modules[i];
            double pushX = driveX + rotate * (m.y / halfDiagonal);
            double pushY = driveY - rotate * (m.x / halfDiagonal);
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
