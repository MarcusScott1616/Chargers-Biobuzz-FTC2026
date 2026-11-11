package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * SwerveDriveV2 - field-centric coaxial swerve drive with continuous rotation servos
 * and REV Through-Bore encoders (in absolute analog mode).
 *
 * Architecture:
 *   - Steering: Continuous Rotation Servos (CRServo) driven by a closed-loop PID.
 *   - Feedback: REV Through-Bore absolute encoders connected to AnalogInput ports (0V - 3.3V).
 *   - Drive: DcMotors with BRAKE mode.
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

    /** When true, CRServos are powered down (0 power) when joysticks are idle to prevent jitter and heating. */
    public static boolean RELAX_SERVOS_WHEN_IDLE = true;

    /** Zero-angle offsets (in degrees) for each module when pointing straight forward (0 rad). */
    public static double RED_OFFSET_DEG = 0.0;
    public static double BLUE_OFFSET_DEG = 0.0;
    public static double GREEN_OFFSET_DEG = 0.0;
    public static double YELLOW_OFFSET_DEG = 0.0;

    /** Set to true if an encoder's voltage decreases when the pod turns to the right (+ angle). */
    public static boolean ENCODER_REVERSED = false;

    /** Set to true if turning power needs to be flipped for all steering servos. */
    public static boolean SERVO_REVERSED = false;

    /** One corner of the drivetrain: a steering CRServo, a drive motor, and an absolute encoder. */
    public static class Module {
        public final String name;
        public final CRServo servo;
        public final DcMotor motor;
        public final AnalogInput encoder;
        public final double x, y;          // wheel position from robot center: +x = right, +y = forward
        public double offsetRad;
        public boolean encoderReversed;

        private double targetAngle = 0;
        private double lastError = 0;
        private double integral = 0;
        private long lastTimeNs = 0;
        private double currentServoPower = 0;
        private double currentMotorPower = 0;

        public Module(String name, CRServo servo, DcMotor motor, AnalogInput encoder,
                      double x, double y, double offsetRad, boolean encoderReversed) {
            this.name = name;
            this.servo = servo;
            this.motor = motor;
            this.encoder = encoder;
            this.x = x;
            this.y = y;
            this.offsetRad = offsetRad;
            this.encoderReversed = encoderReversed;
        }

        /** Reads raw encoder voltage (0.0 to ~3.3V). */
        public double getVoltage() {
            return encoder.getVoltage();
        }

        /** Reads raw angle in radians [0, 2*PI) directly from the absolute encoder. */
        public double getRawAngleRad() {
            double voltage = encoder.getVoltage();
            double maxVoltage = encoder.getMaxVoltage();
            if (maxVoltage <= 0) maxVoltage = 3.3;
            double fraction = Math.max(0.0, Math.min(1.0, voltage / maxVoltage));
            return fraction * 2.0 * Math.PI;
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
            if (dt <= 0 || dt > 0.5) dt = 0.02;

            // PID calculation
            integral += error * dt;
            integral = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integral));

            double derivative = (error - lastError) / dt;
            lastError = error;

            double turnPower = (kP * error) + (kI * integral) + (kD * derivative);
            if (SERVO_REVERSED) {
                turnPower = -turnPower;
            }

            // Clamp servo power to [-1.0, 1.0]
            turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

            if (Math.abs(commandedPower) < DEADZONE && RELAX_SERVOS_WHEN_IDLE) {
                // If robot is stationary, stop steering servos to prevent buzzing/heating
                servo.setPower(0);
                motor.setPower(0);
                integral = 0;
                currentServoPower = 0;
                currentMotorPower = 0;
            } else {
                servo.setPower(turnPower);
                motor.setPower(drivePower);
                currentServoPower = turnPower;
                currentMotorPower = drivePower;
            }
        }

        public void stop() {
            servo.setPower(0);
            motor.setPower(0);
            integral = 0;
            lastError = 0;
            currentServoPower = 0;
            currentMotorPower = 0;
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

        modules = new Module[]{
                new Module("red",
                        hMap.get(CRServo.class, "redServo"),
                        hMap.get(DcMotor.class, "redMotor"),
                        hMap.get(AnalogInput.class, "redServoEncoder"),
                        -halfW,  halfL, Math.toRadians(RED_OFFSET_DEG), ENCODER_REVERSED),
                new Module("blue",
                        hMap.get(CRServo.class, "blueServo"),
                        hMap.get(DcMotor.class, "blueMotor"),
                        hMap.get(AnalogInput.class, "blueServoEncoder"),
                         halfW,  halfL, Math.toRadians(BLUE_OFFSET_DEG), ENCODER_REVERSED),
                new Module("green",
                        hMap.get(CRServo.class, "greenServo"),
                        hMap.get(DcMotor.class, "greenMotor"),
                        hMap.get(AnalogInput.class, "greenServoEncoder"),
                        -halfW, -halfL, Math.toRadians(GREEN_OFFSET_DEG), ENCODER_REVERSED),
                new Module("yellow",
                        hMap.get(CRServo.class, "yellowServo"),
                        hMap.get(DcMotor.class, "yellowMotor"),
                        hMap.get(AnalogInput.class, "yellowServoEncoder"),
                         halfW, -halfL, Math.toRadians(YELLOW_OFFSET_DEG), ENCODER_REVERSED),
        };

        for (Module m : modules) {
            m.motor.setDirection(DcMotor.Direction.FORWARD);
            m.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.servo.setDirection(CRServo.Direction.FORWARD);
            m.stop();
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

    /** Normalizes an angle in radians to [-PI, PI]. */
    public static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }
}
