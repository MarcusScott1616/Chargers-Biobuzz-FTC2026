package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * SwerveDriveV2 - field-centric swerve drive.
 * Call init() once, then move(strafeX, strafeY, rotate) every loop.
 *
 * How it works:
 *   1. Every wheel uses the same math with ITS OWN position on the robot:
 *        wheel push = (drive push) + (spin push at that wheel's spot).
 *   2. A goBILDA servo turns about 300 degrees, not a full circle. For each target we also
 *      check the opposite direction (the motor runs backwards), keep only the angles the servo
 *      can reach, and pick the one closest to where the wheel already is.
 *   3. If the sticks are (almost) centered, the motors stop and the wheels keep their angle.
 */
public class SwerveDriveV2 {

    /*
     * RED--------BLU      front of robot is at the top
     *  |          |
     *  |          |
     * GRN--------YLW
     */

    /** Distance between front and back wheels, and between left and right wheels (inches). */
    public double WheelbaseLength = 17;
    public double WheelbaseWidth = 17;

    /** How many degrees the steering servo turns from position 0.0 to 1.0 (goBILDA: about 300). */
    public static double SERVO_RANGE_DEGREES = 300;

    /** Stick inputs smaller than this are treated as zero. */
    public static double DEADZONE = 0.05;

    /** One corner of the drivetrain: a steering servo and a drive motor. */
    private static class Module {
        final ServoImplEx servo;
        final DcMotor motor;
        final double x, y;          // wheel position from robot center: +x = right, +y = forward
        double angle = 0;           // angle we last told the wheel to point (radians, 0 = forward, + = right)

        Module(ServoImplEx servo, DcMotor motor, double x, double y) {
            this.servo = servo; this.motor = motor; this.x = x; this.y = y;
        }
    }

    private Module[] modules;
    public ServoImplEx[] servoList;
    private IMU imu;

    public void init(HardwareMap hMap) {
        imu = hMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        imu.resetYaw();
        //I'm not sure what our IMU and REV HUB orientation is going to look like so far.

        double halfL = WheelbaseLength / 2, halfW = WheelbaseWidth / 2;
        modules = new Module[]{
                new Module(hMap.get(ServoImplEx.class, "redServo"),    hMap.get(DcMotor.class, "redMotor"),    -halfW,  halfL),
                new Module(hMap.get(ServoImplEx.class, "blueServo"),   hMap.get(DcMotor.class, "blueMotor"),    halfW,  halfL),
                new Module(hMap.get(ServoImplEx.class, "greenServo"),  hMap.get(DcMotor.class, "greenMotor"),  -halfW, -halfL),
                new Module(hMap.get(ServoImplEx.class, "yellowServo"), hMap.get(DcMotor.class, "yellowMotor"),  halfW, -halfL),
        };
        servoList = new ServoImplEx[modules.length];

        for (int i = 0; i < modules.length; i++) {
            Module m = modules[i];
            servoList[i] = m.servo;
            m.servo.setPwmEnable();
            m.servo.setPwmRange(new PwmControl.PwmRange(500, 2500)); // goBILDA servo full range
            m.servo.setDirection(Servo.Direction.FORWARD);
            m.servo.setPosition(angleToServo(0));

            m.motor.setDirection(DcMotor.Direction.FORWARD);
            m.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    /**
     * Drives the robot, field-centric (pushing "forward" always drives away from the driver).
     *
     * @param strafeX left (-1) to right (+1)
     * @param strafeY back (-1) to forward (+1)
     * @param rotate  counter-clockwise (-1) to clockwise (+1)
     */
    public void move(float strafeX, float strafeY, float rotate) {
        // No stick input: stop the motors but leave the wheels where they are.
        if (Math.abs(strafeX) < DEADZONE && Math.abs(strafeY) < DEADZONE && Math.abs(rotate) < DEADZONE) {
            for (Module m : modules) m.motor.setPower(0);
            return;
        }

        // Field-centric: turn the stick direction by the opposite of the robot's heading.
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double driveX = strafeX * Math.cos(-yaw) - strafeY * Math.sin(-yaw);
        double driveY = strafeX * Math.sin(-yaw) + strafeY * Math.cos(-yaw);

        // Each wheel's push = drive push + spin push at that wheel's position.
        // Spinning clockwise, a wheel's push is (y, -x) - sideways to the line from the center
        // to the wheel. Dividing by the half-diagonal makes the corner wheels' spin push = 1.
        double halfDiagonal = Math.hypot(WheelbaseLength, WheelbaseWidth) / 2;
        double[] power = new double[modules.length];
        double[] angle = new double[modules.length];
        double maxPower = 1.0;
        for (int i = 0; i < modules.length; i++) {
            Module m = modules[i];
            double pushX = driveX + rotate * (m.y / halfDiagonal);
            double pushY = driveY - rotate * (m.x / halfDiagonal);
            power[i] = Math.hypot(pushX, pushY);
            angle[i] = Math.atan2(pushX, pushY); // 0 = forward, + = toward the right
            maxPower = Math.max(maxPower, power[i]);
        }

        // If any wheel would need more than full power, slow ALL wheels down by the same amount
        // so they keep pushing in the right ratio.
        for (int i = 0; i < modules.length; i++) {
            setModule(modules[i], angle[i], power[i] / maxPower);
        }
    }

    /** Point the wheel at a reachable angle closest to where it already is. */
    private void setModule(Module m, double targetAngle, double targetPower) {
        double limit = Math.toRadians(SERVO_RANGE_DEGREES / 2); // servo reaches -limit .. +limit
        double bestAngle = m.angle, bestPower = 0, bestTurn = Double.MAX_VALUE;

        // Check the target, the opposite direction (with the motor reversed), and their full-circle twins.
        for (int k = -2; k <= 2; k++) {
            double candidate = targetAngle + k * Math.PI;
            if (Math.abs(candidate) > limit) continue;           // servo can't get there
            double turn = Math.abs(candidate - m.angle);
            if (turn < bestTurn) {
                bestTurn = turn;
                bestAngle = candidate;
                bestPower = (k % 2 == 0) ? targetPower : -targetPower; // odd k = facing backwards
            }
        }

        m.angle = bestAngle;
        m.servo.setPosition(angleToServo(bestAngle));
        m.motor.setPower(bestPower);
    }

    /** Wheel angle (radians, 0 = forward) -> servo position (0..1, 0.5 = forward). */
    private static double angleToServo(double angle) {
        double position = 0.5 + Math.toDegrees(angle) / SERVO_RANGE_DEGREES;
        return Math.max(0, Math.min(1, position));
    }
}
