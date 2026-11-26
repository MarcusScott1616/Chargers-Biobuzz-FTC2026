package swervesim;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.SwerveCode.SwerveDriveV2;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

/*
 * Simple pretend-physics for the swerve robot.
 *
 * Every tick we:
 *   1. Turn each steering servo based on the CRServo power SwerveDriveV2 asked for.
 *   2. Update encoder ticks so SwerveDriveV2's PID can measure pod angle.
 *   3. Work out which way each wheel is pushing, and how hard (from its angle and motor power).
 *   4. Find the robot motion (slide + spin) that best matches all four wheel pushes.
 *   5. Measure how much the wheels DISAGREE with that motion ("wheel fight").
 */
public class Physics {

    // ---------------- Numbers you can tweak ----------------

    /** How fast the steering servo turns at full power (1.0), in degrees per second. */
    public static final double SERVO_SPEED_DEG_PER_SEC = 400;

    /** Legacy display range for simulator UI. */
    public static double SERVO_RANGE_DEG = 360;

    /**
     * Real pod gearing turns the pod opposite to raw positive servo shaft power (+power -> -angle),
     * matching SwerveDriveV2.SERVO_REVERSED = true and Constants.java CRServo.Direction.REVERSE.
     */
    public static final boolean PHYSICAL_STEER_INVERTED = true;

    /** How fast a wheel drives at motor power 1.0, in inches per second. */
    public static final double MAX_WHEEL_SPEED = 60;

    /** An FTC field is 12 ft = 144 inches on a side. */
    public static final double FIELD_SIZE = 144;

    // --------------------------------------------------------

    /** One swerve module: a steering CRServo, drive motor, and through-bore encoder. */
    public static class Module {
        public final String name;
        public final Color color;
        public final double x, y;           // where the wheel sits, in inches from the robot center
        public final CRServo servo;
        public final DcMotor motor;
        public final DcMotor encoder;

        public double currentAngleRad = 0;   // where the wheel physically IS
        public double vx, vy;               // how this wheel is pushing (inches/sec, robot frame)

        Module(String name, Color color, double x, double y, CRServo servo, DcMotor motor, DcMotor encoder) {
            this.name = name; this.color = color; this.x = x; this.y = y;
            this.servo = servo; this.motor = motor; this.encoder = encoder;
        }

        /** Angle the wheel is pointing right now (radians). */
        public double angle() { return currentAngleRad; }

        /** Angle SwerveDriveV2 asked for (radians). */
        public double commandedAngle() { return currentAngleRad; }

        /** Motor power, flipped if the motor is set to REVERSE. */
        public double wheelPower() {
            double p = motor.getPower();
            return motor.getDirection() == DcMotor.Direction.REVERSE ? -p : p;
        }
    }

    public final Module[] modules;
    private final IMU imu;
    private final double halfDiagonal;

    // Robot position on the field (inches). Field +y is "up" on the screen.
    public double x, y, heading;            // heading 0 = facing up the field
    public double vx, vy, omega;            // robot motion (robot frame, in/s and rad/s)
    public double wheelFight;               // in/s - how much the wheels disagree
    public final List<double[]> trail = new ArrayList<>();

    public Physics(HardwareMap hw, double wheelbaseLength, double wheelbaseWidth) {
        double hl = wheelbaseLength / 2, hw2 = wheelbaseWidth / 2;
        // Must match the device names and layout in SwerveDriveV2:
        //   RED--------BLU
        //    |          |
        //   GRN--------YLW
        modules = new Module[]{
                new Module("red",    new Color(220, 50, 50),  -hw2,  hl, hw.get(CRServo.class, "redServo"),    hw.get(DcMotor.class, "redMotor"),    hw.get(DcMotor.class, "redServoEncoder")),
                new Module("blue",   new Color(40, 100, 230),  hw2,  hl, hw.get(CRServo.class, "blueServo"),   hw.get(DcMotor.class, "blueMotor"),   hw.get(DcMotor.class, "blueServoEncoder")),
                new Module("green",  new Color(30, 160, 60),  -hw2, -hl, hw.get(CRServo.class, "greenServo"),  hw.get(DcMotor.class, "greenMotor"),  hw.get(DcMotor.class, "greenServoEncoder")),
                new Module("yellow", new Color(225, 180, 0),   hw2, -hl, hw.get(CRServo.class, "yellowServo"), hw.get(DcMotor.class, "yellowMotor"), hw.get(DcMotor.class, "yellowServoEncoder")),
        };
        imu = hw.get(IMU.class, "imu");
        halfDiagonal = Math.hypot(hl, hw2);
        reset();
    }

    /** Put the robot back in the middle of the field, facing up. */
    public void reset() {
        x = FIELD_SIZE / 2;
        y = FIELD_SIZE / 2;
        heading = 0;
        trail.clear();
        imu.simSetHeading(heading);
        if (modules != null) {
            for (Module m : modules) {
                m.currentAngleRad = 0;
                m.encoder.setCurrentPosition(0);
            }
        }
    }

    /** Move the simulation forward by dt seconds. */
    public void step(double dt) {
        // Turn servos based on CRServo power and update encoder ticks.
        // The encoder is on the servo shaft (before the gear reduction), so ticks are
        // written at the SERVO rate: ENCODER_TICKS_PER_REV ticks per servo revolution.
        // SwerveDriveV2 accounts for gear ratio when converting ticks -> pod angle.
        double maxTurnSpeedRad = Math.toRadians(SERVO_SPEED_DEG_PER_SEC);
        for (Module m : modules) {
            double servoPower = m.servo.getPower();
            if (m.servo.getDirection() == CRServo.Direction.REVERSE) servoPower = -servoPower;
            if (PHYSICAL_STEER_INVERTED) servoPower = -servoPower;
            m.currentAngleRad += servoPower * maxTurnSpeedRad * dt;

            // Write encoder ticks accounting for the gear ratio so SwerveDriveV2 reads
            // the correct pod angle. STEER_GEAR_RATIO pod revolutions per servo revolution
            // means the encoder (on the servo shaft) sees STEER_GEAR_RATIO more ticks
            // than a 1:1 setup for the same pod angle.
            double gearRatio = SwerveDriveV2.STEER_GEAR_RATIO;
            int ticks = (int) Math.round(
                    (m.currentAngleRad / (2.0 * Math.PI)) * SwerveDriveV2.ENCODER_TICKS_PER_REV * gearRatio);
            if ("red".equals(m.name)) {
                ticks = -ticks; // Match real robot: Red encoder ticks decrease when spun CW
            }
            m.encoder.setCurrentPosition(ticks);

            double speed = m.wheelPower() * MAX_WHEEL_SPEED;
            m.vx = speed * Math.sin(m.currentAngleRad);
            m.vy = speed * Math.cos(m.currentAngleRad);
        }

        // 3: best-fit robot motion. Slide = average wheel push.
        //    Spin = how much the wheels push "around" the center (like turning a steering wheel).
        double sumVx = 0, sumVy = 0, sumTurn = 0, sumR2 = 0;
        for (Module m : modules) {
            sumVx += m.vx;
            sumVy += m.vy;
            sumTurn += m.x * m.vy - m.y * m.vx;
            sumR2 += m.x * m.x + m.y * m.y;
        }
        vx = sumVx / modules.length;
        vy = sumVy / modules.length;
        omega = sumTurn / sumR2;

        // 4: wheel fight = how far each wheel's push is from what the robot is actually doing.
        double sumSq = 0;
        for (Module m : modules) {
            double ex = m.vx - (vx - omega * m.y);
            double ey = m.vy - (vy + omega * m.x);
            sumSq += ex * ex + ey * ey;
        }
        wheelFight = Math.sqrt(sumSq / modules.length);

        // Move the robot on the field (turn robot-frame motion into field motion).
        double cos = Math.cos(heading), sin = Math.sin(heading);
        x += (vx * cos - vy * sin) * dt;
        y += (vx * sin + vy * cos) * dt;
        x = Math.max(halfDiagonal, Math.min(FIELD_SIZE - halfDiagonal, x));
        y = Math.max(halfDiagonal, Math.min(FIELD_SIZE - halfDiagonal, y));
        heading += omega * dt;
        imu.simSetHeading(heading);

        double[] last = trail.isEmpty() ? null : trail.get(trail.size() - 1);
        if (last == null || Math.hypot(x - last[0], y - last[1]) > 0.5) {
            trail.add(new double[]{x, y});
            if (trail.size() > 3000) trail.remove(0);
        }
    }
}
