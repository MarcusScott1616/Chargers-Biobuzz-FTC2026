package swervesim;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

/*
 * Simple pretend-physics for the swerve robot.
 *
 * Every tick we:
 *   1. Turn each steering servo toward the position SwerveDrive asked for (servos take time to turn).
 *   2. Work out which way each wheel is pushing, and how hard (from its angle and motor power).
 *   3. Find the robot motion (slide + spin) that best matches all four wheel pushes.
 *   4. Measure how much the wheels DISAGREE with that motion. We call this "wheel fight".
 *      A good swerve drive keeps wheel fight near 0. Big wheel fight = wheels pushing against
 *      each other, which on a real robot means scrubbing, jittering, or not moving at all.
 *
 * This is NOT real physics (no weight, friction, or wheel slip) - it's just enough to see
 * whether the code's wheel commands make sense.
 *
 * Directions used everywhere in here (robot's point of view):
 *   +x = right, +y = forward.
 *   Wheel angle: 0 = pointing forward, positive = turned toward the right (clockwise).
 *   Spin (omega) and heading: counter-clockwise positive, same as the FTC IMU.
 */
public class Physics {

    // ---------------- Numbers you can tweak ----------------

    /** Degrees the steering servo REALLY turns from position 0.0 to 1.0.
     *  goBILDA servos: about 300. (The old SwerveDrive assumed 360 - press G in the sim to compare.) */
    public static double SERVO_RANGE_DEG = 300;

    /** How fast the steering servo turns, in degrees per second. */
    public static final double SERVO_SPEED_DEG_PER_SEC = 400;

    /** How fast a wheel drives at motor power 1.0, in inches per second. */
    public static final double MAX_WHEEL_SPEED = 60;

    /** An FTC field is 12 ft = 144 inches on a side. */
    public static final double FIELD_SIZE = 144;

    // --------------------------------------------------------

    /** One swerve module: a steering servo plus a drive motor. */
    public static class Module {
        public final String name;
        public final Color color;
        public final double x, y;           // where the wheel sits, in inches from the robot center
        public final ServoImplEx servo;
        public final DcMotor motor;

        public double servoPos = 0.5;       // where the servo physically IS (it lags behind the command)
        public double vx, vy;               // how this wheel is pushing (inches/sec, robot frame)

        Module(String name, Color color, double x, double y, ServoImplEx servo, DcMotor motor) {
            this.name = name; this.color = color; this.x = x; this.y = y; this.servo = servo; this.motor = motor;
        }

        /** Angle the wheel is pointing right now (radians). */
        public double angle() { return servoPosToAngle(servo, servoPos); }

        /** Angle SwerveDrive asked for (radians). The wheel catches up to this over time. */
        public double commandedAngle() { return servoPosToAngle(servo, servo.getPosition()); }

        /** Motor power, flipped if the motor is set to REVERSE. */
        public double wheelPower() {
            double p = motor.getPower();
            return motor.getDirection() == DcMotor.Direction.REVERSE ? -p : p;
        }

        private static double servoPosToAngle(Servo servo, double pos) {
            if (servo.getDirection() == Servo.Direction.REVERSE) pos = 1 - pos;
            return Math.toRadians((pos - 0.5) * SERVO_RANGE_DEG);
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
        // Must match the device names and layout in SwerveDrive:
        //   RED--------BLU
        //    |          |
        //   GRN--------YLW
        modules = new Module[]{
                new Module("red",    new Color(220, 50, 50),  -hw2,  hl, hw.get(ServoImplEx.class, "redServo"),    hw.get(DcMotor.class, "redMotor")),
                new Module("blue",   new Color(40, 100, 230),  hw2,  hl, hw.get(ServoImplEx.class, "blueServo"),   hw.get(DcMotor.class, "blueMotor")),
                new Module("green",  new Color(30, 160, 60),  -hw2, -hl, hw.get(ServoImplEx.class, "greenServo"),  hw.get(DcMotor.class, "greenMotor")),
                new Module("yellow", new Color(225, 180, 0),   hw2, -hl, hw.get(ServoImplEx.class, "yellowServo"), hw.get(DcMotor.class, "yellowMotor")),
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
    }

    /** Move the simulation forward by dt seconds. */
    public void step(double dt) {
        // 1 + 2: turn servos toward their targets, then find each wheel's push.
        double maxServoStep = SERVO_SPEED_DEG_PER_SEC / SERVO_RANGE_DEG * dt;
        for (Module m : modules) {
            double error = m.servo.getPosition() - m.servoPos;
            m.servoPos += Math.max(-maxServoStep, Math.min(maxServoStep, error));

            double speed = m.wheelPower() * MAX_WHEEL_SPEED;
            m.vx = speed * Math.sin(m.angle());
            m.vy = speed * Math.cos(m.angle());
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
