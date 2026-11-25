package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * PinpointOdometry - wraps the goBILDA Pinpoint Odometry Computer to provide
 * field-centric robot coordinates (X, Y, Heading) and calculate geometric distance
 * and turret heading to target field landmarks (like the Hives) when AprilTags are not visible.
 */
public class PinpointOdometry {

    private final GoBildaPinpointDriver pinpoint;

    // Field Landmark coordinates (in inches, on the 144" x 144" FTC field).
    // Calibrated to match the shooting positions and ~65" distance table:
    public static double RED_HIVE_1_X = 45.0, RED_HIVE_1_Y = 89.0;
    public static double RED_HIVE_2_X = 45.0, RED_HIVE_2_Y = 43.0;
    public static double BLUE_HIVE_1_X = 96.5, BLUE_HIVE_1_Y = 89.0;
    public static double BLUE_HIVE_2_X = 96.5, BLUE_HIVE_2_Y = 43.0;

    private double currentTargetX = RED_HIVE_1_X;
    private double currentTargetY = RED_HIVE_1_Y;

    public PinpointOdometry(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
    }

    /**
     * Initializes the Pinpoint driver using pod offsets and configuration from Constants.java.
     */
    public static PinpointOdometry init(HardwareMap hardwareMap, String deviceName) {
        GoBildaPinpointDriver driver = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
        driver.setOffsets(Constants.xOffset, Constants.yOffset, DistanceUnit.MM);
        driver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        driver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
        driver.resetPosAndIMU();
        return new PinpointOdometry(driver);
    }

    /** Call once per loop to read fresh sensor data from the Pinpoint computer. */
    public void update() {
        pinpoint.update();
    }

    /** Returns robot field X in inches. */
    public double getX() {
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    /** Returns robot field Y in inches. */
    public double getY() {
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    /** Returns robot heading in degrees [-180, 180]. */
    public double getHeadingDeg() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    /** Returns robot heading in radians [-PI, PI]. */
    public double getHeadingRad() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public void setPose(Pose2D pose) {
        pinpoint.setPosition(pose);
    }

    public void setPose(double xInches, double yInches, double headingDeg) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDeg));
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
    }

    /** Set the field coordinates of the current target (e.g. Hive). */
    public void setTarget(double xInches, double yInches) {
        this.currentTargetX = xInches;
        this.currentTargetY = yInches;
    }

    public double getTargetX() { return currentTargetX; }
    public double getTargetY() { return currentTargetY; }

    /** Select target Hive based on alliance and hive index. */
    public void setTargetByAlliance(String alliance, int hiveNumber) {
        if ("red".equalsIgnoreCase(alliance)) {
            if (hiveNumber == 2) {
                setTarget(RED_HIVE_2_X, RED_HIVE_2_Y);
            } else {
                setTarget(RED_HIVE_1_X, RED_HIVE_1_Y);
            }
        } else {
            if (hiveNumber == 2) {
                setTarget(BLUE_HIVE_2_X, BLUE_HIVE_2_Y);
            } else {
                setTarget(BLUE_HIVE_1_X, BLUE_HIVE_1_Y);
            }
        }
    }

    /** Straight-line Euclidean distance from robot position to current target in inches. */
    public double getDistanceToTarget() {
        return getDistanceTo(currentTargetX, currentTargetY);
    }

    /** Turret angle (+ = right, - = left) needed to point at the current target. */
    public double getTurretAngleToTarget() {
        return getTurretAngleTo(currentTargetX, currentTargetY);
    }

    /** Straight-line Euclidean distance from robot to given field coordinate in inches. */
    public double getDistanceTo(double targetX, double targetY) {
        double dx = targetX - getX();
        double dy = targetY - getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Relative turret angle (in degrees, + = right, - = left) to point at a field coordinate.
     * Takes robot orientation into account and converts from Counter-Clockwise (standard math/IMU)
     * to Clockwise-positive (Turret servo / camera coordinate system).
     */
    public double getTurretAngleTo(double targetX, double targetY) {
        double dx = targetX - getX();
        double dy = targetY - getY();
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = getHeadingDeg();

        // Convert CCW field angle difference to Clockwise-positive Turret coordinates (+ = right, - = left)
        double turretRelativeDeg = robotHeadingDeg - fieldAngleDeg;
        return normalizeDegrees(turretRelativeDeg);
    }

    public static double normalizeDegrees(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }

    public GoBildaPinpointDriver getDriver() {
        return pinpoint;
    }
}
