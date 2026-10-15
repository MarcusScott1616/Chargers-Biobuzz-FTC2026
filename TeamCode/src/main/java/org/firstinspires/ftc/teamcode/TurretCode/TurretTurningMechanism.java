package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Use the FTC SDK's AprilTag class (it has ftcPose; the org.openftc.apriltag one does not).
// In SDK 12, a single tag's id is on AprilTagSingleDetection, not the base AprilTagDetection.
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

/*
 * Turns the turret SERVO to center the AprilTag.
 * A servo has no encoder, so we keep our own "commanded angle" and nudge it toward the tag each loop.
 * When we stop sending changes, the servo holds its position.
 *
 * Hardware: "turret" = the turret SERVO (name must match the Driver Station robot configuration).
 */
public class TurretTurningMechanism {
    // AprilTag the turret aims at.
    private static final int TARGET_TAG_ID = 20;

    private Servo turret;

    // TUNE: kP = fraction of the aim error corrected each loop (0.2 = 20% per loop).
    // kD = damping on how fast the error is changing.
    private double kP = 0.2;

    private double kD = 0.02;

    private double goalX = 0;
    // Goal target, makes it easy to offset
    // for example, if we want to shoot towards the left of the april tag, adjust the goalX
    private double lastError = 0;

    private double angleTolerance = 0.2;
    // adjusts how "off" the turret can be, further away=less tolerance

    // TUNE: the servo position for "straight ahead". 0.5 is the middle of a 0-1 servo.
    private static final double SERVO_CENTER = 0.5;
    // TUNE: how many TURRET degrees the servo position moves across 0 -> 1 (servo travel x gear ratio).
    private static final double DEG_PER_SERVO_UNIT = 300;
    // TUNE: flip to true if the turret turns AWAY from the tag when you test it.
    private static final boolean SERVO_REVERSED = false;
    // Keeps the turret from wrapping its own wires (skip if you use a slip ring).
    private static final double TURRET_LIMIT_DEG = 150;

    // The angle we ASKED the servo for (0 = straight ahead). The servo has no encoder to read back.
    private double commandedDeg = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        turret = hwMap.get(Servo.class, "turret");
        commandedDeg = 0;
        setTurretAngle(commandedDeg);      // start pointing straight ahead
    }

    public void setkP(double newKP) {
        kP = newKP;
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double newKD) {
        kD = newKD;
    }

    public double getkD() {
        return kD;
    }

    /** The angle we COMMANDED the turret to (not a measured position). */
    public double getCommandedDeg() {
        return commandedDeg;
    }

    public void resetTimer() {
        timer.reset();
    }

    // Call with the AprilTag detection for this loop (or null if there isn't one).
    public void update(AprilTagSingleDetection curID) {
        // No tag, the wrong tag, or no pose info: hold still and wait.
        if (curID == null || curID.id != TARGET_TAG_ID || curID.ftcPose == null) {
            lastError = 0;
            return;
        }

        //------------- Starting PD controlling -----------------

        // "With A Limelight this might just be your TX"
        double error = goalX - curID.ftcPose.bearing;

        // Move a little each loop, in degrees (the servo itself is set below).
        double change = 0;
        if (Math.abs(error) >= angleTolerance) {
            change = kP * error + kD * (error - lastError);
        }
        lastError = error;

        // Soft limit so the wires don't wrap.
        commandedDeg = Range.clip(commandedDeg + change, -TURRET_LIMIT_DEG, TURRET_LIMIT_DEG);
        setTurretAngle(commandedDeg);
    }

    /** Turn a turret angle (deg, + = right) into a servo position. */
    private void setTurretAngle(double deg) {
        double units = deg / DEG_PER_SERVO_UNIT;
        turret.setPosition(Range.clip(SERVO_CENTER + (SERVO_REVERSED ? -units : units), 0, 1));
    }
}
