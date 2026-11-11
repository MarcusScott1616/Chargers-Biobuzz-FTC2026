package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Turret - a single turret that aims at an AprilTag cluster, calculates distance,
 * and runs a flywheel + hood to shoot a ball into the HIVE.
 *
 * This replaces NectarTurret and PollenTurret - both were 95% identical. The only
 * difference between Nectar and Pollen is the hardware names and the shot table.
 * Use Turret.nectar() and Turret.pollen() for convenience.
 *
 * Every loop, update(alliance) does three jobs:
 *   1. CLUSTER:  use TagClusters to find which group of AprilTags the camera sees,
 *                filtered by our alliance ("red" or "blue").
 *   2. AIM:      nudge the turret SERVO until one of the cluster's tags is centered
 *                in the turret's HuskyLens.
 *   3. DISTANCE: the STATIONARY HuskyLens's up/down angle to a cluster tag (ty) + trig
 *                = how far away the HIVE is.
 *   4. SHOT:     look the distance up in the shot table -> hood angle + flywheel RPM.
 *
 * CAMERAS: the OpMode calls update() on each TagCamera ONCE per loop, BEFORE this
 * class's update().
 *   distanceCam = the stationary HuskyLens (shared by both turrets)
 *   aimCam      = the HuskyLens on this turret (null = this turret has no aim camera)
 *
 * This is a SKELETON. Anything marked "TUNE:" is a guess or a placeholder.
 */
public class Turret {

    // ======================= SHOT TABLES =======================
    // {distance in, hood angle deg, flywheel RPM} - from TurretSim.
    // TUNE: these are a STARTING POINT. Use TurretTestTeleOp's manual mode: park at each
    // distance, adjust RPM + hood until shots stay in, and write the real numbers here.

    /** Nectar (big 3.6 in blue ball), 14 in launch height (back turret). */
    public static final double[][] NECTAR_SHOT_TABLE = {
            { 36, 64.5, 2500},   // 88%
            { 48, 53.0, 2850},   // 88%
            { 60, 47.5, 3050},   // 90%
            { 72, 40.5, 3525},   // 91%
            { 84, 34.5, 4025},   // 90%
            { 96, 32.5, 4150},   // 82%
            {108, 31.5, 4175},   // 65%
            {120, 30.0, 4300},   // 46%
            {132, 28.0, 4525},   // 27% - this far out, drive closer!
    };

    /** Pollen (small 2.8 in yellow ball), 12 in launch height (front turret). */
    public static final double[][] POLLEN_SHOT_TABLE = {
            { 36, 65.0, 2550},   // 89%
            { 48, 64.0, 2650},   // 89%
            { 60, 47.5, 3150},   // 91%
            { 72, 40.5, 3625},   // 93%
            { 84, 35.0, 4050},   // 91%
            { 96, 31.0, 4475},   // 89%
            {108, 30.5, 4300},   // 77%
            {120, 30.0, 4375},   // 61%
            {132, 26.0, 4900},   // 43% - this far out, drive closer!
    };

    // ======================= CAMERA / DISTANCE (TUNE these) =======================

    // Measure on the robot. They turn the stationary camera's "ty" into a distance:
    //     distance = (tag height - camera height) / tan(camera tilt + ty)
    public double cameraHeightIn = 12.0;       // floor to the center of the stationary HuskyLens lens
    public double cameraTiltDeg  = 25.0;       // how far the stationary camera is tilted UP from flat
    public double tagHeightIn    = 45.0;       // floor to the CENTER of the AprilTag on the HIVE
    public double distanceOffsetIn = 0.0;      // + = the shot distance is farther than camera-to-tag

    // ======================= AIMING (turret left/right, SERVO) =======================

    // TUNE: start with kD = 0 and raise kP until the turret snaps onto the tag and wobbles
    // a little, then add kD to stop the wobble. (TurretTestTeleOp lets you tune with the gamepad.)
    public double aimKP = 0.2;
    public double aimKD = 0.02;
    public double aimToleranceDeg = 1.0;       // "close enough" to shoot
    public double aimOffsetDeg    = 0.0;       // TUNE: aim left/right of the tag if shots drift sideways

    // TUNE: servo centre, travel, direction, and limits.
    public double  turretServoCenter      = 0.5;
    public double  turretDegPerServoUnit  = 300;    // (servo travel deg) x (gear ratio)
    public boolean turretServoReversed    = false;  // flip if turret turns AWAY from the tag
    public double  turretLimitDeg         = 150;    // max travel either side of straight ahead

    // ======================= FLYWHEEL =======================

    // goBILDA 5203 6000 RPM (1:1 gearbox): 28 ticks per motor turn, flywheel 1:1.
    public static final double TICKS_PER_REV  = 28;
    public static final double MOTOR_FREE_RPM = 6000;

    // TUNE: flywheel PIDF. F does most of the work, then P fixes what's left.
    public double flyKF = 1.0 / MOTOR_FREE_RPM;
    public double flyKP = 0.0005;
    public double flyKI = 0.0;
    public double flyKD = 0.0;
    public double rpmTolerance = 75;               // "at speed" if within this many RPM
    public double idleRpm      = 1500;             // TUNE: keep spinning between shots

    // ======================= HOOD =======================

    // TUNE: hood angle = the angle the ball LEAVES at (0 = flat, 90 = straight up).
    public double hoodMinDeg = 25, hoodServoAtMin = 0.10;
    public double hoodMaxDeg = 65, hoodServoAtMax = 0.90;

    // ======================= TAG TIMEOUT =======================

    // If the camera loses the tag for longer than this, clear the stale distance so the
    // turret doesn't keep shooting at a position it drove away from.
    public double tagTimeoutSeconds = 0.5;

    // ======================= HARDWARE NAMES =======================

    private final String turretServoName;
    private final String flywheelMotorName;
    private final String hoodServoName;

    // ======================= SHOT TABLE =======================

    /** {distance in, hood angle deg, flywheel RPM}. Sorted by distance; blended between rows. */
    public double[][] shotTable;

    // ======================= STATE (internal) =======================

    private Servo turretServo, hoodServo;
    private DcMotorEx flywheel;
    private TagCamera distanceCam;     // stationary camera, for distance
    private TagCamera aimCam;          // turret camera, for aiming (null if this turret has none)
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime lastTagSeenTimer = new ElapsedTime();

    private boolean flywheelOn  = false;
    private boolean manualMode  = false;
    private double  manualRpm   = 3000, manualHoodDeg = 45;

    private boolean tagVisible   = false;
    private String  clusterName  = null;     // which cluster is in view ("blue 1", "red 2", etc.)
    private double  aimErrorDeg  = 0, lastAimError = 0;
    private double  turretCmdDeg = 0;        // the angle we ASKED the turret servo for (0 = straight ahead)
    private double  distanceIn   = -1;       // -1 = never seen a tag
    private double  targetRpm = 0, currentRpm = 0, targetHoodDeg = 45;
    private double  flyIntegral = 0, lastFlyError = 0;

    // ======================= CONSTRUCTION =======================

    /**
     * @param turretServoName  servo that spins the turret left/right
     * @param flywheelMotorName goBILDA 5203 6000 RPM motor, 1:1 to the flywheel
     * @param hoodServoName    servo that sets the hood angle
     * @param shotTable        {distance in, hood deg, RPM} rows, sorted by distance
     */
    public Turret(String turretServoName, String flywheelMotorName, String hoodServoName,
                  double[][] shotTable) {
        this.turretServoName  = turretServoName;
        this.flywheelMotorName = flywheelMotorName;
        this.hoodServoName    = hoodServoName;
        this.shotTable        = shotTable;
    }

    /** Convenience: create the Nectar turret (big blue ball, back turret). */
    public static Turret nectar() {
        return new Turret("nectarTurret", "nectarFlywheel", "nectarHood", NECTAR_SHOT_TABLE);
    }

    /** Convenience: create the Pollen turret (small yellow ball, front turret). */
    public static Turret pollen() {
        return new Turret("pollenTurret", "pollenFlywheel", "pollenHood", POLLEN_SHOT_TABLE);
    }

    // ======================= INIT =======================

    /**
     * @param distanceCam the stationary camera (used for distance)
     * @param aimCam      the camera on this turret (used for aiming), or null if there isn't one
     */
    public void init(HardwareMap hardwareMap, TagCamera distanceCam, TagCamera aimCam) {
        this.distanceCam = distanceCam;
        this.aimCam      = aimCam;

        turretServo = hardwareMap.get(Servo.class, turretServoName);
        turretCmdDeg = 0;
        setTurretAngle(turretCmdDeg);                   // start pointing straight ahead

        flywheel = hardwareMap.get(DcMotorEx.class, flywheelMotorName);
        flywheel.setDirection(DcMotor.Direction.FORWARD);    // TUNE: flip if it spins backwards
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);     // our own PIDF below
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hoodServo = hardwareMap.get(Servo.class, hoodServoName);
        setHoodAngle(targetHoodDeg);

        loopTimer.reset();
        lastTagSeenTimer.reset();
    }

    // ======================= UPDATE (call once per loop) =======================

    /**
     * Call once every loop, AFTER the OpMode has called update() on the cameras.
     * @param alliance "red" or "blue" - only that alliance's clusters are considered.
     *                 null = consider all clusters (useful for testing).
     */
    public void update(String alliance) {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        readCameras(alliance);
        aim(dt);
        runShot(dt);
    }

    // ---------- 1: find the cluster, read aim error and distance ----------

    private void readCameras(String alliance) {
        tagVisible  = false;
        clusterName = null;

        // Find which cluster of tags is visible, filtered by alliance.
        // Try the aim camera first; fall back to the distance camera.
        int[] targetTagIds = null;

        if (aimCam != null) {
            TagClusters.Result cluster = TagClusters.identify(aimCam.getBlocks(), alliance);
            if (cluster.name != null) {
                clusterName  = cluster.name;
                targetTagIds = cluster.tagIds;
            }
        }
        if (targetTagIds == null) {
            TagClusters.Result cluster = TagClusters.identify(distanceCam.getBlocks(), alliance);
            if (cluster.name != null) {
                clusterName  = cluster.name;
                targetTagIds = cluster.tagIds;
            }
        }

        if (targetTagIds == null) {
            // No cluster in view. If stale for too long, clear the old distance.
            if (lastTagSeenTimer.seconds() > tagTimeoutSeconds) {
                distanceIn = -1;
            }
            return;
        }

        // Aim: from the camera on this turret. tx = how far left/right the tag is.
        if (aimCam != null) {
            TagCamera.Tag tag = aimCam.findFirstTag(targetTagIds);
            if (tag != null) {
                tagVisible  = true;
                aimErrorDeg = tag.txDeg - aimOffsetDeg;
                lastTagSeenTimer.reset();
            }
        }

        // Distance: from the stationary camera. ty = how far up/down the tag is.
        TagCamera.Tag far = distanceCam.findFirstTag(targetTagIds);
        if (far != null) {
            double angleToTag = Math.toRadians(cameraTiltDeg + far.tyDeg);
            if (angleToTag > 0.01) {
                distanceIn = (tagHeightIn - cameraHeightIn) / Math.tan(angleToTag) + distanceOffsetIn;
                lastTagSeenTimer.reset();
            }
        }
    }

    // ---------- 2: turn the turret (servo, PD on tx, now framerate-independent) ----------

    private void aim(double dt) {
        if (!tagVisible) {               // no tag or no aim camera: hold still.
            lastAimError = 0;
            return;
        }

        double error = aimErrorDeg;
        double derivative = dt > 0 ? (error - lastAimError) / dt : 0;
        double change = Math.abs(error) < aimToleranceDeg ? 0
                : aimKP * error + aimKD * derivative;
        lastAimError = error;

        // Soft limit so the wires don't wrap.
        turretCmdDeg = Range.clip(turretCmdDeg + change, -turretLimitDeg, turretLimitDeg);
        setTurretAngle(turretCmdDeg);
    }

    /** Turn a turret angle (deg, + = right) into a servo position. */
    private void setTurretAngle(double deg) {
        double units = deg / turretDegPerServoUnit;
        turretServo.setPosition(
                Range.clip(turretServoCenter + (turretServoReversed ? -units : units), 0, 1));
    }

    // ---------- 3: hood + flywheel ----------

    private void runShot(double dt) {
        if (manualMode) {
            targetHoodDeg = manualHoodDeg;
            targetRpm = flywheelOn ? manualRpm : 0;
        } else if (distanceIn > 0) {
            double[] shot = lookUp(distanceIn);
            targetHoodDeg = shot[0];
            targetRpm = flywheelOn ? shot[1] : 0;
        } else {
            targetRpm = flywheelOn ? idleRpm : 0;      // haven't seen a tag yet
        }
        setHoodAngle(targetHoodDeg);

        // Flywheel speed from the encoder: ticks per second -> RPM.
        currentRpm = flywheel.getVelocity() / TICKS_PER_REV * 60;
        if (targetRpm <= 0) {
            flywheel.setPower(0);
            flyIntegral = 0;
            lastFlyError = 0;
            return;
        }
        double error = targetRpm - currentRpm;
        flyIntegral = Range.clip(flyIntegral + error * dt, -2000, 2000);
        double derivative = dt > 0 ? (error - lastFlyError) / dt : 0;
        lastFlyError = error;

        double power = flyKF * targetRpm + flyKP * error + flyKI * flyIntegral + flyKD * derivative;
        flywheel.setPower(Range.clip(power, 0, 1));      // never run it backwards to slow down
    }

    /** Find {hood deg, RPM} for a distance by blending the two nearest rows of shotTable. */
    private double[] lookUp(double distance) {
        double[][] t = shotTable;
        if (distance <= t[0][0]) return new double[]{t[0][1], t[0][2]};
        for (int i = 1; i < t.length; i++) {
            if (distance <= t[i][0]) {
                double f = (distance - t[i - 1][0]) / (t[i][0] - t[i - 1][0]);
                return new double[]{t[i - 1][1] + f * (t[i][1] - t[i - 1][1]),
                                    t[i - 1][2] + f * (t[i][2] - t[i - 1][2])};
            }
        }
        double[] last = t[t.length - 1];
        return new double[]{last[1], last[2]};
    }

    private void setHoodAngle(double deg) {
        deg = Range.clip(deg, hoodMinDeg, hoodMaxDeg);
        double f = (deg - hoodMinDeg) / (hoodMaxDeg - hoodMinDeg);
        hoodServo.setPosition(hoodServoAtMin + f * (hoodServoAtMax - hoodServoAtMin));
    }

    // ======================= PUBLIC API =======================

    /**
     * True when it's safe to feed a ball: flywheel at speed, and aimed.
     * A turret with no aim camera can't check its aim, so it only checks the speed.
     */
    public boolean readyToShoot() {
        boolean aimed = manualMode || aimCam == null
                || (tagVisible && Math.abs(aimErrorDeg) < aimToleranceDeg);
        return flywheelOn && aimed && targetRpm > 0
                && Math.abs(targetRpm - currentRpm) < rpmTolerance;
    }

    /** True when the distance is farther than the last row in the shot table (low accuracy). */
    public boolean isTooFar() {
        return distanceIn > 0 && shotTable.length > 0
                && distanceIn > shotTable[shotTable.length - 1][0];
    }

    public void setFlywheelOn(boolean on) { flywheelOn = on; }
    public boolean isFlywheelOn()         { return flywheelOn; }
    public void setManualMode(boolean on) { manualMode = on; }
    public boolean isManualMode()         { return manualMode; }
    public void setManualShot(double rpm, double hoodDeg) {
        manualRpm     = Range.clip(rpm, 0, MOTOR_FREE_RPM);
        manualHoodDeg = Range.clip(hoodDeg, hoodMinDeg, hoodMaxDeg);
    }
    public double getManualRpm()     { return manualRpm; }
    public double getManualHoodDeg() { return manualHoodDeg; }

    public boolean hasAimCamera()    { return aimCam != null; }
    public boolean isTagVisible()    { return tagVisible; }
    /** Which cluster the turret is currently locked onto, or null if none. */
    public String  getClusterName()  { return clusterName; }
    public double  getAimErrorDeg()  { return aimErrorDeg; }
    public double  getDistanceIn()   { return distanceIn; }
    public double  getTargetRpm()    { return targetRpm; }
    public double  getCurrentRpm()   { return currentRpm; }
    public double  getTargetHoodDeg(){ return targetHoodDeg; }
    /** The angle we COMMANDED the turret to (servo has no encoder, so this is not measured). */
    public double  getTurretDeg()    { return turretCmdDeg; }

    public void stop() {
        flywheelOn = false;
        flywheel.setPower(0);      // the turret servo just holds where it is
    }
}
