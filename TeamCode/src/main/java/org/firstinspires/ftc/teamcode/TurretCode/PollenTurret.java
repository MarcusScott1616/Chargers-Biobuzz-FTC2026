package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * PollenTurret - the turret that shoots POLLEN (the small 2.8 in yellow ball).
 * NectarTurret.java is the same idea for the big ball; each turret is tuned on its own.
 *
 * Every loop, update() does three jobs:
 *   1. AIM:      a PD loop turns the turret until the AprilTag is centered in the Limelight (tx = 0).
 *   2. DISTANCE: the Limelight's up/down angle to the tag (ty) + trig = how far away the HIVE is.
 *   3. SHOT:     look the distance up in SHOT_TABLE -> hood angle (servo) + flywheel RPM (PIDF loop).
 *
 * This is a SKELETON. Anything marked "TUNE:" is a guess or a placeholder - measure it on the robot.
 * Anything marked "LIMELIGHT:" is something you also have to set up in the Limelight web page.
 *
 * Hardware (names must match the Driver Station robot configuration):
 *   "pollenTurret"    - motor that spins the turret left/right (with encoder)
 *   "pollenFlywheel"  - goBILDA 5203 6000 RPM motor, 1:1 to the flywheel
 *   "pollenHood"      - servo that sets the hood angle
 *   The Limelight 3A is passed in to init(), so the OpMode decides which camera this turret uses.
 */
public class PollenTurret {

    // ======================= LIMELIGHT =======================

    // LIMELIGHT: open the Limelight web page (http://limelight.local:5801 while plugged into a laptop),
    // make an AprilTag pipeline in this slot, and set its family to the one in the game manual.
    public static int LIMELIGHT_PIPELINE = 0;

    // TUNE: the AprilTag on the HIVE we aim at. Check the game manual for the real IDs.
    public static int TARGET_TAG_ID = 20;

    // TUNE: measure these on the robot. They turn the Limelight's "ty" into a distance:
    //     distance = (tag height - camera height) / tan(camera tilt + ty)
    public static double CAMERA_HEIGHT_IN = 12.0;      // floor to the center of the Limelight lens
    public static double CAMERA_TILT_DEG = 25.0;       // how far the camera is tilted UP from flat
    public static double TAG_HEIGHT_IN = 45.0;         // floor to the CENTER of the AprilTag on the HIVE

    // TUNE: the shot table uses the simulator's distance = floor distance from where the ball leaves
    // the shooter to the bottom edge of the HIVE opening. The camera measures camera-to-tag instead.
    // Put a tape measure down, compare, and put the difference here (+ = the shot distance is farther).
    public static double DISTANCE_OFFSET_IN = 0.0;

    // ======================= AIMING (turret left/right) =======================

    // TUNE: start with kD = 0 and raise kP until the turret snaps onto the tag and wobbles a little,
    // then add kD to stop the wobble. (TurretTestTeleOp lets you change these with the gamepad.)
    public double aimKP = 0.02;                        // motor power per degree of error
    public double aimKD = 0.001;
    public static double AIM_TOLERANCE_DEG = 1.0;      // "close enough" to shoot
    public static double AIM_MAX_POWER = 0.6;
    public static double AIM_OFFSET_DEG = 0.0;         // TUNE: aim a bit left/right of the tag if shots drift sideways

    // TUNE: keeps the turret from wrapping its own wires (skip if you use a slip ring).
    // Turn the turret by hand 90 degrees and read the encoder to find TICKS_PER_DEGREE.
    public static double TURRET_TICKS_PER_DEGREE = 3.0;
    public static double TURRET_LIMIT_DEG = 150;       // turret can go this far either way from where it started

    // ======================= FLYWHEEL =======================

    // goBILDA 5203 6000 RPM (1:1 gearbox): the encoder gives 28 ticks per motor turn.
    // The flywheel is 1:1 with the motor, so motor RPM = flywheel RPM.
    public static final double TICKS_PER_REV = 28;
    public static final double MOTOR_FREE_RPM = 6000;

    // TUNE: flywheel PIDF. F does most of the work: power = RPM / max RPM gets you close,
    // then P fixes what's left. Tune F first (kP = 0): raise it until the RPM settles just
    // below the target, then add kP until it reaches the target quickly without bouncing.
    public double flyKF = 1.0 / MOTOR_FREE_RPM;          // power per RPM of target
    public double flyKP = 0.0005;                        // power per RPM of error
    public double flyKI = 0.0;                           // usually leave at 0
    public double flyKD = 0.0;                           // usually leave at 0
    public static double RPM_TOLERANCE = 75;             // "at speed" if within this many RPM
    public static double IDLE_RPM = 1500;                // TUNE: keep spinning between shots so spin-up is faster

    // ======================= HOOD =======================

    // TUNE: hood angle = the angle the ball LEAVES at (0 = flat, 90 = straight up), same as the simulator.
    // Set the servo to HOOD_SERVO_AT_MIN, measure the launch angle, then do the same for HOOD_SERVO_AT_MAX.
    public static double HOOD_MIN_DEG = 25, HOOD_SERVO_AT_MIN = 0.10;
    public static double HOOD_MAX_DEG = 65, HOOD_SERVO_AT_MAX = 0.90;

    // ======================= SHOT TABLE =======================

    // {distance in, hood angle deg, flywheel RPM} - from TurretSim (Pollen, 0.64 bounciness, 14 in launch
    // height, 4 in flywheel, wobble +/-75 RPM, +/-1 deg, +/-3 in). The % is the sim's chance to stay in.
    // TUNE: these are a STARTING POINT. Use TurretTestTeleOp's manual mode: park at each distance,
    // adjust RPM + hood until shots stay in, and write the real numbers here.
    // Keep the rows sorted by distance. Between rows the code blends the two nearest rows.
    public static double[][] SHOT_TABLE = {
            { 36, 65.0, 2500},   // 90%
            { 48, 65.0, 2600},   // 88%
            { 60, 47.5, 3050},   // 92%
            { 72, 40.0, 3550},   // 93%
            { 84, 32.5, 4300},   // 91%
            { 96, 30.0, 4500},   // 87%
            {108, 30.0, 4300},   // 74%
            {120, 30.0, 4250},   // 54%
            {132, 27.5, 4550},   // 36% - this far out, drive closer!
    };

    // ======================= state =======================

    private DcMotorEx turret, flywheel;
    private Servo hood;
    private Limelight3A limelight;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean flywheelOn = false;
    private boolean manualMode = false;                 // true = use manualRpm/manualHoodDeg instead of the table
    private double manualRpm = 3000, manualHoodDeg = 45;

    private boolean tagVisible = false;
    private double aimErrorDeg = 0, lastAimError = 0;
    private double distanceIn = -1;                     // -1 = never seen the tag
    private double targetRpm = 0, currentRpm = 0, targetHoodDeg = 45;
    private double flyIntegral = 0, lastFlyError = 0;

    public void init(HardwareMap hardwareMap, Limelight3A limelight) {
        this.limelight = limelight;

        turret = hardwareMap.get(DcMotorEx.class, "pollenTurret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    // wherever it starts = 0 degrees
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);       // we do our own PD, but still read the encoder
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel = hardwareMap.get(DcMotorEx.class, "pollenFlywheel");
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);    // TUNE: flip if it spins backwards
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);     // our own PIDF below
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast down, don't slam the brakes

        hood = hardwareMap.get(Servo.class, "pollenHood");
        setHoodAngle(targetHoodDeg);

        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.setPollRateHz(100);
        limelight.start();      // safe to call twice if both turrets share one Limelight
        timer.reset();
    }

    /** Call once every loop. */
    public void update() {
        double dt = timer.seconds();
        timer.reset();

        readLimelight();
        aim(dt);
        runShot(dt);
    }

    // ---------- 1 + 2: read the tag, work out aim error and distance ----------

    private void readLimelight() {
        tagVisible = false;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() != TARGET_TAG_ID) continue;    // ignore other tags
            tagVisible = true;

            // tx: how far left/right the tag is (degrees). If the Limelight is mounted ON the turret,
            // this is exactly how far the turret still has to turn.
            // LIMELIGHT: keep the crosshair centered in the web page, or tx/ty will be offset.
            aimErrorDeg = tag.getTargetXDegrees() - AIM_OFFSET_DEG;

            // ty: how far up/down the tag is. Add the camera's tilt and use trig for the distance.
            double angleToTag = Math.toRadians(CAMERA_TILT_DEG + tag.getTargetYDegrees());
            if (angleToTag > 0.01) {
                distanceIn = (TAG_HEIGHT_IN - CAMERA_HEIGHT_IN) / Math.tan(angleToTag) + DISTANCE_OFFSET_IN;
            }
            return;
        }
    }

    // ---------- 1: turn the turret (PD on tx) ----------

    private void aim(double dt) {
        if (!tagVisible) {               // no tag: stop and wait. (You could add a "search" sweep here.)
            turret.setPower(0);
            lastAimError = 0;
            return;
        }
        // tx is + when the tag is to the RIGHT. TUNE: if the turret turns AWAY from the tag, flip the sign here.
        double error = aimErrorDeg;
        double derivative = dt > 0 ? (error - lastAimError) / dt : 0;
        lastAimError = error;

        double power = Math.abs(error) < AIM_TOLERANCE_DEG ? 0
                : Range.clip(aimKP * error + aimKD * derivative, -AIM_MAX_POWER, AIM_MAX_POWER);

        // Soft limits so the wires don't wrap: don't keep turning past the limit.
        double turretDeg = turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
        if ((turretDeg > TURRET_LIMIT_DEG && power > 0) || (turretDeg < -TURRET_LIMIT_DEG && power < 0)) power = 0;
        turret.setPower(power);
    }

    // ---------- 3: hood + flywheel ----------

    private void runShot(double dt) {
        if (manualMode) {
            targetHoodDeg = manualHoodDeg;
            targetRpm = flywheelOn ? manualRpm : 0;
        } else if (distanceIn > 0) {
            double[] shot = lookUp(distanceIn);    // uses the last distance seen, even if the tag blinks out
            targetHoodDeg = shot[0];
            targetRpm = flywheelOn ? shot[1] : IDLE_RPM;
        } else {
            targetRpm = flywheelOn ? IDLE_RPM : 0;  // haven't seen the tag yet
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
        flyIntegral = Range.clip(flyIntegral + error * dt, -2000, 2000);   // clip so it can't "wind up"
        double derivative = dt > 0 ? (error - lastFlyError) / dt : 0;
        lastFlyError = error;

        double power = flyKF * targetRpm + flyKP * error + flyKI * flyIntegral + flyKD * derivative;
        flywheel.setPower(Range.clip(power, 0, 1));   // never run it backwards to slow down
    }

    /** Find {hood deg, RPM} for a distance by blending the two nearest rows of SHOT_TABLE. */
    private static double[] lookUp(double distance) {
        double[][] t = SHOT_TABLE;
        if (distance <= t[0][0]) return new double[]{t[0][1], t[0][2]};
        for (int i = 1; i < t.length; i++) {
            if (distance <= t[i][0]) {
                double f = (distance - t[i - 1][0]) / (t[i][0] - t[i - 1][0]);   // 0 = previous row, 1 = this row
                return new double[]{t[i - 1][1] + f * (t[i][1] - t[i - 1][1]),
                                    t[i - 1][2] + f * (t[i][2] - t[i - 1][2])};
            }
        }
        double[] last = t[t.length - 1];
        return new double[]{last[1], last[2]};                                 // farther than the table: use the last row
    }

    private void setHoodAngle(double deg) {
        deg = Range.clip(deg, HOOD_MIN_DEG, HOOD_MAX_DEG);
        double f = (deg - HOOD_MIN_DEG) / (HOOD_MAX_DEG - HOOD_MIN_DEG);
        hood.setPosition(HOOD_SERVO_AT_MIN + f * (HOOD_SERVO_AT_MAX - HOOD_SERVO_AT_MIN));
    }

    // ---------- for the OpMode ----------

    /** True when it's safe to feed a ball: tag seen, aimed, flywheel at speed. */
    public boolean readyToShoot() {
        return flywheelOn && (manualMode || (tagVisible && Math.abs(aimErrorDeg) < AIM_TOLERANCE_DEG))
                && targetRpm > 0 && Math.abs(targetRpm - currentRpm) < RPM_TOLERANCE;
    }

    public void setFlywheelOn(boolean on) { flywheelOn = on; }
    public boolean isFlywheelOn() { return flywheelOn; }
    public void setManualMode(boolean on) { manualMode = on; }
    public boolean isManualMode() { return manualMode; }
    public void setManualShot(double rpm, double hoodDeg) {
        manualRpm = Range.clip(rpm, 0, MOTOR_FREE_RPM);
        manualHoodDeg = Range.clip(hoodDeg, HOOD_MIN_DEG, HOOD_MAX_DEG);
    }
    public double getManualRpm() { return manualRpm; }
    public double getManualHoodDeg() { return manualHoodDeg; }

    public boolean isTagVisible() { return tagVisible; }
    public double getAimErrorDeg() { return aimErrorDeg; }
    public double getDistanceIn() { return distanceIn; }
    public double getTargetRpm() { return targetRpm; }
    public double getCurrentRpm() { return currentRpm; }
    public double getTargetHoodDeg() { return targetHoodDeg; }
    public double getTurretDeg() { return turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE; }

    public void stop() {
        flywheelOn = false;
        turret.setPower(0);
        flywheel.setPower(0);
    }
}
