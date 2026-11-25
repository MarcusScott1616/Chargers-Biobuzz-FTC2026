package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.SwerveCode.SwerveDriveV2;

/*
 * Final TeleOp - Integrated Swerve Drive, Vision Targeting, Pinpoint Odometry Fallback, and Intake.
 *
 * GAMEPAD 1:
 *   Left Stick       Field-Centric Drive (strafe X/Y)
 *   Right Stick      Rotate robot
 *   Back / Guide     Reset Field-Centric Yaw (keeps Pinpoint field pose)
 *   Left Trigger     Intake Full Power (1.0)
 *   Left Bumper      Reverse Intake (-1.0)
 *   Right Bumper     Toggle Shooter (Flywheels) ON / OFF
 *   X Button         Toggle Auto-Shoot ON / OFF (turn off for risky manual shots)
 *   Right Trigger    Manual Fire Trigger (always fires 4-ball volley, even from far away)
 *   D-Pad Up/Down    Target Hive 1 (Up) or Hive 2 (Down) for Odometry Fallback.
 *
 * Before PLAY:
 *   Press X for BLUE alliance or A for RED alliance. (Defaults to BLUE).
 *
 * TARGETING:
 *   - Primary: Uses turret HuskyLens (mounted directly on Nectar turret) for both distance and aim via AprilTags.
 *   - Fallback (Option A): If AprilTags are blocked or out of view, the goBILDA Pinpoint odometry
 *     automatically computes distance and turret aim to the target Hive so shooting never stops!
 */
/*
 * ═══════════════════════════════════════════════════════════════════════════════
 *  ROBOT HARDWARE CONFIGURATION  (Driver Hub → Configure Robot)
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 *  PHYSICAL LAYOUT
 *  ─────────────────────────────────────────────────────────────────────────────
 *  Nectar Turret — FRONT of robot (intake side), flywheel height: 6 in off ground
 *  Pollen Turret — REAR  of robot,                flywheel height: 8 in off ground
 *  HuskyLens     — Mounted on Nectar turret, height: 6.5 in, upward tilt: 30°
 *
 *  CONTROL HUB  (REV Robotics Control Hub)
 *  ─────────────────────────────────────────────────────────────────────────────
 *  Motors (DcMotorEx unless noted):
 *    Port 0 → "yellowMotor"      Swerve pod drive (FORWARD)
 *    Port 1 → "redMotor"         Swerve pod drive (FORWARD)
 *    Port 2 → "blueMotor"        Swerve pod drive (FORWARD)
 *    Port 3 → "greenMotor"       Swerve pod drive (FORWARD)
 *
 *  Servos (CRServo):
 *    Port 0 → "yellowServo"      Yellow pod steering (CRServo, REVERSE)
 *    Port 1 → "redServo"         Red pod steering   (CRServo, REVERSE)
 *    Port 2 → "blueServo"        Blue pod steering  (CRServo, REVERSE)
 *    Port 3 → "greenServo"       Green pod steering (CRServo, REVERSE)
 *
 *  Encoder ports (REV Through-Bore on steering servo shaft):
 *    Encoder 0 → "yellowServoEncoder"
 *    Encoder 1 → "redServoEncoder"
 *    Encoder 2 → "blueServoEncoder"
 *    Encoder 3 → "greenServoEncoder"
 *
 *  I2C:
 *    Bus 0 → "pinpoint"          goBILDA Pinpoint Odometry Computer
 *
 *  EXPANSION HUB  (REV Robotics Expansion Hub)
 *  ─────────────────────────────────────────────────────────────────────────────
 *  Motors (DcMotorEx unless noted):
 *    Port 0 → "nectarFlywheel"   Nectar turret flywheel — 6 in height (goBILDA 5203 6000 RPM)
 *    Port 1 → "pollenFlywheel"   Pollen turret flywheel — 8 in height (goBILDA 5203 6000 RPM)
 *    Port 2 → "intakeMotor"      Intake roller          (DcMotor, BRAKE)
 *    Port 3 → "indexMotor"       Ball feeder / indexer  (DcMotor)
 *
 *  Servos:
 *    Port 1 → "nectarTurret"     Nectar turret pan servo  (front, intake side) Continuous
 *    Port 2 → "nectarHood"       Nectar hood angle servo  (front, intake side) Continuous
 *    Port 3 → "pollenTurret"     Pollen turret pan servo  (rear) Continuous
 *    Port 4 → "pollenHood"       Pollen hood angle servo  (rear) Continuous
 *
 *  I2C:
 *    Bus 0 → "turretHusky"       Turret-mounted HuskyLens on Nectar turret (aim & distance)
 *    (stationaryHusky is unavailable / removed)
 * ═══════════════════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "Final Turret TeleOp", group = "Turret")
public class FinalTurretTeleOp extends OpMode {

    // TUNE: how many balls one press of RT feeds.
    private static final int BALLS_PER_VOLLEY = 4;
    private static final String DEFAULT_ALLIANCE = "blue";
    private static final float STICK_DEADZONE = 0.05f;

    private final SwerveDriveV2 swerveDrive = new SwerveDriveV2();
    private final Turret nectar = Turret.nectar();
    private final Turret pollen = Turret.pollen();
    private final TurretAutoLogic feeder = new TurretAutoLogic();
    private DcMotor intakeMotor;

    private TagCamera turretCam;
    private PinpointOdometry odometry;

    private String alliance = null;   // "blue" or "red", chosen in init_loop()
    private int targetHiveNumber = 1; // 1 or 2
    private boolean lastRightTrigger = false;
    private boolean lastRightBumper = false;
    private boolean lastX = false;
    private boolean shooterOn = false;        // Toggle with Right Bumper
    private boolean autoShootEnabled = true;  // Toggle with X button (turn off for risky manual play)
    private final ElapsedTime autoShootTimer = new ElapsedTime();
    private static final double AUTO_SHOOT_COOLDOWN_SEC = 0.6; // Pause between automatic volleys

    private float deadzone(float val) {
        return Math.abs(val) > STICK_DEADZONE ? val : 0f;
    }

    @Override
    public void init() {
        // Camera (mounted directly on Nectar turret; handles both distance and aiming)
        turretCam = TagCamera.init(hardwareMap, "turretHusky");

        // Pinpoint Odometry fallback
        odometry = PinpointOdometry.init(hardwareMap, "pinpoint");

        // Turrets & Feeder
        nectar.init(hardwareMap, turretCam, turretCam);
        pollen.init(hardwareMap, turretCam, null);   // Pollen shares distance from turretCam
        pollen.setMasterTurret(nectar);
        nectar.setOdometry(odometry);
        pollen.setOdometry(odometry);
        feeder.init(hardwareMap);

        // Swerve Drivetrain
        swerveDrive.init(hardwareMap);
        swerveDrive.setPinpoint(odometry.getDriver());

        // Intake Motor
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready. Choose alliance: X = BLUE, A = RED");
    }

    /** Runs repeatedly before PLAY. Pick the alliance here. */
    @Override
    public void init_loop() {
        if (gamepad1.x) alliance = "blue";
        if (gamepad1.a) alliance = "red";

        telemetry.addData("Alliance", alliance == null ? "NOT SET (press X=blue or A=red)" : alliance.toUpperCase());
        telemetry.addLine("Press PLAY when ready.");
        telemetry.update();
    }

    /** Called once when the driver presses PLAY. */
    @Override
    public void start() {
        if (alliance == null) {
            alliance = DEFAULT_ALLIANCE;
            telemetry.log().add("WARNING: No alliance selected! Defaulting to " + DEFAULT_ALLIANCE.toUpperCase());
        }
        odometry.setTargetByAlliance(alliance, targetHiveNumber);
        swerveDrive.enablePwm();
        swerveDrive.resetEncoders();
        shooterOn = false;
        autoShootEnabled = true;
        autoShootTimer.reset();
    }

    @Override
    public void loop() {
        // ----- Odometry update & Target Hive toggle -----
        odometry.update();
        if (gamepad1.dpad_up) {
            targetHiveNumber = 1;
            odometry.setTargetByAlliance(alliance, 1);
        } else if (gamepad1.dpad_down) {
            targetHiveNumber = 2;
            odometry.setTargetByAlliance(alliance, 2);
        }

        // ----- Swerve Drive (Field-Centric with deadzones) -----
        if (gamepad1.back || gamepad1.guide) {
            swerveDrive.resetYaw();
        }

        float strafeX = deadzone(gamepad1.left_stick_x);
        float strafeY = deadzone(-gamepad1.left_stick_y);
        float rotate  = deadzone(gamepad1.right_stick_x);
        swerveDrive.move(strafeX, strafeY, rotate);

        // ----- Intake controls (Left Trigger: Full Intake, Left Bumper: Reverse) -----
        if (gamepad1.left_trigger > 0.3) {
            intakeMotor.setPower(1.0);
        } else if (gamepad1.left_bumper) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0.0);
        }

        // ----- Shooter Toggle (Right Bumper) & Auto-Shoot Mode Toggle (X Button) -----
        boolean rbPressed = gamepad1.right_bumper && !lastRightBumper;
        lastRightBumper = gamepad1.right_bumper;
        if (rbPressed) {
            shooterOn = !shooterOn;
        }

        boolean xPressed = gamepad1.x && !lastX;
        lastX = gamepad1.x;
        if (xPressed) {
            autoShootEnabled = !autoShootEnabled;
        }

        // Keep flywheels spinning if shooter toggle is ON, or while feeder is busy shooting a volley
        boolean shouldFlywheelsSpin = shooterOn || feeder.isBusy();
        nectar.setFlywheelOn(shouldFlywheelsSpin);
        pollen.setFlywheelOn(shouldFlywheelsSpin);

        // ----- Every loop: camera, turrets, feeder -----
        turretCam.update();
        nectar.update(alliance);
        pollen.update(alliance);
        if (nectar.isTagVisible()) {
            pollen.setTurretAngle(nectar.getTurretDeg());
        }

        boolean bothReady = nectar.readyToShoot() && pollen.readyToShoot();

        // Check if we are in ideal shooting window (in range, correct orientation, locked aim, flywheels at speed)
        boolean inIdealShootingWindow = nectar.inIdealShootingWindow() && pollen.readyToShoot();

        // ----- Auto-shoot: fires automatically when shooter is ON, in range & orientation -----
        if (autoShootEnabled && shooterOn && inIdealShootingWindow && !feeder.isBusy() && autoShootTimer.seconds() >= AUTO_SHOOT_COOLDOWN_SEC) {
            feeder.fireShots(BALLS_PER_VOLLEY);
            autoShootTimer.reset();
        }

        // ----- Manual shot trigger (always fires, even if auto-shoot is off or from risky distances/angles) -----
        boolean triggerPressed = gamepad1.right_trigger > 0.3;
        boolean triggerJustPressed = triggerPressed && !lastRightTrigger;
        lastRightTrigger = triggerPressed;

        if (triggerJustPressed && !feeder.isBusy()) {
            feeder.fireShots(BALLS_PER_VOLLEY);
            autoShootTimer.reset();
        }

        feeder.update(bothReady);

        // ----- Driver Station Telemetry -----
        telemetry.addData("Alliance", alliance.toUpperCase());
        telemetry.addData("Target Hive", "Hive " + targetHiveNumber);
        telemetry.addData("Shooter (RB)", shooterOn ? "ON (Spinning)" : "OFF");
        telemetry.addData("Auto-Shoot (X)", autoShootEnabled ? "ENABLED (Auto in Zone)" : "DISABLED (Risky / Manual Mode)");
        telemetry.addData("Shoot Window", inIdealShootingWindow ? "IDEAL (Aligned & In Range)"
                : (nectar.isTooFar() ? "TOO FAR (>96 in)" : (Math.abs(nectar.getTurretDeg()) > Turret.MAX_AUTO_TURRET_ANGLE_DEG ? "TURRET TOO OBLIQUE" : "ALIGNING / SPINNING UP")));
        telemetry.addData("Tracking Mode", nectar.isUsingOdometryFallback() ? "PINPOINT ODOMETRY (Fallback)" : "VISION (AprilTag)");
        telemetry.addData("Status", feeder.isBusy() ? "SHOOTING VOLLEY"
                : (inIdealShootingWindow && shooterOn ? "AUTO-FIRING" : "IDLE"));

        if (nectar.isTooFar() || pollen.isTooFar()) {
            telemetry.addLine("!! TOO FAR - drive closer !!");
        }

        telemetry.addData("Pose", "X: %.1f in, Y: %.1f in, Heading: %.1f°",
                odometry.getX(), odometry.getY(), odometry.getHeadingDeg());
        telemetry.addData("Nectar dist / aim", "%.1f in / %.1f° (ready: %s)",
                nectar.getDistanceIn(), nectar.getAimErrorDeg(), nectar.readyToShoot());
        telemetry.addData("Turret Angle", "%.1f° (Max Auto: ±%.0f°)", nectar.getTurretDeg(), Turret.MAX_AUTO_TURRET_ANGLE_DEG);
        telemetry.addData("Nectar RPM", "%.0f / %.0f", nectar.getCurrentRpm(), nectar.getTargetRpm());
        telemetry.addData("Pollen RPM", "%.0f / %.0f (ready: %s)", pollen.getCurrentRpm(), pollen.getTargetRpm(), pollen.readyToShoot());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterOn = false;
        if (intakeMotor != null) {
            intakeMotor.setPower(0);
        }
        feeder.stop();
        nectar.stop();
        pollen.stop();
        swerveDrive.stop();
    }
}
