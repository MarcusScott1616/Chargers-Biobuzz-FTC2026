package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SwerveCode.SwerveDriveV2;

/*
 * Color Detection Tracking Test - Closed-loop ground tracking with Swerve Drive.
 *
 * HOW IT WORKS:
 *   1. Normal driving: Left stick strafes, right stick turns (Manual TeleOp).
 *   2. Auto-Track & Intake: HOLD Right Bumper (or A button).
 *      - Robot searches for Alliance Nectar first; falls back to Pollen if no nectar is seen.
 *      - Swerve automatically aligns laterally (strafeX) and drives forward (strafeY).
 *      - When close enough, it runs the intake roller to swallow the piece.
 *   3. Safety: RELEASE the button at any time to immediately stop and regain manual control.
 *
 * CONTROLS:
 *   Left stick            Manual strafe (when not auto-tracking)
 *   Right stick           Manual turn (when not auto-tracking)
 *   Right Bumper (hold)   Engage Auto-Track & Intake
 *   A button (hold)       Alternative Auto-Track & Intake
 *   B button              Abort intake sequence / emergency stop intake
 *   Back / Guide          Reset gyro / yaw
 */
@TeleOp(name = "Color Detection Tracking Test", group = "Vision")
public class ColorDetectionTrackingTest extends OpMode {

    // ==================== TUNEABLE SETTINGS ====================

    // Alliance: "red" or "blue" (prioritizes this alliance's nectar, ignores opponent's nectar)
    public static String ALLIANCE = "red";

    // Camera name in Robot Configuration (e.g. "groundHusky" or "stationaryHusky")
    public static String CAMERA_NAME = "groundHusky";

    // Proportional gains for visual servoing
    public static double KP_ALIGN_STRAFE = 0.025; // Lateral strafe correction per degree of tx error
    public static double KP_ALIGN_TURN   = 0.015; // Heading rotation correction per degree of tx error
    public static double APPROACH_SPEED  = 0.35;  // Forward driving speed while chasing an element
    public static double MAX_CORRECTION  = 0.50;  // Maximum lateral / rotational correction power

    // Trigger thresholds for intake deployment
    // tyDeg becomes negative as the piece gets close to the bottom of the camera frame
    public static double INTAKE_TY_THRESHOLD   = -2.0;  // Deploy when ty is at or below this angle (degrees)
    public static int    INTAKE_AREA_THRESHOLD = 3500;  // Or deploy if bounding box area is this large

    // Intake timing & power
    public static double INTAKE_POWER    = 1.0;
    public static double INTAKE_DURATION = 1.2; // Seconds to run intake roller over the piece

    // ==================== HARDWARE & STATE ====================

    private SwerveDriveV2 swerveDrive;
    private HuskyLens husky;
    private DcMotor intakeMotor;

    private enum TrackingState {
        MANUAL,      // User is driving manually
        SEARCHING,   // Auto engaged, looking for nectar/pollen
        CHASING,     // Locked on, driving towards element
        INTAKING     // At the element, running intake sequence
    }

    private TrackingState state = TrackingState.MANUAL;
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private boolean cameraOk = false;

    @Override
    public void init() {
        swerveDrive = new SwerveDriveV2();
        swerveDrive.init(hardwareMap);

        // Try initializing the ground camera
        try {
            husky = hardwareMap.get(HuskyLens.class, CAMERA_NAME);
            if (husky.knock()) {
                husky.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
                cameraOk = true;
            }
        } catch (Exception e) {
            cameraOk = false;
        }

        // Intake hardware
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            intakeMotor.setPower(0);
        } catch (Exception ignored) {}

        telemetry.addLine("Color Detection Tracking Test Ready.");
        telemetry.addData("Camera Status", cameraOk ? "CONNECTED" : "NOT FOUND (check name: " + CAMERA_NAME + ")");
        telemetry.addData("Alliance", ALLIANCE.toUpperCase());
        telemetry.addLine("HOLD Right Bumper or A to Auto-Track & Intake!");
        telemetry.update();
    }

    @Override
    public void start() {
        swerveDrive.enablePwm();
        swerveDrive.resetEncoders();
    }

    @Override
    public void loop() {
        // Reset gyro on back/guide
        if (gamepad1.back || gamepad1.guide) {
            swerveDrive.resetYaw();
        }

        // Emergency abort intake if B is pressed
        if (gamepad1.b) {
            stopIntake();
            state = TrackingState.MANUAL;
        }

        // Check if user is holding the Auto-Track button
        boolean autoTrackRequested = (gamepad1.right_bumper || gamepad1.a);

        // Read HuskyLens blocks
        HuskyLens.Block[] blocks = cameraOk ? husky.blocks() : new HuskyLens.Block[0];

        // Find best target (prioritizes Alliance Nectar > Pollen)
        HuskyLensColorDetection.DetectedElement target =
                HuskyLensColorDetection.findBestTarget(blocks, ALLIANCE);

        // ==================== STATE MACHINE ====================
        if (!autoTrackRequested) {
            // ----- MANUAL MODE -----
            if (state == TrackingState.INTAKING) {
                stopIntake();
            }
            state = TrackingState.MANUAL;

            float strafeX = gamepad1.left_stick_x;
            float strafeY = -gamepad1.left_stick_y; // forward stick is negative in FTC
            float rotate  = gamepad1.right_stick_x;
            swerveDrive.move(strafeX, strafeY, rotate);

        } else {
            // ----- AUTO-TRACKING MODES -----

            if (state == TrackingState.INTAKING) {
                // Currently swallowing the piece
                if (intakeTimer.seconds() < INTAKE_DURATION) {
                    // Drive forward slowly to eat the piece
                    swerveDrive.moveRobotCentric(0, 0.20f, 0);
                    runIntake();
                } else {
                    // Intake complete!
                    stopIntake();
                    state = TrackingState.SEARCHING;
                }

            } else if (target != null) {
                // Target is in view -> CHASE IT!
                state = TrackingState.CHASING;

                // Check if target is close enough to intake
                if (target.tyDeg <= INTAKE_TY_THRESHOLD || target.area >= INTAKE_AREA_THRESHOLD) {
                    // Trigger intake!
                    state = TrackingState.INTAKING;
                    intakeTimer.reset();
                    runIntake();
                    swerveDrive.moveRobotCentric(0, 0.20f, 0);

                } else {
                    // Visual Servoing: align with target and approach
                    double alignStrafe = clamp(target.txDeg * KP_ALIGN_STRAFE, -MAX_CORRECTION, MAX_CORRECTION);
                    double alignTurn   = clamp(target.txDeg * KP_ALIGN_TURN,   -MAX_CORRECTION, MAX_CORRECTION);
                    double forward     = APPROACH_SPEED;

                    // Robot-centric drive directly towards the piece
                    swerveDrive.moveRobotCentric((float) alignStrafe, (float) forward, (float) alignTurn);
                }

            } else {
                // No target in view -> SEARCHING
                state = TrackingState.SEARCHING;
                stopIntake();

                // Stop or gently hold position while searching
                swerveDrive.moveRobotCentric(0, 0, 0);
            }
        }

        // ==================== TELEMETRY ====================
        telemetry.addLine("=== TRACKING STATUS ===");
        telemetry.addData("State", state);
        telemetry.addData("Auto-Track Button", autoTrackRequested ? "HELD (ACTIVE)" : "released (manual)");

        if (target != null) {
            String label = target.isNectar() ? "NECTAR [PRIORITY]" : "POLLEN";
            telemetry.addLine(String.format("Target: %s (ID %d)", label, target.id));
            telemetry.addData("  tx Error (lateral)", String.format("%+.1f°", target.txDeg));
            telemetry.addData("  ty Error (distance)", String.format("%+.1f°", target.tyDeg));
            telemetry.addData("  Area", "%d px", target.area);
        } else {
            telemetry.addLine("Target: None in view");
        }

        telemetry.update();
    }

    private void runIntake() {
        if (intakeMotor != null) intakeMotor.setPower(INTAKE_POWER);
    }

    private void stopIntake() {
        if (intakeMotor != null) intakeMotor.setPower(0);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    @Override
    public void stop() {
        stopIntake();
        swerveDrive.stop();
    }
}
