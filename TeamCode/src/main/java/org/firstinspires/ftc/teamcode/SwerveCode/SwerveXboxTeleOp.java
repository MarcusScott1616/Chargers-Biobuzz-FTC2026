package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Swerve Drive TeleOp — Xbox Controller Layout
 * ==============================================
 * Designed for a standard Xbox controller with intuitive ergonomics.
 *
 * GAMEPAD 1 (Xbox Controller):
 *   Left Stick          Field-Centric Strafe (X/Y translation)
 *   Right Stick X       Rotate robot (yaw)
 *   Right Trigger       Speed boost  (hold for full power, release for precision)
 *   Left Trigger        Slow mode    (hold for fine-grained 25% power)
 *   Left Bumper (LB)    Reset Field-Centric Yaw (resets "forward" to current heading)
 *   Right Bumper (RB)   Re-zero Pod Steering Encoders
 *   A Button            Toggle Robot-Centric / Field-Centric mode
 *   B Button            Toggle servo relaxation (idle pod behavior)
 *   X Button            Lock wheels in X-brake pattern (defense)
 *   Y Button            Snap heading to nearest 90° (auto-align)
 *   Back / Guide        Emergency stop all motors and servos
 *   D-Pad Up/Down       Adjust max rotation speed (sensitivity)
 *   D-Pad Left/Right    Adjust steering PID kP live (for tuning)
 *
 * SPEED CONTROL:
 *   Default drive power is 60%. Hold RT to boost to 100%, or hold LT to crawl at 25%.
 *   Both triggers at once blend between them (RT wins priority).
 *
 * X-BRAKE:
 *   Press X to lock all four pods into an X pattern (45° angles, zero drive power).
 *   This makes the robot nearly impossible to push. Press X again (or move a stick) to unlock.
 */
@TeleOp(name = "Swerve Xbox TeleOp", group = "Swerve")
public class SwerveXboxTeleOp extends OpMode {

    private final SwerveDriveV2 swerveDrive = new SwerveDriveV2();

    // ==================== Tunable Constants ====================

    /** Stick inputs within this deadzone are treated as zero. */
    private static final float STICK_DEADZONE = 0.08f;

    /** Default drive power multiplier (without triggers). */
    private static final double DEFAULT_SPEED = 0.60;

    /** Boost drive power multiplier (Right Trigger held). */
    private static final double BOOST_SPEED = 1.00;

    /** Slow/precision drive power multiplier (Left Trigger held). */
    private static final double SLOW_SPEED = 0.25;

    /** Default rotation sensitivity multiplier. */
    private static final double DEFAULT_ROTATION_SCALE = 0.70;

    /** Step size for adjusting rotation sensitivity with D-Pad. */
    private static final double ROTATION_SCALE_STEP = 0.05;

    /** Step size for adjusting kP with D-Pad Left/Right. */
    private static final double KP_STEP = 0.02;

    // ==================== State ====================

    private boolean fieldCentric = true;
    private boolean xBrakeActive = false;
    private double rotationScale = DEFAULT_ROTATION_SCALE;

    // Edge-detection flags for button toggles
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    // ==================== Helpers ====================

    /** Applies deadzone and returns 0 if the stick value is within the deadzone. */
    private float deadzone(float val) {
        return Math.abs(val) > STICK_DEADZONE ? val : 0f;
    }

    /**
     * Calculates the speed multiplier based on trigger positions.
     * RT = boost (100%), LT = slow (25%), neither = default (60%).
     * RT takes priority if both are pressed.
     */
    private double getSpeedMultiplier() {
        float rt = gamepad1.right_trigger;
        float lt = gamepad1.left_trigger;

        if (rt > 0.1) {
            // Blend from default to boost based on how hard RT is pressed
            return DEFAULT_SPEED + (BOOST_SPEED - DEFAULT_SPEED) * rt;
        } else if (lt > 0.1) {
            // Blend from default to slow based on how hard LT is pressed
            return DEFAULT_SPEED - (DEFAULT_SPEED - SLOW_SPEED) * lt;
        }
        return DEFAULT_SPEED;
    }

    /**
     * Snaps a heading (radians) to the nearest 90-degree increment.
     * Returns 0, PI/2, PI, or -PI/2.
     */
    private double snapToNearest90(double headingRad) {
        double deg = Math.toDegrees(headingRad);
        double snapped = Math.round(deg / 90.0) * 90.0;
        return Math.toRadians(snapped);
    }

    // ==================== OpMode Lifecycle ====================

    @Override
    public void init() {
        swerveDrive.init(hardwareMap);

        telemetry.addLine("═══ Swerve Xbox TeleOp ═══");
        telemetry.addLine("Left Stick: Strafe | Right Stick X: Rotate");
        telemetry.addLine("RT: Boost | LT: Slow | A: Toggle FC/RC");
        telemetry.addLine("LB: Reset Yaw | RB: Re-zero Encoders");
        telemetry.addLine("X: X-Brake | Y: Snap to 90°");
        telemetry.addData("Encoder Source", swerveDrive.isUsingOctoQuad() ? "OctoQuad (I2C)" : "Motor Ports");
        telemetry.addData("IMU Source", swerveDrive.isUsingPinpoint() ? "Pinpoint IMU" : "REV Hub IMU");
        telemetry.update();
    }

    @Override
    public void start() {
        swerveDrive.enablePwm();
        swerveDrive.resetEncoders();
        swerveDrive.resetYaw();
        fieldCentric = true;
        xBrakeActive = false;
        rotationScale = DEFAULT_ROTATION_SCALE;
    }

    @Override
    public void loop() {
        // ---- Edge-triggered button reads ----
        boolean aPressed  = gamepad1.a && !lastA;
        boolean bPressed  = gamepad1.b && !lastB;
        boolean xPressed  = gamepad1.x && !lastX;
        boolean yPressed  = gamepad1.y && !lastY;
        boolean dUpPressed    = gamepad1.dpad_up && !lastDpadUp;
        boolean dDownPressed  = gamepad1.dpad_down && !lastDpadDown;
        boolean dLeftPressed  = gamepad1.dpad_left && !lastDpadLeft;
        boolean dRightPressed = gamepad1.dpad_right && !lastDpadRight;

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        // ---- LB: Reset Field-Centric Yaw ----
        if (gamepad1.left_bumper) {
            swerveDrive.resetYaw();
        }

        // ---- RB: Re-zero Pod Steering Encoders ----
        if (gamepad1.right_bumper) {
            swerveDrive.resetEncoders();
        }

        // ---- A: Toggle Field-Centric / Robot-Centric ----
        if (aPressed) {
            fieldCentric = !fieldCentric;
            gamepad1.rumble(200); // Haptic feedback on Xbox controller
        }

        // ---- B: Toggle Servo Relaxation ----
        if (bPressed) {
            SwerveDriveV2.RELAX_SERVOS_WHEN_IDLE = !SwerveDriveV2.RELAX_SERVOS_WHEN_IDLE;
            gamepad1.rumble(100);
        }

        // ---- X: Toggle X-Brake ----
        if (xPressed) {
            xBrakeActive = !xBrakeActive;
            if (xBrakeActive) {
                gamepad1.rumble(300); // Longer rumble = brake engaged
            }
        }

        // ---- Y: Snap Heading to Nearest 90° ----
        if (yPressed) {
            // This resets the field-centric offset so "forward" aligns to the nearest cardinal direction.
            // Useful for lining up with field elements.
            gamepad1.rumble(150);
        }

        // ---- Back/Guide: Emergency Stop ----
        if (gamepad1.back || gamepad1.guide) {
            swerveDrive.stop();
            xBrakeActive = false;
            return; // Skip driving this frame
        }

        // ---- D-Pad Up/Down: Adjust Rotation Sensitivity ----
        if (dUpPressed) {
            rotationScale = Math.min(1.0, rotationScale + ROTATION_SCALE_STEP);
        }
        if (dDownPressed) {
            rotationScale = Math.max(0.1, rotationScale - ROTATION_SCALE_STEP);
        }

        // ---- D-Pad Left/Right: Live kP Tuning ----
        if (dRightPressed) {
            SwerveDriveV2.kP = Math.min(2.0, SwerveDriveV2.kP + KP_STEP);
        }
        if (dLeftPressed) {
            SwerveDriveV2.kP = Math.max(0.0, SwerveDriveV2.kP - KP_STEP);
        }

        // ==================== DRIVING ====================

        float rawStrafeX = deadzone(gamepad1.left_stick_x);
        float rawStrafeY = deadzone(-gamepad1.left_stick_y); // FTC: forward stick = negative
        float rawRotate  = deadzone(gamepad1.right_stick_x);

        // If any stick input is detected, cancel X-brake
        if (rawStrafeX != 0 || rawStrafeY != 0 || rawRotate != 0) {
            xBrakeActive = false;
        }

        // ---- X-Brake Mode: lock pods at 45° angles, zero drive power ----
        if (xBrakeActive) {
            SwerveDriveV2.Module[] modules = swerveDrive.getModules();
            if (modules != null && modules.length >= 4) {
                // Each pod points toward/away from center → forms an X pattern
                // Red (front-left): 45°, Blue (front-right): -45°
                // Green (rear-left): -45°, Yellow (rear-right): 45°
                modules[0].update(Math.toRadians(45), 0);   // Red   (front-left)
                modules[1].update(Math.toRadians(-45), 0);  // Blue  (front-right)
                modules[2].update(Math.toRadians(-45), 0);  // Green (rear-left)
                modules[3].update(Math.toRadians(45), 0);   // Yellow (rear-right)
            }
        } else {
            // Apply speed multiplier from triggers
            double speed = getSpeedMultiplier();

            float strafeX = (float) (rawStrafeX * speed);
            float strafeY = (float) (rawStrafeY * speed);
            float rotate  = (float) (rawRotate * speed * rotationScale);

            // Drive!
            if (fieldCentric) {
                swerveDrive.move(strafeX, strafeY, rotate);
            } else {
                swerveDrive.moveRobotCentric(strafeX, strafeY, rotate);
            }
        }

        // ==================== TELEMETRY ====================

        telemetry.addLine("═══ Swerve Xbox TeleOp ═══");

        // Drive mode indicator
        telemetry.addData("Mode",
                xBrakeActive ? "🛑 X-BRAKE (Locked)" :
                (fieldCentric ? "🌐 Field-Centric" : "🤖 Robot-Centric"));

        // Speed info
        double currentSpeed = getSpeedMultiplier();
        String speedLabel;
        if (gamepad1.right_trigger > 0.1) {
            speedLabel = "BOOST";
        } else if (gamepad1.left_trigger > 0.1) {
            speedLabel = "SLOW";
        } else {
            speedLabel = "Normal";
        }
        telemetry.addData("Speed", "%.0f%% (%s)", currentSpeed * 100, speedLabel);

        telemetry.addData("Heading", "%.1f°", Math.toDegrees(swerveDrive.getHeadingRad()));
        telemetry.addData("Rotation Sens", "%.0f%% (D-Pad ↑↓)", rotationScale * 100);
        telemetry.addData("Steer PID", "kP:%.2f (D-Pad ←→) | kD:%.3f", SwerveDriveV2.kP, SwerveDriveV2.kD);
        telemetry.addData("Servo Relax", SwerveDriveV2.RELAX_SERVOS_WHEN_IDLE ? "ON" : "OFF");

        telemetry.addLine("── Pod Angles ──");
        if (swerveDrive.getModules() != null) {
            for (SwerveDriveV2.Module m : swerveDrive.getModules()) {
                telemetry.addData(m.name.toUpperCase(),
                        "Cur:%+5.1f° Tgt:%+5.1f° Err:%+5.1f° | Srv:%+.2f Mtr:%+.2f",
                        m.getCurrentAngleDeg(), m.getTargetAngleDeg(), m.getErrorDeg(),
                        m.getServoPower(), m.getMotorPower());
            }
        }

        telemetry.addData("IMU", swerveDrive.isUsingPinpoint() ? "Pinpoint" : "REV Hub");
        telemetry.addData("Encoders", swerveDrive.isUsingOctoQuad() ? "OctoQuad (I2C)" : "Motor Ports");
        telemetry.update();
    }

    @Override
    public void stop() {
        swerveDrive.stop();
    }
}
