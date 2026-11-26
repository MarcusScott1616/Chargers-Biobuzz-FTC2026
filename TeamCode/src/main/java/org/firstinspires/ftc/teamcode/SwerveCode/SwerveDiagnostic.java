
package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Swerve Diagnostic - test each drive motor, continuous steering servo,
 * REV Through-Bore encoders (via OctoQuad or motor ports), and Pinpoint IMU.
 *
 * GAMEPAD 1
 *   D-pad up      run RED drive motor
 *   D-pad right   run BLUE drive motor
 *   D-pad left    run GREEN drive motor
 *   D-pad down    run YELLOW drive motor
 *
 *   Left bumper   PID steer all pods to 0 deg (straight ahead)
 *   Right bumper  PID steer all pods to +90 deg (facing right)
 *   X button      PID steer all pods to -90 deg (facing left)
 *   Y button      PID steer all pods to 180 deg (facing back)
 *   A button      Re-zero all pod encoders (set current position as 0 deg)
 *   B button      Toggle SERVO_REVERSED live (test both pod gear directions!)
 *
 *   Right Trigger Manual jog all steering servos CW / right (bypasses PID)
 *   Left Trigger  Manual jog all steering servos CCW / left (bypasses PID)
 *
 * Telemetry displays raw encoder ticks, angles, servo powers, and heading.
 */
@TeleOp(name = "Swerve Diagnostic")
public class SwerveDiagnostic extends OpMode {

    private static final double TEST_POWER = 0.4; // test power for drive motors

    private SwerveDriveV2 swerve;
    private SwerveDriveV2.Module[] modules;
    private boolean lastB = false;
    private boolean lastLeftStickBtn = false;
    private boolean lastRSUp = false, lastRSDown = false, lastRSRight = false, lastRSLeft = false;

    @Override
    public void init() {
        swerve = new SwerveDriveV2();
        swerve.init(hardwareMap);
        modules = swerve.getModules();

        telemetry.addLine("Swerve Diagnostic ready.");
        telemetry.addLine("D-pad: test drive motors (up=RED, right=BLUE, left=GREEN, down=YLW)");
        telemetry.addLine("Bumpers/Buttons: test steering PID (LB=0°, RB=90°, X=-90°, Y=180°)");
        telemetry.addLine("B button: toggle SERVO_REVERSED | L3: toggle ENCODER_REVERSED");
        telemetry.addLine("Triggers: manual servo jog (RT=CW/+, LT=CCW/-)");
        telemetry.addLine("A button: re-zero encoders (works in INIT and PLAY)");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.a) {
            swerve.resetEncoders();
        }
        telemetry.addLine("=== INIT (Servos Limp - Align Pods Forward & Press A) ===");
        telemetry.addData("Encoder Source", swerve.isUsingOctoQuad() ? "OctoQuad (I2C)" : "Motor Ports (Fallback)");
        if (modules != null) {
            for (SwerveDriveV2.Module m : modules) {
                telemetry.addData(m.name.toUpperCase(),
                        String.format("Ticks: %6d | Angle: %+6.1f°",
                                m.getTicks(),
                                m.getCurrentAngleDeg()));
            }
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        // Toggle SERVO_REVERSED on B button press (edge-triggered)
        boolean bPressed = gamepad1.b && !lastB;
        lastB = gamepad1.b;
        if (bPressed) {
            SwerveDriveV2.SERVO_REVERSED = !SwerveDriveV2.SERVO_REVERSED;
        }

        // Toggle ENCODER_REVERSED on Left Stick click (L3, edge-triggered)
        boolean l3Pressed = gamepad1.left_stick_button && !lastLeftStickBtn;
        lastLeftStickBtn = gamepad1.left_stick_button;
        if (l3Pressed) {
            SwerveDriveV2.ENCODER_REVERSED = !SwerveDriveV2.ENCODER_REVERSED;
            if (modules != null) {
                for (SwerveDriveV2.Module m : modules) {
                    m.encoderReversed = SwerveDriveV2.ENCODER_REVERSED;
                }
            }
        }

        // Zero all encoders if A button is pressed
        if (gamepad1.a) {
            swerve.resetEncoders();
        }

        // Live PID tuning with Right Stick flicks (>80% threshold prevents accidental bumps)
        boolean rsUp    = gamepad1.right_stick_y < -0.8;
        boolean rsDown  = gamepad1.right_stick_y >  0.8;
        boolean rsRight = gamepad1.right_stick_x >  0.8;
        boolean rsLeft  = gamepad1.right_stick_x < -0.8;

        if (rsUp && !lastRSUp)       SwerveDriveV2.kP = Math.min(1.0,  Math.round((SwerveDriveV2.kP + 0.02) * 100.0) / 100.0);
        if (rsDown && !lastRSDown)   SwerveDriveV2.kP = Math.max(0.02, Math.round((SwerveDriveV2.kP - 0.02) * 100.0) / 100.0);
        if (rsRight && !lastRSRight) SwerveDriveV2.kD = Math.min(0.10, Math.round((SwerveDriveV2.kD + 0.002) * 1000.0) / 1000.0);
        if (rsLeft && !lastRSLeft)   SwerveDriveV2.kD = Math.max(0.0,  Math.round((SwerveDriveV2.kD - 0.002) * 1000.0) / 1000.0);

        lastRSUp = rsUp;
        lastRSDown = rsDown;
        lastRSRight = rsRight;
        lastRSLeft = rsLeft;

        // ==================== 1. DRIVE MOTORS MANUAL TEST ====================
        // Control each module's drive motor directly without interference
        if (modules != null && modules.length >= 4) {
            modules[0].motor.setPower(gamepad1.dpad_up ? TEST_POWER : 0);
            modules[1].motor.setPower(gamepad1.dpad_right ? TEST_POWER : 0);
            modules[2].motor.setPower(gamepad1.dpad_left ? TEST_POWER : 0);
            modules[3].motor.setPower(gamepad1.dpad_down ? TEST_POWER : 0);
        }

        // ==================== 2. STEERING SERVOS TEST ====================
        // Check for manual open-loop trigger jog first
        double manualJog = gamepad1.right_trigger - gamepad1.left_trigger;

        if (Math.abs(manualJog) > 0.05) {
            // Direct open-loop test: drives the CR servos directly with trigger input
            double jogPower = manualJog * 0.6; // scale to 60% max for safety
            if (modules != null) {
                for (SwerveDriveV2.Module m : modules) {
                    m.setDirectServoPower(jogPower);
                }
            }
        } else {
            // Steering PID test targets
            Double targetRad = null;
            if (gamepad1.left_bumper) {
                targetRad = Math.toRadians(0);
            } else if (gamepad1.right_bumper) {
                targetRad = Math.toRadians(90);
            } else if (gamepad1.x) {
                targetRad = Math.toRadians(-90);
            } else if (gamepad1.y) {
                targetRad = Math.toRadians(180);
            }

            if (targetRad != null) {
                if (modules != null) {
                    for (SwerveDriveV2.Module m : modules) {
                        m.updateSteering(targetRad);
                    }
                }
            } else {
                // Disable PWM so steering servos are completely limp and can be turned freely by hand
                if (modules != null) {
                    for (SwerveDriveV2.Module m : modules) {
                        m.disablePwm();
                    }
                }
            }
        }

        // ==================== 3. TELEMETRY ====================
        telemetry.addLine("=== SYSTEM STATUS ===");
        telemetry.addData("SERVO_REVERSED (B)", "%b | ENCODER_REVERSED (L3): %b",
                SwerveDriveV2.SERVO_REVERSED, SwerveDriveV2.ENCODER_REVERSED);
        telemetry.addData("PID (R-Stick ↑↓ kP, ←→ kD)", "kP:%.2f kD:%.3f Tol:%.1f°",
                SwerveDriveV2.kP, SwerveDriveV2.kD, Math.toDegrees(SwerveDriveV2.STEER_TOLERANCE));
        telemetry.addData("Encoder Source", swerve.isUsingOctoQuad() ? "OctoQuad (I2C)" : "Motor Ports (Fallback)");
        telemetry.addData("Heading", "%.1f° (Source: %s)",
                Math.toDegrees(swerve.getHeadingRad()),
                swerve.isUsingPinpoint() ? "Pinpoint IMU" : "REV Hub IMU");
        telemetry.addLine("Check: Hold RT -> Angle must INCREASE (+). Hold RB -> pods lock cleanly at +90°");

        telemetry.addLine("=== POD ENCODERS & SERVOS ===");
        if (modules != null) {
            for (SwerveDriveV2.Module m : modules) {
                telemetry.addData(m.name.toUpperCase(),
                        String.format("Ticks: %6d | Angle: %+6.1f° | Err: %+5.1f° | Srv: %+.2f",
                                m.getTicks(),
                                m.getCurrentAngleDeg(),
                                m.getErrorDeg(),
                                m.getServoPower()));
            }
        }

        telemetry.addLine("=== DRIVE MOTORS ===");
        telemetry.addData("Red", gamepad1.dpad_up ? "RUNNING" : "off");
        telemetry.addData("Blue", gamepad1.dpad_right ? "RUNNING" : "off");
        telemetry.addData("Green", gamepad1.dpad_left ? "RUNNING" : "off");
        telemetry.addData("Yellow", gamepad1.dpad_down ? "RUNNING" : "off");
        telemetry.update();
    }

    @Override
    public void stop() {
        swerve.stop();
    }
}

