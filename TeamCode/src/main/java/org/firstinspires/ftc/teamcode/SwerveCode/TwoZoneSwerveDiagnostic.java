package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Two-Zone Swerve Diagnostic - dedicated test OpMode for TwoZoneSwerveDrive.
 * Keeps Two-Zone Approach tuning completely separate from PID/SwerveDiagnostic.
 *
 * GAMEPAD 1:
 *   D-pad Up/Right/Left/Down   Test individual drive motors (Red/Blue/Green/Yellow)
 *   Left Bumper (LB)           Steer all pods to   0 deg
 *   Right Bumper (RB)          Steer all pods to +90 deg
 *   X Button                   Steer all pods to -90 deg
 *   Y Button                   Steer all pods to 180 deg
 *   A Button                   Re-zero all pod encoders (works in INIT and PLAY)
 *   B Button                   Toggle SERVO_REVERSED live
 *   Right / Left Trigger       Manual steering jog CW / CCW
 *   Right Stick Up / Down      Adjust CREEP_POWER (±0.01)
 *   Right Stick Left / Right   Adjust SLOW_ZONE_DEG (±2 deg)
 */
@TeleOp(name = "Two-Zone Swerve Diagnostic", group = "Two-Zone")
public class TwoZoneSwerveDiagnostic extends OpMode {

    private static final double TEST_POWER = 0.4;

    private TwoZoneSwerveDrive swerve;
    private TwoZoneSwerveDrive.Module[] modules;
    private boolean lastB = false;
    private boolean lastRSUp = false, lastRSDown = false, lastRSRight = false, lastRSLeft = false;

    @Override
    public void init() {
        swerve = new TwoZoneSwerveDrive();
        swerve.init(hardwareMap);
        modules = swerve.getModules();

        telemetry.addLine("Two-Zone Swerve Diagnostic ready.");
        telemetry.addLine("LB=0° RB=+90° X=-90° Y=180° | A=Zero Encoders (INIT & PLAY)");
        telemetry.addLine("R-Stick ↑↓: CREEP_POWER (±0.01) | R-Stick ←→: SLOW_ZONE_DEG (±2°)");
        telemetry.addLine("D-pad: Drive Motors | Triggers: Manual Servo Jog | B: SERVO_REVERSED");
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
            for (TwoZoneSwerveDrive.Module m : modules) {
                telemetry.addData(m.name.toUpperCase(),
                        String.format("Ticks: %6d | Angle: %+6.1f°", m.getTicks(), m.getCurrentAngleDeg()));
            }
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean bPressed = gamepad1.b && !lastB;
        lastB = gamepad1.b;
        if (bPressed) {
            SwerveDriveV2.SERVO_REVERSED = !SwerveDriveV2.SERVO_REVERSED;
        }

        if (gamepad1.a) {
            swerve.resetEncoders();
        }

        // Live Two-Zone tuning via Right Stick flicks (>80% prevents accidental bumps)
        boolean rsUp    = gamepad1.right_stick_y < -0.8;
        boolean rsDown  = gamepad1.right_stick_y >  0.8;
        boolean rsRight = gamepad1.right_stick_x >  0.8;
        boolean rsLeft  = gamepad1.right_stick_x < -0.8;

        if (rsUp && !lastRSUp) {
            TwoZoneSwerveDrive.CREEP_POWER = Math.min(0.40,
                    Math.round((TwoZoneSwerveDrive.CREEP_POWER + 0.01) * 100.0) / 100.0);
        }
        if (rsDown && !lastRSDown) {
            TwoZoneSwerveDrive.CREEP_POWER = Math.max(0.02,
                    Math.round((TwoZoneSwerveDrive.CREEP_POWER - 0.01) * 100.0) / 100.0);
        }
        if (rsRight && !lastRSRight) {
            TwoZoneSwerveDrive.SLOW_ZONE_DEG = Math.min(60.0, TwoZoneSwerveDrive.SLOW_ZONE_DEG + 2.0);
        }
        if (rsLeft && !lastRSLeft) {
            TwoZoneSwerveDrive.SLOW_ZONE_DEG = Math.max(6.0, TwoZoneSwerveDrive.SLOW_ZONE_DEG - 2.0);
        }

        lastRSUp = rsUp;
        lastRSDown = rsDown;
        lastRSRight = rsRight;
        lastRSLeft = rsLeft;

        // 1. Direct Drive Motor Test
        if (modules != null && modules.length >= 4) {
            modules[0].motor.setPower(gamepad1.dpad_up ? TEST_POWER : 0);
            modules[1].motor.setPower(gamepad1.dpad_right ? TEST_POWER : 0);
            modules[2].motor.setPower(gamepad1.dpad_left ? TEST_POWER : 0);
            modules[3].motor.setPower(gamepad1.dpad_down ? TEST_POWER : 0);
        }

        // 2. Steering Test (Manual Trigger Jog or Two-Zone Target)
        double manualJog = gamepad1.right_trigger - gamepad1.left_trigger;
        if (Math.abs(manualJog) > 0.05) {
            double jogPower = manualJog * 0.6;
            if (modules != null) {
                for (TwoZoneSwerveDrive.Module m : modules) {
                    m.setDirectServoPower(jogPower);
                }
            }
        } else {
            Double targetRad = null;
            if (gamepad1.left_bumper)       targetRad = Math.toRadians(0);
            else if (gamepad1.right_bumper) targetRad = Math.toRadians(90);
            else if (gamepad1.x)            targetRad = Math.toRadians(-90);
            else if (gamepad1.y)            targetRad = Math.toRadians(180);

            if (targetRad != null) {
                if (modules != null) {
                    for (TwoZoneSwerveDrive.Module m : modules) {
                        m.updateSteering(targetRad);
                    }
                }
            } else {
                if (modules != null) {
                    for (TwoZoneSwerveDrive.Module m : modules) {
                        m.disablePwm();
                    }
                }
            }
        }

        // 3. Telemetry
        telemetry.addLine("=== TWO-ZONE DIAGNOSTIC ===");
        telemetry.addData("SERVO_REVERSED (B)", SwerveDriveV2.SERVO_REVERSED);
        telemetry.addData("R-Stick ↑↓ Creep | ←→ Zone", "Creep:%.2f Approach:%.2f Fast:%.2f Zone:%.0f° Tol:%.1f°",
                TwoZoneSwerveDrive.CREEP_POWER, TwoZoneSwerveDrive.APPROACH_POWER,
                TwoZoneSwerveDrive.FAST_TURN_POWER, TwoZoneSwerveDrive.SLOW_ZONE_DEG,
                TwoZoneSwerveDrive.STEER_TOLERANCE_DEG);
        telemetry.addData("Encoder Source", swerve.isUsingOctoQuad() ? "OctoQuad (I2C)" : "Motor Ports (Fallback)");

        telemetry.addLine("=== POD STATUS ===");
        if (modules != null) {
            for (TwoZoneSwerveDrive.Module m : modules) {
                telemetry.addData(m.name.toUpperCase(),
                        String.format("[%-8s] Ticks:%6d | Ang:%+6.1f° | Err:%+5.1f° | Srv:%+.2f",
                                m.getActiveZone(), m.getTicks(),
                                m.getCurrentAngleDeg(), m.getErrorDeg(), m.getServoPower()));
            }
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        swerve.stop();
    }
}
