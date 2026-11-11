package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Swerve Diagnostic - test each drive motor, continuous steering servo,
 * and REV Through-Bore encoder (analog mode).
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
 *
 * Telemetry displays raw encoder voltages and angles for offset calibration.
 */
@TeleOp(name = "Swerve Diagnostic")
public class SwerveDiagnostic extends OpMode {

    private static final double TEST_POWER = 0.3; // low test power for drive motors

    private SwerveDriveV2 swerve;
    private SwerveDriveV2.Module[] modules;
    private DcMotor redMotor, blueMotor, greenMotor, yellowMotor;

    @Override
    public void init() {
        swerve = new SwerveDriveV2();
        swerve.init(hardwareMap);
        modules = swerve.getModules();

        redMotor = hardwareMap.get(DcMotor.class, "redMotor");
        blueMotor = hardwareMap.get(DcMotor.class, "blueMotor");
        greenMotor = hardwareMap.get(DcMotor.class, "greenMotor");
        yellowMotor = hardwareMap.get(DcMotor.class, "yellowMotor");

        telemetry.addLine("Swerve Diagnostic ready.");
        telemetry.addLine("D-pad: test drive motors (up=RED, right=BLUE, left=GREEN, down=YLW)");
        telemetry.addLine("Bumpers/Buttons: test steering PID (LB=0°, RB=90°, X=-90°, Y=180°)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Drive motors manual test
        redMotor.setPower(gamepad1.dpad_up ? TEST_POWER : 0);
        blueMotor.setPower(gamepad1.dpad_right ? TEST_POWER : 0);
        greenMotor.setPower(gamepad1.dpad_left ? TEST_POWER : 0);
        yellowMotor.setPower(gamepad1.dpad_down ? TEST_POWER : 0);

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
            for (SwerveDriveV2.Module m : modules) {
                // Command angle with zero drive power to test steering PID in place
                m.update(targetRad, 0.001); // tiny non-zero power to keep PID active
                m.motor.setPower(0);
            }
        } else {
            // Relax steering when no steer button is pressed
            for (SwerveDriveV2.Module m : modules) {
                m.stop();
            }
        }

        // Telemetry calibration readouts
        telemetry.addLine("=== POD ENCODERS (FOR CALIBRATION) ===");
        for (SwerveDriveV2.Module m : modules) {
            telemetry.addData(m.name.toUpperCase(),
                    String.format("%.2fV | Angle: %6.1f° (Raw: %6.1f°) | Pwr: %+.2f",
                            m.getVoltage(),
                            m.getCurrentAngleDeg(),
                            Math.toDegrees(m.getRawAngleRad()),
                            m.getServoPower()));
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
