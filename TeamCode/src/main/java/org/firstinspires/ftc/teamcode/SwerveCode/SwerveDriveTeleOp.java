package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Swerve Drive TeleOp")
public class SwerveDriveTeleOp extends OpMode {

    SwerveDriveV2 swerveDrive = new SwerveDriveV2();

    /** Stick inputs within this deadzone are treated as exactly zero. */
    private static final float STICK_DEADZONE = 0.05f;

    private float deadzone(float val) {
        return Math.abs(val) > STICK_DEADZONE ? val : 0f;
    }

    @Override
    public void init() {
        swerveDrive.init(hardwareMap);
        telemetry.addLine("Swerve Drive TeleOp Ready");
        telemetry.addLine("Gamepad 1 Back/Guide: Reset Field-Centric Yaw");
        telemetry.addLine("Gamepad 1 Start: Re-zero Pod Steering Encoders");
        telemetry.addData("Encoder Source", swerveDrive.isUsingOctoQuad() ? "OctoQuad (I2C)" : "Motor Ports");
        telemetry.addData("IMU Source", swerveDrive.isUsingPinpoint() ? "Pinpoint IMU" : "REV Hub IMU");
        telemetry.update();
    }

    @Override
    public void start() {
        swerveDrive.enablePwm();
        swerveDrive.resetEncoders();
    }

    @Override
    public void loop() {
        // Reset field-centric forward direction
        if (gamepad1.back || gamepad1.guide) {
            swerveDrive.resetYaw();
        }

        // Re-zero pod steering encoders if manually re-aligned
        if (gamepad1.start) {
            swerveDrive.resetEncoders();
        }

        // Apply deadzones to eliminate stick drift turning the swerve pods
        float strafeX = deadzone(gamepad1.left_stick_x);
        float strafeY = deadzone(-gamepad1.left_stick_y); // In FTC, forward stick is negative
        float rotate  = deadzone(gamepad1.right_stick_x);

        swerveDrive.move(strafeX, strafeY, rotate);

        telemetry.addData("Heading", "%.1f° (IMU: %s)",
                Math.toDegrees(swerveDrive.getHeadingRad()),
                swerveDrive.isUsingPinpoint() ? "Pinpoint" : "REV Hub");
        telemetry.addData("Encoder Src", swerveDrive.isUsingOctoQuad() ? "OctoQuad (I2C)" : "Motor Ports");
        telemetry.addData("Strafe X", "%.2f", strafeX);
        telemetry.addData("Strafe Y", "%.2f", strafeY);
        telemetry.addData("Rotate", "%.2f", rotate);

        if (swerveDrive.getModules() != null) {
            for (SwerveDriveV2.Module m : swerveDrive.getModules()) {
                telemetry.addData(m.name.toUpperCase(), "%.1f°", m.getCurrentAngleDeg());
            }
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        swerveDrive.stop();
    }
}
