package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Swerve Drive TeleOp")
public class SwerveDriveTeleOp extends OpMode {

    SwerveDriveV2 swerveDrive = new SwerveDriveV2();

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
        swerveDrive.resetYaw();
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

        // Pass raw stick values directly — SwerveDriveV2.move() applies its own deadzones
        float strafeX = gamepad1.left_stick_x;
        float strafeY = -gamepad1.left_stick_y; // In FTC, forward stick is negative
        float rotate  = gamepad1.right_stick_x;

        swerveDrive.move(strafeX, strafeY, rotate);

        telemetry.addData("Heading", "%.1f° (IMU: %s)",
                Math.toDegrees(swerveDrive.getHeadingRad()),
                swerveDrive.isUsingPinpoint() ? "Pinpoint" : "REV Hub");
        telemetry.addData("Encoder Src", swerveDrive.isUsingOctoQuad() ? "OctoQuad (I2C)" : "Motor Ports");
        telemetry.addData("PID / Rev", "kP:%.2f kD:%.3f Tol:%.1f° | SrvRev:%b",
                SwerveDriveV2.kP, SwerveDriveV2.kD,
                Math.toDegrees(SwerveDriveV2.STEER_TOLERANCE),
                SwerveDriveV2.SERVO_REVERSED);
        telemetry.addData("DriveCmd", "X:%+.2f Y:%+.2f Rot:%+.2f", strafeX, strafeY, rotate);

        if (swerveDrive.getModules() != null) {
            for (SwerveDriveV2.Module m : swerveDrive.getModules()) {
                telemetry.addData(m.name.toUpperCase(),
                        "Cur:%+5.1f° Tgt:%+5.1f° Err:%+5.1f° | Srv:%+.2f Mtr:%+.2f",
                        m.getCurrentAngleDeg(), m.getTargetAngleDeg(), m.getErrorDeg(),
                        m.getServoPower(), m.getMotorPower());
            }
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        swerveDrive.stop();
    }
}
