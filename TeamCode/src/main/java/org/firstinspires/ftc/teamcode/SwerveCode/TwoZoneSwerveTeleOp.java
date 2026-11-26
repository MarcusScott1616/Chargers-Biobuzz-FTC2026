package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Two-Zone Swerve TeleOp - clean field-centric swerve TeleOp using TwoZoneSwerveDrive
 * (overshoot-proof Two-Zone Approach controller without PID/kD).
 *
 * GAMEPAD 1:
 *   Left Stick     Field-centric strafe (X/Y)
 *   Right Stick X  Rotate robot
 *   Back / Guide   Reset field-centric yaw
 *   Start          Re-zero pod steering encoders
 */
@TeleOp(name = "Two-Zone Swerve TeleOp", group = "Two-Zone")
public class TwoZoneSwerveTeleOp extends OpMode {

    private final TwoZoneSwerveDrive swerveDrive = new TwoZoneSwerveDrive();

    @Override
    public void init() {
        swerveDrive.init(hardwareMap);
        telemetry.addLine("Two-Zone Swerve TeleOp Ready");
        telemetry.addLine("Back/Guide: Reset Field-Centric Yaw | Start: Re-zero Encoders");
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
        if (gamepad1.back || gamepad1.guide) {
            swerveDrive.resetYaw();
        }
        if (gamepad1.start) {
            swerveDrive.resetEncoders();
        }

        float strafeX = gamepad1.left_stick_x;
        float strafeY = -gamepad1.left_stick_y; // FTC forward stick is negative
        float rotate  = gamepad1.right_stick_x;

        swerveDrive.move(strafeX, strafeY, rotate);

        telemetry.addData("Heading", "%.1f° (IMU: %s)",
                Math.toDegrees(swerveDrive.getHeadingRad()),
                swerveDrive.isUsingPinpoint() ? "Pinpoint" : "REV Hub");
        telemetry.addData("Encoder Src", swerveDrive.isUsingOctoQuad() ? "OctoQuad (I2C)" : "Motor Ports");
        telemetry.addData("Zone Config", "SlowZone:%.0f° Creep:%.2f Approach:%.2f Fast:%.2f",
                TwoZoneSwerveDrive.SLOW_ZONE_DEG, TwoZoneSwerveDrive.CREEP_POWER,
                TwoZoneSwerveDrive.APPROACH_POWER, TwoZoneSwerveDrive.FAST_TURN_POWER);
        telemetry.addData("DriveCmd", "X:%+.2f Y:%+.2f Rot:%+.2f", strafeX, strafeY, rotate);

        if (swerveDrive.getModules() != null) {
            for (TwoZoneSwerveDrive.Module m : swerveDrive.getModules()) {
                telemetry.addData(m.name.toUpperCase(),
                        "[%-8s] Cur:%+5.1f° Tgt:%+5.1f° Err:%+5.1f° | Srv:%+.2f Mtr:%+.2f",
                        m.getActiveZone(), m.getCurrentAngleDeg(), m.getTargetAngleDeg(),
                        m.getErrorDeg(), m.getServoPower(), m.getMotorPower());
            }
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        swerveDrive.stop();
    }
}
