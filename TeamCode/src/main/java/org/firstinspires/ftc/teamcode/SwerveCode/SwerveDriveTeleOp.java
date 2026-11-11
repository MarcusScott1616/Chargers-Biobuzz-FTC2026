package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Swerve Drive TeleOp")
public class SwerveDriveTeleOp extends OpMode {

    SwerveDriveV2 swerveDrive = new SwerveDriveV2();

    @Override
    public void init() {
        swerveDrive.init(hardwareMap);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        float strafeX = gamepad1.left_stick_x;
        float strafeY = -gamepad1.left_stick_y; // In FTC, forward stick is negative
        float rotate = gamepad1.right_stick_x;

        swerveDrive.move(strafeX, strafeY, rotate);
    }

    @Override
    public void stop() {
        swerveDrive.stop();
    }
}
