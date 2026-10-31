package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/*
 * Swerve Diagnostic - test each drive motor and steering servo by hand.
 * It talks to the hardware directly, so the drive math in SwerveDriveV2 is not involved.
 *
 * GAMEPAD 1
 *   D-pad up      run the RED drive motor
 *   D-pad right   run the BLUE drive motor
 *   D-pad left    run the GREEN drive motor
 *   D-pad down    run the YELLOW drive motor
 *   Left bumper   all steering servos to straight ahead (position 0.5)
 *   Right bumper  all steering servos to position 0.0 (far left)
 *
 * Servos hold their last position when no bumper is pressed.
 */
@TeleOp
public class SwerveDiagnostic extends OpMode {

    private static final double TEST_POWER = 0.3; // keep low: the wheels will spin in place

    private ServoImplEx[] servos;
    private DcMotor redMotor, blueMotor, greenMotor, yellowMotor;

    @Override
    public void init() {
        // SwerveDriveV2.init() sets up the IMU, servo ranges, and motor modes the same way the drive does.
        SwerveDriveV2 swerve = new SwerveDriveV2();
        swerve.init(hardwareMap);
        servos = swerve.servoList;

        redMotor = hardwareMap.get(DcMotor.class, "redMotor");
        blueMotor = hardwareMap.get(DcMotor.class, "blueMotor");
        greenMotor = hardwareMap.get(DcMotor.class, "greenMotor");
        yellowMotor = hardwareMap.get(DcMotor.class, "yellowMotor");

        telemetry.addLine("Swerve Diagnostic ready.");
        telemetry.addLine("D-pad: run one drive motor (up=RED, right=BLUE, left=GREEN, down=YELLOW)");
        telemetry.addLine("Left bumper: servos forward (0.5). Right bumper: servos 0.0");
        telemetry.update();
    }

    @Override
    public void loop() {
        redMotor.setPower(gamepad1.dpad_up ? TEST_POWER : 0);
        blueMotor.setPower(gamepad1.dpad_right ? TEST_POWER : 0);
        greenMotor.setPower(gamepad1.dpad_left ? TEST_POWER : 0);
        yellowMotor.setPower(gamepad1.dpad_down ? TEST_POWER : 0);

        if (gamepad1.left_bumper) {
            setServos(0.5);
        } else if (gamepad1.right_bumper) {
            setServos(0.0);
        }

        telemetry.addData("Red motor", gamepad1.dpad_up ? "running" : "off");
        telemetry.addData("Blue motor", gamepad1.dpad_right ? "running" : "off");
        telemetry.addData("Green motor", gamepad1.dpad_left ? "running" : "off");
        telemetry.addData("Yellow motor", gamepad1.dpad_down ? "running" : "off");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Don't leave a wheel spinning after the OpMode ends.
        redMotor.setPower(0);
        blueMotor.setPower(0);
        greenMotor.setPower(0);
        yellowMotor.setPower(0);
    }

    private void setServos(double position) {
        for (ServoImplEx servo : servos) {
            servo.setPosition(position);
        }
    }
}
