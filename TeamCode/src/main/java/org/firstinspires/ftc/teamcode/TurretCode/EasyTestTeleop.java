package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Easy Test Teleop", group = "Turret")
public class EasyTestTeleop extends OpMode {

    public int flywheelRPM = 3500;
    public double targetPower = 0.8;

    // Encoder ticks per revolution (goBILDA 5203 1:1 is 28 ticks per rev)
    public static final double TICKS_PER_REV = 28.0;

    // Hardware configuration name for the flywheel motor
    public String motorName = "pollenFlywheel";

    private DcMotorEx flywheelMotor;
    private String activeMotorName = "NONE";
    private boolean isReversed = true;
    private boolean lastX = false;

    @Override
    public void init() {
        // Search for the motor by specified name, with fallback to other common names
        String[] candidateNames = {motorName, "pollenFlywheel", "nectarFlywheel", "flywheelMotor", "flywheel"};
        for (String name : candidateNames) {
            try {
                flywheelMotor = hardwareMap.get(DcMotorEx.class, name);
                activeMotorName = name;
                break;
            } catch (Exception ignored) {
            }
        }

        if (flywheelMotor != null) {
            flywheelMotor.setDirection(isReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addData("Motor Connected", activeMotorName);
        } else {
            telemetry.addLine("ERROR: No flywheel motor found! Check robot configuration.");
        }

        telemetry.addLine("Ready. Press Start + A on controller 1 if not connected.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (flywheelMotor == null) {
            telemetry.addLine("ERROR: Motor not connected! Check hardware configuration.");
            telemetry.update();
            return;
        }

        // Press X to toggle direction live (FORWARD <-> REVERSE)
        if (gamepad1.x && !lastX) {
            isReversed = !isReversed;
            flywheelMotor.setDirection(isReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        }
        lastX = gamepad1.x;

        // D-Pad Up / Down adjusts power by +/- 0.05
        if (gamepad1.dpad_up) {
            targetPower = Math.min(1.0, targetPower + 0.005);
        } else if (gamepad1.dpad_down) {
            targetPower = Math.max(0.1, targetPower - 0.005);
        }

        // Run flywheel with Right Trigger OR Right Bumper OR A button
        boolean runMotor = (gamepad1.right_trigger > 0.1) || gamepad1.right_bumper || gamepad1.a;

        if (runMotor) {
            flywheelMotor.setPower(targetPower);
        } else {
            flywheelMotor.setPower(0.0);
        }

        // Calculate current RPM from encoder velocity (ticks/sec -> RPM)
        double rawVelocity = flywheelMotor.getVelocity();
        double currentRPM = Math.abs(rawVelocity / TICKS_PER_REV * 60.0);

        // Telemetry diagnostics on Driver Hub
        telemetry.addData("Motor Bound To", activeMotorName);
        telemetry.addData("Direction", isReversed ? "REVERSE" : "FORWARD (press X to toggle)");
        telemetry.addData("Target Power", "%.2f (D-pad Up/Down)", targetPower);
        telemetry.addData("Trigger / Button Active", runMotor ? "YES" : "NO (hold RT, RB, or A)");
        telemetry.addData("Raw Ticks/sec", "%.1f", rawVelocity);
        telemetry.addData("Current RPM", "%.0f / %d", currentRPM, flywheelRPM);

        if (currentRPM >= flywheelRPM) {
            telemetry.addLine("target RPM reached!");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (flywheelMotor != null) {
            flywheelMotor.setPower(0);
        }
    }
}
