package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Turret Test - try out NectarTurret and PollenTurret, tune them, and build the shot tables.
 *
 * GAMEPAD 1
 *   X              pick which turret the other buttons change (Nectar <-> Pollen)
 *   Left bumper    Nectar flywheel on/off
 *   Right bumper   Pollen flywheel on/off
 *   Y              selected turret: AUTO (shot table) <-> MANUAL (you pick RPM + hood)
 *
 *   In MANUAL mode (use this to fill in SHOT_TABLE):
 *     Dpad up/down      flywheel RPM +/- 50
 *     Dpad left/right   hood angle -/+ 1 degree
 *     -> Park at a distance, change RPM + hood until shots stay in, then copy the
 *        "distance", "hood" and "RPM" from the screen into that turret's SHOT_TABLE.
 *
 *   In AUTO mode (use this to tune the aiming):
 *     B                 change the step size
 *     Dpad left/right   aim kP -/+ step
 *     Dpad up/down      aim kD +/- step
 *     -> Write the good kP/kD numbers into the turret's class so they stick.
 *
 * LIMELIGHT: by default both turrets share ONE Limelight named "limelight" in the robot config.
 * If each turret has its own camera, set TWO_LIMELIGHTS = true and name them "nectarLimelight" and
 * "pollenLimelight". Two Limelights on one robot each need their own address - set a different
 * static IP for each in its web page settings, and check that both show up in the configuration.
 * NOTE: with one shared camera, only a turret the camera is mounted on can aim properly -
 * tx is "how far to turn" only for the turret carrying the camera.
 */
@TeleOp(name = "Turret Test", group = "Turret")
public class TurretTestTeleOp extends OpMode {

    // LIMELIGHT: see the note above.
    private static final boolean TWO_LIMELIGHTS = false;

    private final NectarTurret nectar = new NectarTurret();
    private final PollenTurret pollen = new PollenTurret();

    private boolean tuningNectar = true;     // which turret the buttons change
    private final double[] stepSizes = {0.01, 0.001, 0.0001, 0.00001};
    private int stepIndex = 1;

    @Override
    public void init() {
        if (TWO_LIMELIGHTS) {
            nectar.init(hardwareMap, hardwareMap.get(Limelight3A.class, "nectarLimelight"));
            pollen.init(hardwareMap, hardwareMap.get(Limelight3A.class, "pollenLimelight"));
        } else {
            Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
            nectar.init(hardwareMap, limelight);
            pollen.init(hardwareMap, limelight);
        }
        telemetry.addLine("Turrets ready. Press PLAY.");
    }

    @Override
    public void loop() {
        // ----- buttons -----
        if (gamepad1.xWasPressed()) tuningNectar = !tuningNectar;
        if (gamepad1.leftBumperWasPressed()) nectar.setFlywheelOn(!nectar.isFlywheelOn());
        if (gamepad1.rightBumperWasPressed()) pollen.setFlywheelOn(!pollen.isFlywheelOn());

        if (tuningNectar) handleNectarButtons();
        else handlePollenButtons();

        // ----- run both turrets -----
        nectar.update();
        pollen.update();

        // ----- screen -----
        telemetry.addData("Buttons change", tuningNectar ? "NECTAR (press X for Pollen)" : "POLLEN (press X for Nectar)");
        telemetry.addLine();
        telemetry.addLine("=== NECTAR ===  " + (nectar.isManualMode() ? "MANUAL" : "AUTO") + (nectar.readyToShoot() ? "   READY TO SHOOT" : ""));
        telemetry.addData("Tag seen", nectar.isTagVisible());
        telemetry.addData("Distance (in)", "%.1f", nectar.getDistanceIn());
        telemetry.addData("Aim error (deg)", "%.1f   turret at %.0f deg", nectar.getAimErrorDeg(), nectar.getTurretDeg());
        telemetry.addData("RPM", "%.0f / %.0f target", nectar.getCurrentRpm(), nectar.getTargetRpm());
        telemetry.addData("Hood (deg)", "%.1f", nectar.getTargetHoodDeg());
        telemetry.addLine();
        telemetry.addLine("=== POLLEN ===  " + (pollen.isManualMode() ? "MANUAL" : "AUTO") + (pollen.readyToShoot() ? "   READY TO SHOOT" : ""));
        telemetry.addData("Tag seen", pollen.isTagVisible());
        telemetry.addData("Distance (in)", "%.1f", pollen.getDistanceIn());
        telemetry.addData("Aim error (deg)", "%.1f   turret at %.0f deg", pollen.getAimErrorDeg(), pollen.getTurretDeg());
        telemetry.addData("RPM", "%.0f / %.0f target", pollen.getCurrentRpm(), pollen.getTargetRpm());
        telemetry.addData("Hood (deg)", "%.1f", pollen.getTargetHoodDeg());
        telemetry.addLine();
        telemetry.addData("Step size (B)", stepSizes[stepIndex]);
        telemetry.addData("Nectar aim kP / kD", "%.5f / %.5f", nectar.aimKP, nectar.aimKD);
        telemetry.addData("Pollen aim kP / kD", "%.5f / %.5f", pollen.aimKP, pollen.aimKD);
        telemetry.update();
    }

    // The two turrets are separate classes, so each gets its own button handler.

    private void handleNectarButtons() {
        if (gamepad1.yWasPressed()) nectar.setManualMode(!nectar.isManualMode());
        if (nectar.isManualMode()) {
            double rpm = nectar.getManualRpm(), hood = nectar.getManualHoodDeg();
            if (gamepad1.dpadUpWasPressed()) rpm += 50;
            if (gamepad1.dpadDownWasPressed()) rpm -= 50;
            if (gamepad1.dpadRightWasPressed()) hood += 1;
            if (gamepad1.dpadLeftWasPressed()) hood -= 1;
            nectar.setManualShot(rpm, hood);
        } else {
            if (gamepad1.bWasPressed()) stepIndex = (stepIndex + 1) % stepSizes.length;
            double step = stepSizes[stepIndex];
            if (gamepad1.dpadRightWasPressed()) nectar.aimKP += step;
            if (gamepad1.dpadLeftWasPressed()) nectar.aimKP = Math.max(0, nectar.aimKP - step);
            if (gamepad1.dpadUpWasPressed()) nectar.aimKD += step;
            if (gamepad1.dpadDownWasPressed()) nectar.aimKD = Math.max(0, nectar.aimKD - step);
        }
    }

    private void handlePollenButtons() {
        if (gamepad1.yWasPressed()) pollen.setManualMode(!pollen.isManualMode());
        if (pollen.isManualMode()) {
            double rpm = pollen.getManualRpm(), hood = pollen.getManualHoodDeg();
            if (gamepad1.dpadUpWasPressed()) rpm += 50;
            if (gamepad1.dpadDownWasPressed()) rpm -= 50;
            if (gamepad1.dpadRightWasPressed()) hood += 1;
            if (gamepad1.dpadLeftWasPressed()) hood -= 1;
            pollen.setManualShot(rpm, hood);
        } else {
            if (gamepad1.bWasPressed()) stepIndex = (stepIndex + 1) % stepSizes.length;
            double step = stepSizes[stepIndex];
            if (gamepad1.dpadRightWasPressed()) pollen.aimKP += step;
            if (gamepad1.dpadLeftWasPressed()) pollen.aimKP = Math.max(0, pollen.aimKP - step);
            if (gamepad1.dpadUpWasPressed()) pollen.aimKD += step;
            if (gamepad1.dpadDownWasPressed()) pollen.aimKD = Math.max(0, pollen.aimKD - step);
        }
    }

    @Override
    public void stop() {
        nectar.stop();
        pollen.stop();
    }
}
