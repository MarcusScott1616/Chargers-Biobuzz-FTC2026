package org.firstinspires.ftc.teamcode.TurretCode;

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
 * CAMERAS (HuskyLens, names must match the robot configuration):
 *   "stationaryHusky" - fixed camera. Gives distance to the HIVE for BOTH turrets.
 *   "turretHusky"     - camera on the NECTAR turret. Gives aim for NECTAR only.
 *   POLLEN has no aim camera, so it doesn't auto-aim. Its readiness only checks the flywheel speed.
 */
@TeleOp(name = "Turret Test", group = "Turret")
public class TurretTestTeleOp extends OpMode {

    private final NectarTurret nectar = new NectarTurret();
    private final PollenTurret pollen = new PollenTurret();

    private boolean tuningNectar = true;     // which turret the buttons change
    private final double[] stepSizes = {0.01, 0.001, 0.0001, 0.00001};
    private int stepIndex = 1;

    private TagCamera stationary, turretCam;   // the two HuskyLenses (see the notes above)

    @Override
    public void init() {
        stationary = TagCamera.init(hardwareMap, "stationaryHusky");
        turretCam = TagCamera.init(hardwareMap, "turretHusky");

        nectar.init(hardwareMap, stationary, turretCam);
        pollen.init(hardwareMap, stationary, null);   // no aim camera for Pollen

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

        // ----- read the cameras ONCE, then run both turrets -----
        stationary.update();
        turretCam.update();
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
