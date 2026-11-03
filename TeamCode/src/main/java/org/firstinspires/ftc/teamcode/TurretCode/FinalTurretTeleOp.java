package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Final Turret TeleOp - ONE button shoots.
 *
 * GAMEPAD 1
 *   A (press once)   VOLLEY: spin up both turrets, aim, and feed balls.
 *                    Nectar and Pollen are checked together, so the feeder only
 *                    pushes a ball when BOTH turrets are at speed and aimed.
 *                    Press A again during a volley and it is ignored (it finishes first).
 *
 * Before PLAY: press X for BLUE or A for RED. The alliance is only shown on telemetry for now.
 *   Aiming still uses NectarTurret.TARGET_TAG_ID until the cluster is wired into aiming.
 *
 * This file only handles shooting. Add your drive code to this OpMode.
 *
 * CAMERAS (HuskyLens, names must match the robot configuration):
 *   "stationaryHusky" - fixed camera: distance to the HIVE for both turrets.
 *   "turretHusky"     - camera on the Nectar turret: aims Nectar.
 *   Pollen has no aim camera, so it only checks its flywheel speed.
 *
 * HARDWARE: "indexMotor" (feeder, via TurretAutoLogic), plus the turret and flywheel
 * names listed in NectarTurret and PollenTurret.
 */
@TeleOp(name = "Final Turret TeleOp", group = "Turret")
public class FinalTurretTeleOp extends OpMode {

    // TUNE: how many balls one press of A feeds. Set this to the number of balls you have loaded.
    private static final int BALLS_PER_VOLLEY = 3;

    private final NectarTurret nectar = new NectarTurret();
    private final PollenTurret pollen = new PollenTurret();
    private final TurretAutoLogic feeder = new TurretAutoLogic();

    private TagCamera stationary, turretCam;

    private String alliance = null;   // "blue" or "red", chosen in init_loop()

    @Override
    public void init() {
        stationary = TagCamera.init(hardwareMap, "stationaryHusky");
        turretCam = TagCamera.init(hardwareMap, "turretHusky");

        nectar.init(hardwareMap, stationary, turretCam);
        pollen.init(hardwareMap, stationary, null);   // Pollen has no aim camera
        feeder.init(hardwareMap);

        telemetry.addLine("Ready. Choose alliance: X = BLUE, A = RED");
    }

    /** Runs repeatedly before PLAY. Pick the alliance here. */
    @Override
    public void init_loop() {
        if (gamepad1.x) alliance = "blue";
        if (gamepad1.a) alliance = "red";

        telemetry.addData("Alliance", alliance == null ? "NOT SET" : alliance.toUpperCase());
        telemetry.addLine("Press PLAY when ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ----- the one button -----
        // Start a volley: turn on both flywheels and tell the feeder how many balls to push.
        // fireShots() does nothing while a volley is already running, so pressing again is safe.
        if (gamepad1.aWasPressed() && !feeder.isBusy()) {
            nectar.setFlywheelOn(true);
            pollen.setFlywheelOn(true);
            feeder.fireShots(BALLS_PER_VOLLEY);
        }

        // ----- every loop, in this order: cameras, turrets, feeder -----
        stationary.update();
        turretCam.update();
        nectar.update();
        pollen.update();

        // The feeder pushes a ball only when BOTH turrets are ready. This makes the two shoot together.
        boolean bothReady = nectar.readyToShoot() && pollen.readyToShoot();
        feeder.update(bothReady);

        // When the volley is over, turn the flywheels off to save battery and heat.
        if (!feeder.isBusy()) {
            nectar.setFlywheelOn(false);
            pollen.setFlywheelOn(false);
        }

        // ----- screen -----
        telemetry.addData("Alliance", alliance == null ? "NOT SET" : alliance.toUpperCase());
        telemetry.addData("Status", feeder.isBusy() ? "SHOOTING" : (bothReady ? "READY - press A" : "idle"));
        telemetry.addData("Nectar ready / aim error", "%s / %.1f deg", nectar.readyToShoot(), nectar.getAimErrorDeg());
        telemetry.addData("Nectar RPM", "%.0f / %.0f", nectar.getCurrentRpm(), nectar.getTargetRpm());
        telemetry.addData("Pollen ready", pollen.readyToShoot());
        telemetry.addData("Pollen RPM", "%.0f / %.0f", pollen.getCurrentRpm(), pollen.getTargetRpm());
        telemetry.update();
    }

    @Override
    public void stop() {
        feeder.stop();
        nectar.stop();
        pollen.stop();
    }
}
