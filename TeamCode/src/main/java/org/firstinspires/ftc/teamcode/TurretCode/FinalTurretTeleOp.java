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
 * Before PLAY: press X for BLUE or A for RED. If no alliance is chosen, defaults to BLUE.
 *   The turrets use TagClusters to find the correct cluster for the chosen alliance.
 *
 * This file only handles shooting. Add your drive code to this OpMode.
 *
 * CAMERAS (HuskyLens, names must match the robot configuration):
 *   "stationaryHusky" - fixed camera: distance to the HIVE for both turrets.
 *   "turretHusky"     - camera on the Nectar turret: aims Nectar.
 *   Pollen has no aim camera, so it only checks its flywheel speed.
 *
 * HARDWARE: "indexMotor" (feeder, via TurretAutoLogic), plus the turret and flywheel
 * names listed in Turret.nectar() and Turret.pollen().
 */
@TeleOp(name = "Final Turret TeleOp", group = "Turret")
public class FinalTurretTeleOp extends OpMode {

    // TUNE: how many balls one press of A feeds. Set this to the number of balls you have loaded.
    private static final int BALLS_PER_VOLLEY = 3;
    // If the driver forgets to pick an alliance before PLAY, use this one.
    private static final String DEFAULT_ALLIANCE = "blue";

    private final Turret nectar = Turret.nectar();
    private final Turret pollen = Turret.pollen();
    private final TurretAutoLogic feeder = new TurretAutoLogic();

    private TagCamera stationary, turretCam;

    private String alliance = null;   // "blue" or "red", chosen in init_loop()

    @Override
    public void init() {
        stationary = TagCamera.init(hardwareMap, "stationaryHusky");
        turretCam  = TagCamera.init(hardwareMap, "turretHusky");

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

        telemetry.addData("Alliance", alliance == null ? "NOT SET (press X=blue or A=red)" : alliance.toUpperCase());
        telemetry.addLine("Press PLAY when ready.");
        telemetry.update();
    }

    /** Called once when the driver presses PLAY. */
    @Override
    public void start() {
        if (alliance == null) {
            alliance = DEFAULT_ALLIANCE;
            telemetry.log().add("WARNING: No alliance selected! Defaulting to " + DEFAULT_ALLIANCE.toUpperCase());
        }
    }

    @Override
    public void loop() {
        // ----- the one button -----
        if (gamepad1.aWasPressed() && !feeder.isBusy()) {
            nectar.setFlywheelOn(true);
            pollen.setFlywheelOn(true);
            feeder.fireShots(BALLS_PER_VOLLEY);
        }

        // ----- every loop, in this order: cameras, turrets, feeder -----
        stationary.update();
        turretCam.update();
        nectar.update(alliance);
        pollen.update(alliance);

        boolean bothReady = nectar.readyToShoot() && pollen.readyToShoot();
        feeder.update(bothReady);

        // When the volley is over, turn the flywheels off to save battery and heat.
        if (!feeder.isBusy()) {
            nectar.setFlywheelOn(false);
            pollen.setFlywheelOn(false);
        }

        // ----- screen -----
        telemetry.addData("Alliance", alliance.toUpperCase());
        telemetry.addData("Status", feeder.isBusy() ? "SHOOTING"
                : (bothReady ? "READY - press A" : "idle"));

        if (nectar.isTooFar() || pollen.isTooFar()) {
            telemetry.addLine("!! TOO FAR - drive closer !!");
        }

        telemetry.addData("Nectar cluster", nectar.getClusterName() == null ? "none" : nectar.getClusterName());
        telemetry.addData("Nectar ready / aim", "%s / %.1f deg", nectar.readyToShoot(), nectar.getAimErrorDeg());
        telemetry.addData("Nectar RPM", "%.0f / %.0f", nectar.getCurrentRpm(), nectar.getTargetRpm());
        telemetry.addData("Nectar dist (in)", "%.1f", nectar.getDistanceIn());
        telemetry.addData("Pollen cluster", pollen.getClusterName() == null ? "none" : pollen.getClusterName());
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
