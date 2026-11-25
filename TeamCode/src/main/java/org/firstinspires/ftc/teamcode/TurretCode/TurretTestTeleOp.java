package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Turret Test - try out both turrets, tune them, and build the shot tables.
 *
 * GAMEPAD 1
 *   Before PLAY:   X = BLUE alliance, A = RED alliance (only that alliance's clusters are used)
 *   X              pick which turret the other buttons change (Nectar <-> Pollen)
 *   Left bumper    Nectar flywheel on/off
 *   Right bumper   Pollen flywheel on/off
 *   Y              selected turret: AUTO (shot table) <-> MANUAL (you pick RPM + hood)
 *
 *   In MANUAL mode (use this to fill in the shot table):
 *     Dpad up/down      flywheel RPM +/- 50
 *     Dpad left/right   hood angle -/+ 1 degree
 *     -> Park at a distance, change RPM + hood until shots stay in, then copy the
 *        "distance", "hood" and "RPM" from the screen into that turret's shot table.
 *
 *   In AUTO mode (use this to tune the aiming):
 *     B                 change the step size
 *     Dpad left/right   aim kP -/+ step
 *     Dpad up/down      aim kD +/- step
 *     -> Write the good kP/kD numbers into the Turret's fields so they stick.
 *
 * CAMERAS (HuskyLens, names must match the robot configuration):
 *   "turretHusky" - camera on the NECTAR turret. Gives both aim and distance.
 *   POLLEN shares distance from turretHusky and has no aim camera.
 */
@TeleOp(name = "Turret Test", group = "Turret")
public class TurretTestTeleOp extends OpMode {

    private static final String DEFAULT_ALLIANCE = "blue";

    private final Turret nectar = Turret.nectar();
    private final Turret pollen = Turret.pollen();

    private boolean tuningNectar = true;     // which turret the buttons change
    private final double[] stepSizes = {0.01, 0.001, 0.0001, 0.00001};
    private int stepIndex = 1;

    private TagCamera turretCam;
    private String alliance = null;

    // Edge-detection button states
    private boolean lastX = false, lastY = false, lastB = false;
    private boolean lastLB = false, lastRB = false;
    private boolean lastDpadUp = false, lastDpadDown = false;
    private boolean lastDpadLeft = false, lastDpadRight = false;

    @Override
    public void init() {
        turretCam = TagCamera.init(hardwareMap, "turretHusky");

        nectar.init(hardwareMap, turretCam, turretCam);
        pollen.init(hardwareMap, turretCam, null);   // shares distance, no aim camera for Pollen

        telemetry.addLine("Turrets ready. Choose alliance: X = BLUE, A = RED");
    }

    /** Runs repeatedly before PLAY. Press X for blue or A for red. */
    @Override
    public void init_loop() {
        if (gamepad1.x) alliance = "blue";
        if (gamepad1.a) alliance = "red";

        telemetry.addData("Alliance", alliance == null ? "NOT SET (press X=blue or A=red)" : alliance.toUpperCase());
        telemetry.addLine("Press PLAY when ready.");
        telemetry.update();
    }

    @Override
    public void start() {
        if (alliance == null) {
            alliance = DEFAULT_ALLIANCE;
            telemetry.log().add("WARNING: No alliance selected! Defaulting to " + DEFAULT_ALLIANCE.toUpperCase());
        }
    }

    @Override
    public void loop() {
        // ----- buttons with single-press edge detection -----
        boolean xPressed = gamepad1.x && !lastX;
        boolean yPressed = gamepad1.y && !lastY;
        boolean bPressed = gamepad1.b && !lastB;
        boolean lbPressed = gamepad1.left_bumper && !lastLB;
        boolean rbPressed = gamepad1.right_bumper && !lastRB;
        boolean dpadUp = gamepad1.dpad_up && !lastDpadUp;
        boolean dpadDown = gamepad1.dpad_down && !lastDpadDown;
        boolean dpadLeft = gamepad1.dpad_left && !lastDpadLeft;
        boolean dpadRight = gamepad1.dpad_right && !lastDpadRight;

        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastB = gamepad1.b;
        lastLB = gamepad1.left_bumper;
        lastRB = gamepad1.right_bumper;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        if (xPressed) tuningNectar = !tuningNectar;
        if (lbPressed) nectar.setFlywheelOn(!nectar.isFlywheelOn());
        if (rbPressed) pollen.setFlywheelOn(!pollen.isFlywheelOn());

        handleTuningButtons(tuningNectar ? nectar : pollen, yPressed, bPressed, dpadUp, dpadDown, dpadLeft, dpadRight);

        // ----- read the camera ONCE, then run both turrets -----
        turretCam.update();
        nectar.update(alliance);
        pollen.update(alliance);

        // ----- screen -----
        telemetry.addData("Buttons change", tuningNectar
                ? "NECTAR (press X for Pollen)" : "POLLEN (press X for Nectar)");

        // Cluster the turret camera sees, counting only OUR alliance's clusters.
        TagClusters.Result cluster = TagClusters.identify(turretCam.getBlocks(), alliance);
        telemetry.addData("Alliance", alliance.toUpperCase());
        telemetry.addData("Our cluster in view", cluster.name == null ? "none"
                : String.format("%s (%d%% of tags)", cluster.name, cluster.percentFound()));
        telemetry.addLine();

        printTurret("NECTAR", nectar);
        printTurret("POLLEN", pollen);

        telemetry.addData("Step size (B)", stepSizes[stepIndex]);
        telemetry.addData("Nectar aim kP / kD", "%.5f / %.5f", nectar.aimKP, nectar.aimKD);
        telemetry.addData("Pollen aim kP / kD", "%.5f / %.5f", pollen.aimKP, pollen.aimKD);
        telemetry.update();
    }

    /** Print one turret's status block to telemetry. */
    private void printTurret(String label, Turret t) {
        String header = "=== " + label + " ===  " + (t.isManualMode() ? "MANUAL" : "AUTO");
        if (t.readyToShoot()) header += "   READY TO SHOOT";
        if (t.isTooFar())     header += "   !! TOO FAR";
        telemetry.addLine(header);
        telemetry.addData("Cluster", t.getClusterName() == null ? "none" : t.getClusterName());
        telemetry.addData("Tag seen", t.isTagVisible());
        telemetry.addData("Distance (in)", "%.1f", t.getDistanceIn());
        telemetry.addData("Aim error (deg)", "%.1f   turret at %.0f deg", t.getAimErrorDeg(), t.getTurretDeg());
        telemetry.addData("RPM", "%.0f / %.0f target", t.getCurrentRpm(), t.getTargetRpm());
        telemetry.addData("Hood (deg)", "%.1f (gear ratio: %.2f)", t.getTargetHoodDeg(), t.getHoodGearRatio());
        telemetry.addLine();
    }

    /**
     * Handle the dpad and tuning buttons for whichever turret is selected.
     * Works on any Turret instance with edge-triggered button states.
     */
    private void handleTuningButtons(Turret t, boolean yPressed, boolean bPressed,
                                     boolean dpadUp, boolean dpadDown, boolean dpadLeft, boolean dpadRight) {
        if (yPressed) t.setManualMode(!t.isManualMode());

        if (t.isManualMode()) {
            double rpm = t.getManualRpm(), hood = t.getManualHoodDeg();
            if (dpadUp)    rpm += 50;
            if (dpadDown)  rpm -= 50;
            if (dpadRight) hood += 1;
            if (dpadLeft)  hood -= 1;
            t.setManualShot(rpm, hood);
        } else {
            if (bPressed) stepIndex = (stepIndex + 1) % stepSizes.length;
            double step = stepSizes[stepIndex];
            if (dpadRight) t.aimKP += step;
            if (dpadLeft)  t.aimKP = Math.max(0, t.aimKP - step);
            if (dpadUp)    t.aimKD += step;
            if (dpadDown)  t.aimKD = Math.max(0, t.aimKD - step);
        }
    }

    @Override
    public void stop() {
        nectar.stop();
        pollen.stop();
    }
}
