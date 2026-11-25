package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TurretCode.Turret;
import org.firstinspires.ftc.teamcode.TurretCode.TagCamera;
import org.firstinspires.ftc.teamcode.TurretCode.TurretAutoLogic;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

/**
 * BlueForagerAuto - Forager Autonomous for BLUE Alliance:
 * Mirrored mathematically from RedForagerAuto across the X = 72.0" field centerline.
 *
 * Sequence of Actions:
 *  1. SPRINT & CORRAL 1:
 *     Fires 4 preloaded pollen into Hive 2 (causing Hive 2 to tip & pop Hive 1 upright),
 *     while continuously driving forward with intake running to corral/vacuum spilled floor elements.
 *  2. SPRINT UPFIELD + SHOOT MOVE 1:
 *     Accelerates upfield along the X ≈ 81" corridor while turret locks onto Hive 1 and fires the 4
 *     corralled balls on the move
 *  2b. CHECK HIVE 1 TIPPING & ADAPTIVE FLOWER RECOVERY:
 *     - If Hive 1 tips (AprilTags disappear): Sweeps upfield corridor (CORRAL 2) for freshly spilled floor balls.
 *     - If Hive 1 does NOT tip: Diverts to upfield Flower 1, intakes 4 fresh balls, shoots them into Hive 1
 *       to tip it, and then sweeps the upfield floor for the spilled balls
 *  3. CORRAL 2 (UPFIELD SWEEP):
 *     Sweeps across the upfield corridor with intake running to corral the freshly spilled floor balls.
 *  4. SPRINT DOWNFIELD + SHOOT MOVE 2:
 *     Curves downfield while turret locks onto Hive 2 (now upright again!) and fires the 4 balls on the move!
 *     (Total of 12 balls scored in ~18-20 seconds).
 *  5.TIME-GATED ENDGAME:
 *     - If match time < 7.5s: Sprints straight to close park.
 *     - If match time >= 7.5s (Bonus): Intakes Garden corner, scores Garden volley, taps Alliance Flower, and parks!
 *
 * Mirror formulas:
 *   X_blue = 144.0 - X_red
 *   Y_blue = Y_red
 *   Heading_blue = 180.0 - Heading_red (mod 360)
 *
 * Coordinates in inches, headings in degrees (PoseFactory.degrees()).
 */
@Autonomous(name = "Blue Forager Auto", group = "Autonomous")
public class BlueForagerAuto extends OpMode {

    private static final String ALLIANCE = "blue";

    // ---------- TUNE: timing and hardware values ----------
    private static final double AIM_MAX_SECONDS           = 1.5;   // max time to wait for turrets to aim
    private static final int    SHOTS_PER_VOLLEY          = 4;     // balls fed per volley
    private static final double GARDEN_INTAKE_SECONDS     = 1.0;   // dwell time at garden
    private static final double FLOWER_INTAKE_SECONDS     = 0.6;   // quick tap dwell at alliance flower
    private static final double HIVE_TIP_CHECK_SECONDS    = 0.5;   // time to observe if Hive 1 is tipping
    private static final double FLOWER_ONE_INTAKE_SECONDS = 1.2;   // dwell time to intake from Flower 1
    private static final double INTAKE_POWER              = 1.0;   // intake motor power
    private static final double MATCH_DURATION_SECONDS    = 30.0;
    private static final double BONUS_TIME_THRESHOLD      = 7.5;   // Go for Garden bonus only if >= 7.5s left
    private static final double FLOWER_TIME_THRESHOLD     = 4.5;   // Tap Alliance Flower if >= 4.5s left

    // ---------- Positions: Mirrored 144.0 - X_red across field centerline ----------
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose                  = poseFactory.of(89.185, 14.518, 90);
    private final Pose corralSweepOne             = poseFactory.of(82.667, 22.568, 60);
    private final Pose upfieldCorridor            = poseFactory.of(81.247, 101.913, 90);
    private final Pose intakeFlowerOne            = poseFactory.of(96.853, 132.632, 90);
    private final Pose shootHiveOneAfterFlower    = poseFactory.of(97.018, 108.624, 270);
    private final Pose corralSweepTwoPrep         = poseFactory.of(90.332, 118.939, 0);
    private final Pose corralSweepTwo             = poseFactory.of(114.909, 100.747, 270);
    private final Pose downfieldHome              = poseFactory.of(96.698, 8.221, 0);
    private final Pose intakeGarden               = poseFactory.of(136.581, 7.691, 0);
    private final Pose shootGarden                = poseFactory.of(89.248, 23.899, 90);
    private final Pose allianceFlower             = poseFactory.of(136.371, 46.602, 0);
    private final Pose parkPose                   = poseFactory.of(136.298, 95.799, 90);

    // ---------- Pedro Paths ----------
    private Path toCorralSweepOne, toUpfieldCorridor, toCorralSweepTwoPrep,
                 toCorralSweepTwo, toDownfieldHome, toIntakeGarden, toShootGarden,
                 toAllianceFlower, toParkFromFlower, toParkDirect, toParkFromGardenShot,
                 toFlowerOne, toShootHiveOneAfterFlower, toCorralSweepTwoFromShot;

    // ---------- Hardware ----------
    private Follower follower;
    private TagCamera turretCam;
    private final Turret nectar = Turret.nectar();
    private final Turret pollen = Turret.pollen();
    private final TurretAutoLogic feeder = new TurretAutoLogic();
    private DcMotor intakeMotor;

    // ---------- State Machine ----------
    private enum State {
        // Phase 1: Preload shot + First Floor Corral
        SHOOT_PRELOAD_AND_CORRAL_ONE,

        // Phase 2: Sprint upfield & Shoot on the Move into Hive 1
        SPRINT_UPFIELD_AND_SHOOT_MOVE_ONE,

        // Phase 2b: Check if Hive 1 is tipping / tipped
        CHECK_HIVE_ONE_TIPPING,

        // Phase 2c: Fallback diversion if Hive 1 did NOT tip -> intake from Flower 1, shoot Hive 1, then sweep
        DRIVE_TO_FLOWER_ONE,
        INTAKE_FLOWER_ONE,
        DRIVE_TO_SHOOT_HIVE_ONE_AFTER_FLOWER,
        AIM_HIVE_ONE_AFTER_FLOWER,
        SHOOT_HIVE_ONE_AFTER_FLOWER,
        CORRAL_SWEEP_TWO_FROM_SHOT,

        // Phase 3: Upfield Floor Corral (spill from Hive 1, if it tipped originally)
        CORRAL_SWEEP_TWO_PREP,
        CORRAL_SWEEP_TWO,

        // Phase 4: Sprint downfield & Shoot on the Move into Hive 2
        SPRINT_DOWNFIELD_AND_SHOOT_MOVE_TWO,

        // Phase 5: Time-gated Endgame
        DRIVE_TO_INTAKE_GARDEN,
        INTAKE_GARDEN,
        DRIVE_TO_SHOOT_GARDEN,
        AIM_SHOOT_GARDEN,
        SHOOT_GARDEN,
        DRIVE_TO_ALLIANCE_FLOWER,
        DWELL_ALLIANCE_FLOWER,          // Bug 2 fix: separate dwell state so timer starts AFTER path ends
        DRIVE_TO_PARK_FROM_FLOWER,

        // Direct failsafe parks
        DRIVE_TO_PARK_DIRECT,
        DRIVE_TO_PARK_FROM_GARDEN,

        DONE
    }

    private State state = State.SHOOT_PRELOAD_AND_CORRAL_ONE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean intakedFromFlowerOne = false;

    @Override
    public void init() {
        buildPaths();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        turretCam     = TagCamera.init(hardwareMap, "turretHusky");
        nectar.init(hardwareMap, turretCam, turretCam);
        pollen.init(hardwareMap, turretCam, null);
        pollen.setMasterTurret(nectar);
        feeder.init(hardwareMap);

        intakeMotor = hardwareMap.tryGet(DcMotor.class, "intakeMotor");
        if (intakeMotor == null) telemetry.addLine("WARNING: intakeMotor not found!");

        telemetry.addData("Alliance", ALLIANCE.toUpperCase());
        telemetry.addLine("Blue Forager Auto Initialized.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        telemetry.addData("Alliance", ALLIANCE.toUpperCase());
        telemetry.addData("Pose", "X: %.1f, Y: %.1f, Heading: %.1f deg",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
        telemetry.addLine("Ready. Press PLAY.");
        telemetry.update();
    }

    @Override
    public void start() {
        matchTimer.reset();
        Constants.resetPodEncoders();
        intakedFromFlowerOne = false;
        setState(State.SHOOT_PRELOAD_AND_CORRAL_ONE);
    }

    @Override
    public void loop() {
        // Continuous updates every loop
        follower.update();
        turretCam.update();
        nectar.update(ALLIANCE);
        pollen.update(ALLIANCE);
        if (nectar.isTagVisible()) {
            pollen.setTurretAngle(nectar.getTurretDeg());
        }
        feeder.update(turretsReady());

        switch (state) {
            // 1. Shoot preloads into Hive 2 and drive forward to corral spilled balls
            case SHOOT_PRELOAD_AND_CORRAL_ONE:
                if (stateTimer.seconds() > 0.05 && !follower.isBusy() && !feeder.isBusy()) {
                    setState(State.SPRINT_UPFIELD_AND_SHOOT_MOVE_ONE);
                }
                break;

            // 2. Sprint upfield along corridor while shooting corralled balls on the move into Hive 1
            case SPRINT_UPFIELD_AND_SHOOT_MOVE_ONE:
                if (stateTimer.seconds() > 0.05 && !follower.isBusy() && !feeder.isBusy()) {
                    setState(State.CHECK_HIVE_ONE_TIPPING);
                }
                break;

            // 2b. Check if Hive 1 is tipping / tipped
            case CHECK_HIVE_ONE_TIPPING:
                // If Hive 1 tipped (tag cluster lost / tipped away), proceed to corral spilled floor balls
                if (stateTimer.seconds() >= 0.25 && nectar.getClusterName() == null) {
                    setState(State.CORRAL_SWEEP_TWO_PREP);
                } else if (stateTimer.seconds() >= HIVE_TIP_CHECK_SECONDS) {
                    // Timeout elapsed and Hive 1 tag is still visible -> Hive 1 did NOT tip!
                    // Divert to upfield flower to intake fresh balls
                    setState(State.DRIVE_TO_FLOWER_ONE);
                }
                break;

            // 2c. Divert to upfield Flower 1
            case DRIVE_TO_FLOWER_ONE:
                if (!follower.isBusy()) {
                    setState(State.INTAKE_FLOWER_ONE);
                }
                break;

            // 2d. Intake from upfield Flower 1
            case INTAKE_FLOWER_ONE:
                if (stateTimer.seconds() >= FLOWER_ONE_INTAKE_SECONDS) {
                    intakeMotor.setPower(0);
                    intakedFromFlowerOne = true;
                    setState(State.DRIVE_TO_SHOOT_HIVE_ONE_AFTER_FLOWER);
                }
                break;

            // 2e. Drive to shoot position facing Hive 1
            case DRIVE_TO_SHOOT_HIVE_ONE_AFTER_FLOWER:
                if (!follower.isBusy()) {
                    setState(turretsReady() ? State.SHOOT_HIVE_ONE_AFTER_FLOWER : State.AIM_HIVE_ONE_AFTER_FLOWER);
                }
                break;

            // 2f. Aim at Hive 1 with flower balls
            case AIM_HIVE_ONE_AFTER_FLOWER:
                if (turretsReady() || stateTimer.seconds() >= AIM_MAX_SECONDS) {
                    setState(State.SHOOT_HIVE_ONE_AFTER_FLOWER);
                }
                break;

            // 2g. Shoot into Hive 1 to finish tipping it
            case SHOOT_HIVE_ONE_AFTER_FLOWER:
                if (!feeder.isBusy()) {
                    // Hive 1 tipped and spilled balls! Now sweep across upfield floor for new balls
                    setState(State.CORRAL_SWEEP_TWO_FROM_SHOT);
                }
                break;

            // 2h. Sweep across upfield floor after flower shot
            case CORRAL_SWEEP_TWO_FROM_SHOT:
                if (!follower.isBusy()) {
                    setState(State.SPRINT_DOWNFIELD_AND_SHOOT_MOVE_TWO);
                }
                break;

            // 3. Approach upfield sweep corridor
            case CORRAL_SWEEP_TWO_PREP:
                if (!follower.isBusy()) {
                    setState(State.CORRAL_SWEEP_TWO);
                }
                break;

            // 4. Corral spilled balls across upfield floor
            case CORRAL_SWEEP_TWO:
                if (!follower.isBusy()) {
                    setState(State.SPRINT_DOWNFIELD_AND_SHOOT_MOVE_TWO);
                }
                break;

            // 5. Sprint downfield along curve while shooting on the move into Hive 2
            case SPRINT_DOWNFIELD_AND_SHOOT_MOVE_TWO:
                if (stateTimer.seconds() > 0.05 && !follower.isBusy() && !feeder.isBusy()) {
                    // Check remaining time: if >= 7.5s, grab Garden bonus! Else park immediately
                    if (getRemainingMatchSeconds() >= BONUS_TIME_THRESHOLD) {
                        setState(State.DRIVE_TO_INTAKE_GARDEN);
                    } else {
                        setState(State.DRIVE_TO_PARK_DIRECT);
                    }
                }
                break;

            // 6. Bonus: Drive into Garden corner
            case DRIVE_TO_INTAKE_GARDEN:
                if (!follower.isBusy()) {
                    setState(State.INTAKE_GARDEN);
                }
                break;

            // 7. Bonus: Dwell at Garden to collect
            case INTAKE_GARDEN:
                if (stateTimer.seconds() >= GARDEN_INTAKE_SECONDS) {
                    intakeMotor.setPower(0);
                    setState(State.DRIVE_TO_SHOOT_GARDEN);
                }
                break;

            // 8. Bonus: Drive to Home Garden shot position
            case DRIVE_TO_SHOOT_GARDEN:
                if (!follower.isBusy()) {
                    // Skip AIM state if flywheels are already up to speed during the drive
                    setState(turretsReady() ? State.SHOOT_GARDEN : State.AIM_SHOOT_GARDEN);
                }
                break;

            // 9. Bonus: Aim Garden volley
            case AIM_SHOOT_GARDEN:
                if (turretsReady() || stateTimer.seconds() >= AIM_MAX_SECONDS) {
                    setState(State.SHOOT_GARDEN);
                }
                break;

            // 10. Bonus: Shoot Garden volley
            case SHOOT_GARDEN:
                if (!feeder.isBusy()) {
                    if (getRemainingMatchSeconds() >= FLOWER_TIME_THRESHOLD) {
                        setState(State.DRIVE_TO_ALLIANCE_FLOWER);
                    } else {
                        setState(State.DRIVE_TO_PARK_FROM_GARDEN);
                    }
                }
                break;

            // 11. Bonus: Tap Alliance Flower
            case DRIVE_TO_ALLIANCE_FLOWER:
                if (!follower.isBusy()) setState(State.DWELL_ALLIANCE_FLOWER);
                break;

            // 11b. Dwell at flower until intake has had time to grab (timer starts AFTER path ends)
            case DWELL_ALLIANCE_FLOWER:
                if (stateTimer.seconds() >= FLOWER_INTAKE_SECONDS)
                    setState(State.DRIVE_TO_PARK_FROM_FLOWER);
                break;

            // 12. Final Parks
            case DRIVE_TO_PARK_FROM_FLOWER:
            case DRIVE_TO_PARK_DIRECT:
            case DRIVE_TO_PARK_FROM_GARDEN:
                if (!follower.isBusy()) {
                    setState(State.DONE);
                }
                break;

            case DONE:
                break;
        }

        // Telemetry diagnostics
        telemetry.addData("Alliance", ALLIANCE.toUpperCase());
        telemetry.addData("Time Left", "%.1f s", getRemainingMatchSeconds());
        telemetry.addData("State", state);
        telemetry.addData("Intaked Flower 1", intakedFromFlowerOne);
        telemetry.addData("X", "%.1f", follower.pose().x());
        telemetry.addData("Y", "%.1f", follower.pose().y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Feeder Busy", feeder.isBusy());
        telemetry.addData("Hive Cluster", nectar.getClusterName() == null ? "NONE" : nectar.getClusterName());
        telemetry.addData("Nectar Ready", nectar.readyToShoot());
        telemetry.addData("Pollen Ready", pollen.readyToShoot());
        telemetry.update();
    }

    @Override
    public void stop() {
        nectar.stop();
        pollen.stop();
        feeder.stop();
        intakeMotor.setPower(0);
    }

    private double getRemainingMatchSeconds() {
        return Math.max(0, MATCH_DURATION_SECONDS - matchTimer.seconds());
    }

    private boolean turretsReady() {
        return nectar.readyToShoot() && pollen.readyToShoot();
    }

    private void setFlywheels(boolean on) {
        nectar.setFlywheelOn(on);
        pollen.setFlywheelOn(on);
    }

    private void setState(State next) {
        state = next;
        stateTimer.reset();

        switch (state) {
            case SHOOT_PRELOAD_AND_CORRAL_ONE:
                setFlywheels(true);
                feeder.fireShots(SHOTS_PER_VOLLEY);       // fire preloads immediately into Hive 2
                intakeMotor.setPower(INTAKE_POWER);        // start vacuuming floor elements
                follower.follow(toCorralSweepOne);
                break;

            case SPRINT_UPFIELD_AND_SHOOT_MOVE_ONE:
                intakeMotor.setPower(0);
                setFlywheels(true);
                feeder.fireShots(SHOTS_PER_VOLLEY);       // shoot corralled balls on the move into Hive 1
                follower.follow(toUpfieldCorridor);
                break;

            case CHECK_HIVE_ONE_TIPPING:
                setFlywheels(false);
                intakeMotor.setPower(0);
                break;

            case DRIVE_TO_FLOWER_ONE:
                setFlywheels(false);
                intakeMotor.setPower(INTAKE_POWER);
                follower.follow(toFlowerOne);
                break;

            case INTAKE_FLOWER_ONE:
                intakeMotor.setPower(INTAKE_POWER);
                break;

            case DRIVE_TO_SHOOT_HIVE_ONE_AFTER_FLOWER:
                intakeMotor.setPower(0);
                setFlywheels(true); // spin up early
                follower.follow(toShootHiveOneAfterFlower);
                break;

            case AIM_HIVE_ONE_AFTER_FLOWER:
                setFlywheels(true);
                break;

            case SHOOT_HIVE_ONE_AFTER_FLOWER:
                setFlywheels(true);
                feeder.fireShots(SHOTS_PER_VOLLEY); // shoot into Hive 1 to tip it
                break;

            case CORRAL_SWEEP_TWO_FROM_SHOT:
                setFlywheels(false);
                intakeMotor.setPower(INTAKE_POWER); // vacuum newly spilled floor balls
                follower.follow(toCorralSweepTwoFromShot);
                break;

            case CORRAL_SWEEP_TWO_PREP:
                setFlywheels(false);
                intakeMotor.setPower(INTAKE_POWER);
                follower.follow(toCorralSweepTwoPrep);
                break;

            case CORRAL_SWEEP_TWO:
                intakeMotor.setPower(INTAKE_POWER);
                follower.follow(toCorralSweepTwo);
                break;

            case SPRINT_DOWNFIELD_AND_SHOOT_MOVE_TWO:
                intakeMotor.setPower(0);
                setFlywheels(true);
                feeder.fireShots(SHOTS_PER_VOLLEY);       // shoot balls into Hive 2
                follower.follow(toDownfieldHome);
                break;

            case DRIVE_TO_INTAKE_GARDEN:
                setFlywheels(false);
                intakeMotor.setPower(INTAKE_POWER);
                follower.follow(toIntakeGarden);
                break;

            case INTAKE_GARDEN:
                intakeMotor.setPower(INTAKE_POWER);
                break;

            case DRIVE_TO_SHOOT_GARDEN:
                intakeMotor.setPower(0);
                setFlywheels(true);   // spin up early during the drive to save ~1.5s aiming time
                follower.follow(toShootGarden);
                break;

            case AIM_SHOOT_GARDEN:
                setFlywheels(true);
                break;

            case SHOOT_GARDEN:
                feeder.fireShots(SHOTS_PER_VOLLEY);
                break;

            case DRIVE_TO_ALLIANCE_FLOWER:
                setFlywheels(false);
                intakeMotor.setPower(INTAKE_POWER);
                follower.follow(toAllianceFlower);
                break;

            case DWELL_ALLIANCE_FLOWER:
                // intake keeps running from DRIVE_TO_ALLIANCE_FLOWER; nothing new to start
                break;

            case DRIVE_TO_PARK_FROM_FLOWER:
                intakeMotor.setPower(0);
                follower.follow(toParkFromFlower);
                break;

            case DRIVE_TO_PARK_DIRECT:
                setFlywheels(false);
                intakeMotor.setPower(0);
                follower.follow(toParkDirect);
                break;

            case DRIVE_TO_PARK_FROM_GARDEN:
                setFlywheels(false);
                intakeMotor.setPower(0);
                follower.follow(toParkFromGardenShot);
                break;

            case DONE:
                setFlywheels(false);
                feeder.stop();
                intakeMotor.setPower(0);
                break;
        }
    }

    /**
     * Builds all paths for BlueForagerAuto (mirrored across X = 72.0).
     */
    private void buildPaths() {
        // 1. Start wall to First Corral Sweep: linear heading 90 -> 60 while shooting preloads
        toCorralSweepOne = line(startPose, corralSweepOne)
                .linear(startPose, corralSweepOne);

        // 2. Sprint upfield along corridor: linear heading 60 -> 90 while shooting on the move into Hive 1
        toUpfieldCorridor = line(corralSweepOne, upfieldCorridor)
                .linear(corralSweepOne, upfieldCorridor);

        // 2b. Upfield corridor to Flower 1 diversion (if Hive 1 is NOT tipping): constant heading 90
        toFlowerOne = line(upfieldCorridor, intakeFlowerOne)
                .constant(intakeFlowerOne.heading());

        // 2c. Flower 1 to shoot position facing Hive 1: linear heading 90 -> 270
        toShootHiveOneAfterFlower = line(intakeFlowerOne, shootHiveOneAfterFlower)
                .linear(intakeFlowerOne, shootHiveOneAfterFlower);

        // 2d. Sweep across upfield floor from shot to corralSweepTwo: constant heading 270
        toCorralSweepTwoFromShot = line(shootHiveOneAfterFlower, corralSweepTwo)
                .constant(corralSweepTwo.heading());

        // 3. Upfield corridor to corral sweep prep: linear heading 90 -> 0
        toCorralSweepTwoPrep = line(upfieldCorridor, corralSweepTwoPrep)
                .linear(upfieldCorridor, corralSweepTwoPrep);

        // 4. Corral sweep across upfield floor: linear heading 0 -> 270 intaking spilled balls
        toCorralSweepTwo = line(corralSweepTwoPrep, corralSweepTwo)
                .linear(corralSweepTwoPrep, corralSweepTwo);

        // 5. Downfield sprint curve: Bézier (CP: 120.752, 52.473) linear heading 270 -> 0 shooting on move into Hive 2
        toDownfieldHome = bezier(corralSweepTwo, downfieldHome, 120.752, 52.473)
                .linear(corralSweepTwo, downfieldHome);

        // 6. Bonus: to Garden corner facing 0
        toIntakeGarden = line(downfieldHome, intakeGarden)
                .constant(intakeGarden.heading());

        // 7. Bonus: Garden to shoot position, linear heading 0 -> 90
        toShootGarden = line(intakeGarden, shootGarden)
                .linear(intakeGarden, shootGarden);

        // 8. Bonus: Shoot position to Alliance Flower, linear heading 90 -> 0
        toAllianceFlower = line(shootGarden, allianceFlower)
                .linear(shootGarden, allianceFlower);

        // 9. Park from Flower: straight to park facing 90
        toParkFromFlower = line(allianceFlower, parkPose)
                .constant(parkPose.heading());

        // 10. Direct Park from Downfield Home (failsafe if time < 7.5s)
        toParkDirect = line(downfieldHome, parkPose)
                .linear(downfieldHome, parkPose);

        // 11. Park from Garden Shot (failsafe if time < 4.5s)
        toParkFromGardenShot = line(shootGarden, parkPose)
                .linear(shootGarden, parkPose);
    }

    private static Path bezier(Pose from, Pose to, double... controlXY) {
        Vector2D[] points = new Vector2D[controlXY.length / 2 + 2];
        points[0] = Vector2D.cartesian(from.x(), from.y());
        for (int i = 0; i < controlXY.length / 2; i++) {
            points[i + 1] = Vector2D.cartesian(controlXY[2 * i], controlXY[2 * i + 1]);
        }
        points[points.length - 1] = Vector2D.cartesian(to.x(), to.y());
        return curve(points);
    }
}
