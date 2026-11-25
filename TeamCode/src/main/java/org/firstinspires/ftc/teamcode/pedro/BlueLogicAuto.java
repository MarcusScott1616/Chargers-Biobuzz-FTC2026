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

/*
 * BlueLogicAuto - a STATE MACHINE autonomous for the BLUE Alliance.
 *
 * All coordinates, headings, and control points are mirrored from RED_2.5Tip_FarPark.pp
 * matching BLUE_2.5Tip_FarPark.pp.json across the field centerline.
 *
 * Coordinates are in inches, heading in degrees (PoseFactory.degrees()).
 */
@Autonomous(name = "Blue Logic Auto", group = "Autonomous")
public class BlueLogicAuto extends OpMode {

    private static final String ALLIANCE = "blue";

    // ---------- TUNE: timing and hardware values (measure these on the robot) ----------
    private static final double AIM_MAX_SECONDS = 1.5;   // stop waiting for the turrets after this long, then shoot anyway
    private static final int SHOTS_PER_VOLLEY = 4;       // balls fed per SHOOT state. Match this to the real volley.
    private static final double INTAKE_SECONDS = 1.5;    // how long the intake runs at the flowers
    private static final double GARDEN_INTAKE_SECONDS = 1.0; // dwell time to collect balls at the garden
    private static final double INTAKE_POWER = 1.0;      // intake motor power
    private static final double MATCH_DURATION_SECONDS = 30.0;
    private static final double PARK_TIME_THRESHOLD    = 7.0; // Park if less than 7s remaining in match
    private static final double CHECK_HIVE_TIMEOUT     = 0.4; // Max seconds to wait for tag cluster before deciding tipped

    // ---------- every position the robot visits (exact mirrored points from BLUE_2.5Tip_FarPark.pp.json) ----------
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(85.810, 15.431, 90);
    private final Pose faceGarden = poseFactory.of(110.654, 9.330, 0);
    private final Pose intakeGarden = poseFactory.of(133.604, 8.639, 0);
    private final Pose otherSideSecondVolley = poseFactory.of(84.220, 118.126, 0);
    private final Pose intakeFlowerOne = poseFactory.of(94.353, 132.632, 90);
    private final Pose otherSideShootThirdVolley = poseFactory.of(94.518, 108.624, 270);
    private final Pose intakeFlowerTwo = poseFactory.of(131.953, 47.539, 0);
    private final Pose shootFourthVolley = poseFactory.of(98.731, 24.418, 90);
    private final Pose goToPark = poseFactory.of(119.174, 111.687, 0);
    private final Pose parkFarSide = poseFactory.of(133.472, 111.595, 0);

    // ---------- the paths between those positions (built once in init) ----------
    private Path toFaceGarden, toIntakeGarden, toSecondShot, toFlowerOne, toThirdShot,
            toFlowerTwo, toFourthShot, toLineUpPark, toPark,
            toFourthShotDirect, toThirdShotDirect, toLineUpParkFromThird,
            toFlowerTwoFromFourth;

    // ---------- hardware ----------
    private TagCamera stationaryCam, turretCam;             // stationary = distance, turret = aim
    private final Turret nectar = Turret.nectar();
    private final Turret pollen = Turret.pollen();
    private final TurretAutoLogic feeder = new TurretAutoLogic();
    private DcMotor intakeMotor;                             // goBILDA motor on the intake

    // ---------- the states ----------
    private enum State {
        DRIVE_TO_FACE_GARDEN,
        DRIVE_TO_INTAKE_GARDEN,
        INTAKE_GARDEN,
        DRIVE_TO_SECOND_SHOT,
        AIM_SECOND,
        SHOOT_SECOND,
        DRIVE_TO_FLOWER_ONE,
        INTAKE_FLOWER_ONE,
        DRIVE_TO_THIRD_SHOT,
        CHECK_THIRD_HIVE,
        AIM_THIRD,
        SHOOT_THIRD,
        DRIVE_TO_FOURTH_SHOT_DIRECT,
        CHECK_FOURTH_HIVE_ALT,
        AIM_FOURTH_ALT,
        SHOOT_FOURTH_ALT,
        DRIVE_TO_FLOWER_TWO,
        DRIVE_TO_FLOWER_TWO_FROM_FOURTH,
        INTAKE_FLOWER_TWO,
        DRIVE_TO_FOURTH_SHOT,
        CHECK_FOURTH_HIVE,
        AIM_FOURTH,
        SHOOT_FOURTH,
        DRIVE_TO_THIRD_SHOT_DIRECT,
        CHECK_THIRD_HIVE_FINAL,
        AIM_THIRD_FINAL,
        SHOOT_THIRD_FINAL,
        DRIVE_TO_LINE_UP_PARK,
        DRIVE_TO_LINE_UP_PARK_FROM_THIRD,
        DRIVE_TO_PARK,
        DONE
    }

    private Follower follower;
    private State state = State.DRIVE_TO_FACE_GARDEN;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean hiveTwoTipped = false;

    @Override
    public void init() {
        buildPaths();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose); // where the robot is placed at the start of the match

        stationaryCam = TagCamera.init(hardwareMap, "stationaryHusky");
        turretCam = TagCamera.init(hardwareMap, "turretHusky");
        nectar.init(hardwareMap, stationaryCam, turretCam);
        pollen.init(hardwareMap, stationaryCam, null);   // Pollen has no aim camera
        feeder.init(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        telemetry.addData("Alliance", ALLIANCE.toUpperCase());
        telemetry.addLine("Autonomous ready.");
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
        Constants.resetPodEncoders(); // Zero pods to their straightened positions on PLAY
        hiveTwoTipped = false;
        setState(State.DRIVE_TO_FACE_GARDEN); // start the first step
    }

    @Override
    public void loop() {
        // Everything that must run every loop: the follower, the cameras, the turrets, the feeder.
        follower.update();
        stationaryCam.update();
        turretCam.update();
        nectar.update(ALLIANCE);
        pollen.update(ALLIANCE);
        pollen.setTurretAngle(nectar.getTurretDeg());
        feeder.update(turretsReady());

        switch (state) {
            // ----- driving states: wait for the path to finish -----
            case DRIVE_TO_FACE_GARDEN:
                if (!follower.isBusy()) setState(State.DRIVE_TO_INTAKE_GARDEN);
                break;
            case DRIVE_TO_INTAKE_GARDEN:
                if (!follower.isBusy()) setState(State.INTAKE_GARDEN);
                break;
            case INTAKE_GARDEN:
                if (stateTimer.seconds() >= GARDEN_INTAKE_SECONDS) {
                    intakeMotor.setPower(0);
                    setState(State.DRIVE_TO_SECOND_SHOT);
                }
                break;
            case DRIVE_TO_SECOND_SHOT:
                if (!follower.isBusy()) setState(State.AIM_SECOND);
                break;
            case DRIVE_TO_FLOWER_ONE:
                if (!follower.isBusy()) setState(State.INTAKE_FLOWER_ONE);
                break;
            case INTAKE_FLOWER_ONE:
                if (stateTimer.seconds() >= INTAKE_SECONDS) {
                    intakeMotor.setPower(0);
                    setState(State.DRIVE_TO_THIRD_SHOT);
                }
                break;
            case DRIVE_TO_THIRD_SHOT:
                if (!follower.isBusy()) setState(State.CHECK_THIRD_HIVE);
                break;
            case DRIVE_TO_FOURTH_SHOT_DIRECT:
                if (!follower.isBusy()) setState(State.CHECK_FOURTH_HIVE_ALT);
                break;
            case DRIVE_TO_FLOWER_TWO:
            case DRIVE_TO_FLOWER_TWO_FROM_FOURTH:
                if (!follower.isBusy()) setState(State.INTAKE_FLOWER_TWO);
                break;
            case DRIVE_TO_FOURTH_SHOT:
                if (!follower.isBusy()) setState(State.CHECK_FOURTH_HIVE);
                break;
            case DRIVE_TO_THIRD_SHOT_DIRECT:
                if (!follower.isBusy()) setState(State.CHECK_THIRD_HIVE_FINAL);
                break;
            case DRIVE_TO_LINE_UP_PARK:
            case DRIVE_TO_LINE_UP_PARK_FROM_THIRD:
                if (!follower.isBusy()) setState(State.DRIVE_TO_PARK);
                break;
            case DRIVE_TO_PARK:
                if (!follower.isBusy()) setState(State.DONE);
                break;

            // ----- check if the hive is tipped before aiming / shooting -----
            case CHECK_THIRD_HIVE:
                if (getRemainingMatchSeconds() < PARK_TIME_THRESHOLD) {
                    setState(State.DRIVE_TO_LINE_UP_PARK_FROM_THIRD);
                } else if (nectar.getClusterName() != null) {
                    setState(State.AIM_THIRD);
                } else if (stateTimer.seconds() >= CHECK_HIVE_TIMEOUT) {
                    // Hive at Shoot Tip 2 is tipped -> take Flower 1's volley over to other side (Shoot Tip 3)
                    hiveTwoTipped = true;
                    setState(State.DRIVE_TO_FOURTH_SHOT_DIRECT);
                }
                break;

            case CHECK_FOURTH_HIVE_ALT:
                if (getRemainingMatchSeconds() < PARK_TIME_THRESHOLD) {
                    setState(State.DRIVE_TO_LINE_UP_PARK);
                } else if (nectar.getClusterName() != null) {
                    setState(State.AIM_FOURTH_ALT);
                } else if (stateTimer.seconds() >= CHECK_HIVE_TIMEOUT) {
                    // Both Hive 2 and Hive 3 are tipped -> park immediately
                    setState(State.DRIVE_TO_LINE_UP_PARK);
                }
                break;

            case CHECK_FOURTH_HIVE:
                if (getRemainingMatchSeconds() < PARK_TIME_THRESHOLD) {
                    setState(State.DRIVE_TO_LINE_UP_PARK);
                } else if (nectar.getClusterName() != null) {
                    setState(State.AIM_FOURTH);
                } else if (stateTimer.seconds() >= CHECK_HIVE_TIMEOUT) {
                    // Hive at Shoot Tip 3 is tipped: if Hive 2 was already tipped or low time, park
                    if (hiveTwoTipped || getRemainingMatchSeconds() < PARK_TIME_THRESHOLD) {
                        setState(State.DRIVE_TO_LINE_UP_PARK);
                    } else {
                        setState(State.DRIVE_TO_THIRD_SHOT_DIRECT);
                    }
                }
                break;

            case CHECK_THIRD_HIVE_FINAL:
                if (getRemainingMatchSeconds() < PARK_TIME_THRESHOLD) {
                    setState(State.DRIVE_TO_LINE_UP_PARK_FROM_THIRD);
                } else if (nectar.getClusterName() != null) {
                    setState(State.AIM_THIRD_FINAL);
                } else if (stateTimer.seconds() >= CHECK_HIVE_TIMEOUT) {
                    // Both hives tipped -> park immediately
                    setState(State.DRIVE_TO_LINE_UP_PARK_FROM_THIRD);
                }
                break;

            // ----- aim: wait for both turrets to be ready (or give up after a while) -----
            case AIM_SECOND:
                if (turretsReady() || stateTimer.seconds() >= AIM_MAX_SECONDS) setState(State.SHOOT_SECOND);
                break;
            case AIM_THIRD:
                if (turretsReady() || stateTimer.seconds() >= AIM_MAX_SECONDS) setState(State.SHOOT_THIRD);
                break;
            case AIM_FOURTH_ALT:
                if (turretsReady() || stateTimer.seconds() >= AIM_MAX_SECONDS) setState(State.SHOOT_FOURTH_ALT);
                break;
            case AIM_FOURTH:
                if (turretsReady() || stateTimer.seconds() >= AIM_MAX_SECONDS) setState(State.SHOOT_FOURTH);
                break;
            case AIM_THIRD_FINAL:
                if (turretsReady() || stateTimer.seconds() >= AIM_MAX_SECONDS) setState(State.SHOOT_THIRD_FINAL);
                break;

            // ----- shoot: wait for TurretAutoLogic to finish feeding -----
            case SHOOT_SECOND:
                if (!feeder.isBusy()) setState(State.DRIVE_TO_FLOWER_ONE);
                break;

            case SHOOT_THIRD:
                // Shot volley from Flower 1 at Shoot Tip 2 -> go to Flower 2
                if (!feeder.isBusy()) {
                    if (getRemainingMatchSeconds() < PARK_TIME_THRESHOLD) {
                        setState(State.DRIVE_TO_LINE_UP_PARK_FROM_THIRD);
                    } else {
                        setState(State.DRIVE_TO_FLOWER_TWO);
                    }
                }
                break;

            case SHOOT_FOURTH_ALT:
                // Shot volley from Flower 1 at Shoot Tip 3 -> go to Flower 2
                if (!feeder.isBusy()) {
                    if (getRemainingMatchSeconds() < PARK_TIME_THRESHOLD) {
                        setState(State.DRIVE_TO_LINE_UP_PARK);
                    } else {
                        setState(State.DRIVE_TO_FLOWER_TWO_FROM_FOURTH);
                    }
                }
                break;

            case SHOOT_FOURTH:
                // Shot volley from Flower 2 at Shoot Tip 3 -> park!
                if (!feeder.isBusy()) setState(State.DRIVE_TO_LINE_UP_PARK);
                break;

            case SHOOT_THIRD_FINAL:
                // Shot volley from Flower 2 at Shoot Tip 2 -> park!
                if (!feeder.isBusy()) setState(State.DRIVE_TO_LINE_UP_PARK_FROM_THIRD);
                break;

            // ----- intake: run the intake for a set time -----
            case INTAKE_FLOWER_TWO:
                if (stateTimer.seconds() >= INTAKE_SECONDS) {
                    intakeMotor.setPower(0);
                    if (getRemainingMatchSeconds() < PARK_TIME_THRESHOLD) {
                        setState(State.DRIVE_TO_LINE_UP_PARK);
                    } else {
                        setState(State.DRIVE_TO_FOURTH_SHOT);
                    }
                }
                break;

            case DONE:
                // Nothing left to do. The follower holds the final spot.
                break;
        }

        telemetry.addData("Alliance", ALLIANCE.toUpperCase());
        telemetry.addData("Time Left", "%.1f s", getRemainingMatchSeconds());
        telemetry.addData("State", state);
        telemetry.addData("X", "%.1f", follower.pose().x());
        telemetry.addData("Y", "%.1f", follower.pose().y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Following", follower.isBusy());
        telemetry.addData("Hive Cluster", nectar.getClusterName() == null ? "NONE (TIPPED)" : nectar.getClusterName());
        telemetry.addData("Nectar ready", nectar.readyToShoot());
        telemetry.addData("Pollen ready", pollen.readyToShoot());
        telemetry.addData("Nectar aim error (deg)", "%.1f", nectar.getAimErrorDeg());
        telemetry.update();
    }

    @Override
    public void stop() {
        nectar.stop();
        pollen.stop();
        feeder.stop();
        intakeMotor.setPower(0);
    }

    /** Seconds remaining in the 30-second autonomous period. */
    private double getRemainingMatchSeconds() {
        return Math.max(0, MATCH_DURATION_SECONDS - matchTimer.seconds());
    }

    /** True when both turrets are at speed and aimed, so it's safe to feed a ball. */
    private boolean turretsReady() {
        return nectar.readyToShoot() && pollen.readyToShoot();
    }

    /** Turn both turret flywheels on or off. */
    private void setFlywheels(boolean on) {
        nectar.setFlywheelOn(on);
        pollen.setFlywheelOn(on);
    }

    /** Switch to a new state: remember it, restart the timer, and start that state's job. */
    private void setState(State next) {
        state = next;
        stateTimer.reset();

        switch (state) {
            case DRIVE_TO_FACE_GARDEN:     follower.follow(toFaceGarden);  break;
            case DRIVE_TO_INTAKE_GARDEN:
                intakeMotor.setPower(INTAKE_POWER);          // spin intake while driving into garden
                follower.follow(toIntakeGarden);
                break;
            case INTAKE_GARDEN:
                intakeMotor.setPower(INTAKE_POWER);          // keep running intake to collect
                break;
            case DRIVE_TO_SECOND_SHOT:
                intakeMotor.setPower(0);
                follower.follow(toSecondShot);
                break;
            case DRIVE_TO_FLOWER_ONE:
                intakeMotor.setPower(0);
                setFlywheels(false);
                follower.follow(toFlowerOne);
                break;
            case INTAKE_FLOWER_ONE:
                setFlywheels(false);
                intakeMotor.setPower(INTAKE_POWER);          // run intake at flower one
                break;
            case DRIVE_TO_THIRD_SHOT:
                intakeMotor.setPower(0);
                follower.follow(toThirdShot);
                break;
            case DRIVE_TO_THIRD_SHOT_DIRECT:
                intakeMotor.setPower(0);
                setFlywheels(false);
                follower.follow(toThirdShotDirect);
                break;
            case DRIVE_TO_FLOWER_TWO:
                intakeMotor.setPower(0);
                setFlywheels(false);
                follower.follow(toFlowerTwo);
                break;
            case DRIVE_TO_FLOWER_TWO_FROM_FOURTH:
                intakeMotor.setPower(0);
                setFlywheels(false);
                follower.follow(toFlowerTwoFromFourth);
                break;
            case DRIVE_TO_FOURTH_SHOT:
                intakeMotor.setPower(0);
                follower.follow(toFourthShot);
                break;
            case DRIVE_TO_FOURTH_SHOT_DIRECT:
                intakeMotor.setPower(0);
                setFlywheels(false);
                follower.follow(toFourthShotDirect);
                break;
            case DRIVE_TO_LINE_UP_PARK:
                intakeMotor.setPower(0);
                setFlywheels(false);
                follower.follow(toLineUpPark);
                break;
            case DRIVE_TO_LINE_UP_PARK_FROM_THIRD:
                intakeMotor.setPower(0);
                setFlywheels(false);
                follower.follow(toLineUpParkFromThird);
                break;
            case DRIVE_TO_PARK:
                intakeMotor.setPower(0);
                follower.follow(toPark);
                break;

            case CHECK_THIRD_HIVE:
            case CHECK_THIRD_HIVE_FINAL:
            case CHECK_FOURTH_HIVE:
            case CHECK_FOURTH_HIVE_ALT:
            case AIM_SECOND:
            case AIM_THIRD:
            case AIM_THIRD_FINAL:
            case AIM_FOURTH:
            case AIM_FOURTH_ALT:
                setFlywheels(true);                          // spin up; the turrets aim themselves every loop
                break;

            case SHOOT_SECOND:
            case SHOOT_THIRD:
            case SHOOT_THIRD_FINAL:
            case SHOOT_FOURTH:
            case SHOOT_FOURTH_ALT:
                feeder.fireShots(SHOTS_PER_VOLLEY);          // TurretAutoLogic feeds the balls
                break;

            case INTAKE_FLOWER_TWO:
                setFlywheels(false);
                intakeMotor.setPower(INTAKE_POWER);          // run the intake
                break;

            case DONE:
                setFlywheels(false);
                feeder.stop();
                break;
        }
    }

    /**
     * Build all the paths once, up front. These match the nine segments in BLUE_2.5Tip_FarPark.pp.json, in order.
     */
    private void buildPaths() {
        // 1. Turn_To_Garden: straight. Heading turns 90 -> 0.
        toFaceGarden = line(startPose, faceGarden).linear(startPose, faceGarden);

        // 2. Intake_Garden: straight. Heading follows the direction of travel (tangent -> 0).
        toIntakeGarden = line(faceGarden, intakeGarden).tangent();

        // 3. Pass_through_middle_Shoot: CURVE. Heading maintains 0 across the middle so it seamlessly enters Flower One.
        toSecondShot = bezier(intakeGarden, otherSideSecondVolley, 78.755, 7.237, 75.542, 75.892)
                .linear(intakeGarden, otherSideSecondVolley);

        // 4. Intake_Flower: CURVE with one control point. Heading turns 0 -> 90.
        toFlowerOne = bezier(otherSideSecondVolley, intakeFlowerOne, 93.662, 114.531)
                .linear(otherSideSecondVolley, intakeFlowerOne);

        // 5. Shoot_Tip_2: straight. Heading turns 90 -> 270.
        toThirdShot = line(intakeFlowerOne, otherSideShootThirdVolley).linear(intakeFlowerOne, otherSideShootThirdVolley);

        // Direct cross-field path from Shoot Tip 2 to Shoot Tip 3 (if Hive 2 is tipped)
        toFourthShotDirect = bezier(otherSideShootThirdVolley, shootFourthVolley, 83.5, 66.0)
                .linear(otherSideShootThirdVolley, shootFourthVolley);

        // Direct return path from Shoot Tip 3 back to Shoot Tip 2 (if Hive 3 is tipped)
        toThirdShotDirect = bezier(shootFourthVolley, otherSideShootThirdVolley, 83.5, 66.0)
                .linear(shootFourthVolley, otherSideShootThirdVolley);

        // 6. Intake_Other_Flower: CURVE with two control points. Heading turns 270 -> 0.
        toFlowerTwo = bezier(otherSideShootThirdVolley, intakeFlowerTwo, 116.710, 96.850, 99.856, 43.769)
                .linear(otherSideShootThirdVolley, intakeFlowerTwo);

        // Path to Flower 2 if Flower 1's volley was shot at Shoot Tip 3
        toFlowerTwoFromFourth = line(shootFourthVolley, intakeFlowerTwo).linear(shootFourthVolley, intakeFlowerTwo);

        // 7. Shoot+Tip_3: straight. Heading turns 0 -> 90.
        toFourthShot = line(intakeFlowerTwo, shootFourthVolley).linear(intakeFlowerTwo, shootFourthVolley);

        // Direct park from Shoot Tip 2 (other side)
        toLineUpParkFromThird = line(otherSideShootThirdVolley, goToPark).linear(otherSideShootThirdVolley, goToPark);

        // 8. Go_To_Park: straight. Heading turns 90 -> 0.
        toLineUpPark = line(shootFourthVolley, goToPark).linear(shootFourthVolley, goToPark);

        // 9. Park: straight. Heading follows the direction of travel (tangent -> 0).
        toPark = line(goToPark, parkFarSide).tangent();
    }

    /**
     * A curved (Bezier) path from one point to another, bent by the control points.
     * controlXY is a flat list of x, y pairs, e.g. bezier(from, to, c1x, c1y, c2x, c2y).
     */
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
