package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TurretCode.NectarTurret;
import org.firstinspires.ftc.teamcode.TurretCode.PollenTurret;
import org.firstinspires.ftc.teamcode.TurretCode.TagCamera;
import org.firstinspires.ftc.teamcode.TurretCode.TurretAutoLogic;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

/*
 * AutonomousPractice - a STATE MACHINE autonomous.
 *
 * How a state machine works:
 *   - The robot is always in exactly ONE state (the "state" variable).
 *   - Every loop() we look at the current state and ask "am I done with this step?"
 *   - When the step is done we call setState(next). setState() starts the job of the new state.
 *
 * The kinds of state:
 *   DRIVE_*      - setState() tells the follower to follow a path. loop() waits until
 *                  follower.isBusy() is false (the path is finished), then moves on.
 *   AIM_*        - setState() turns the turret flywheels on. loop() waits until both turrets say
 *                  they're ready (at speed and aimed), or until AIM_MAX_SECONDS passes.
 *   SHOOT_*      - setState() asks TurretAutoLogic to feed balls. loop() waits until it's done.
 *   INTAKE_*     - setState() lowers the ramp and runs the intake motor. loop() waits, then moves on.
 *   RAISE_RAMP   - setState() stops the intake and raises the ramp. loop() waits for the servo.
 *
 * Every loop() also updates the cameras and both turrets, so they keep aiming while the robot drives.
 *
 * Coordinates are in inches, heading in degrees (PoseFactory.degrees()).
 */
@Autonomous
public class AutonomousPractice extends OpMode {

    // ---------- TUNE: timing and hardware values (measure these on the robot) ----------
    private static final double AIM_MAX_SECONDS = 3.0;   // stop waiting for the turrets after this long, then shoot anyway
    private static final int SHOTS_PER_VOLLEY = 1;       // balls fed per SHOOT state. Match this to the real volley.
    private static final double RAMP_SECONDS = 0.5;      // time for the ramp servo to move
    private static final double INTAKE_SECONDS = 2.0;    // how long the intake runs at the flowers
    private static final double INTAKE_POWER = 1.0;      // intake motor power
    private static final double RAMP_DOWN_POSITION = 0.0; // ramp lowered to reach the flowers. Swap with RAMP_UP if it moves the wrong way.
    private static final double RAMP_UP_POSITION = 1.0;   // ramp raised out of the way

    // ---------- every position the robot visits (exact points from RED_2.5Tip_FarPark.pp) ----------
    // Headings only matter where a path uses a linear heading. The others follow the path direction.
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(55.69, 15.431, 90);
    private final Pose faceGarden = poseFactory.of(30.846, 9.330, 180);
    private final Pose intakeGarden = poseFactory.of(7.896, 8.639, 180);            // tangent heading: not used
    private final Pose otherSideSecondVolley = poseFactory.of(57.280, 118.126, 0);   // tangent heading: not used
    private final Pose intakeFlowerOne = poseFactory.of(47.147, 132.632, 90);
    private final Pose otherSideShootThirdVolley = poseFactory.of(46.982, 108.624, 270);
    private final Pose intakeFlowerTwo = poseFactory.of(9.547, 47.539, 180);
    private final Pose shootFourthVolley = poseFactory.of(42.769, 24.418, 90);
    private final Pose goToPark = poseFactory.of(22.326, 111.687, 180);
    private final Pose parkFarSide = poseFactory.of(8.028, 111.595, 0);             // tangent heading: not used

    // ---------- the paths between those positions (built once in init) ----------
    private Path toFaceGarden, toIntakeGarden, toSecondShot, toFlowerOne, toThirdShot,
            toFlowerTwo, toFourthShot, toLineUpPark, toPark;

    // ---------- hardware ----------
    private TagCamera stationaryCam, turretCam;             // stationary = distance, turret = aim
    private final NectarTurret nectar = new NectarTurret();
    private final PollenTurret pollen = new PollenTurret();
    private final TurretAutoLogic feeder = new TurretAutoLogic();
    private DcMotor intakeMotor;                             // goBILDA motor on the intake
    private Servo rampServo;                                 // lowers/raises the ramp for flowers

    // ---------- the states ----------
    // Read top to bottom: this is the order the robot does things in.
    private enum State {
        DRIVE_TO_FACE_GARDEN,
        DRIVE_TO_INTAKE_GARDEN,
        DRIVE_TO_SECOND_SHOT,
        AIM_SECOND,
        SHOOT_SECOND,
        DRIVE_TO_FLOWER_ONE,
        DRIVE_TO_THIRD_SHOT,
        AIM_THIRD,
        SHOOT_THIRD,
        DRIVE_TO_FLOWER_TWO,
        INTAKE_FLOWER_TWO,
        RAISE_RAMP,
        DRIVE_TO_FOURTH_SHOT,
        AIM_FOURTH,
        SHOOT_FOURTH,
        DRIVE_TO_LINE_UP_PARK,
        DRIVE_TO_PARK,
        DONE
    }

    private Follower follower;
    private State state = State.DRIVE_TO_FACE_GARDEN;
    private final ElapsedTime stateTimer = new ElapsedTime();

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
        rampServo = hardwareMap.get(Servo.class, "rampServo");
        rampServo.setPosition(RAMP_UP_POSITION);
    }

    @Override
    public void start() {
        setState(State.DRIVE_TO_FACE_GARDEN); // start the first step
    }

    @Override
    public void loop() {
        // Everything that must run every loop: the follower, the cameras, the turrets, the feeder.
        follower.update();
        stationaryCam.update();
        turretCam.update();
        nectar.update();
        pollen.update();
        feeder.update(turretsReady());

        switch (state) {
            // ----- driving states: wait for the path to finish -----
            case DRIVE_TO_FACE_GARDEN:
                if (!follower.isBusy()) setState(State.DRIVE_TO_INTAKE_GARDEN);
                break;
            case DRIVE_TO_INTAKE_GARDEN:
                if (!follower.isBusy()) setState(State.DRIVE_TO_SECOND_SHOT);
                break;
            case DRIVE_TO_SECOND_SHOT:
                if (!follower.isBusy()) setState(State.AIM_SECOND);
                break;
            case DRIVE_TO_FLOWER_ONE:
                if (!follower.isBusy()) setState(State.DRIVE_TO_THIRD_SHOT);
                break;
            case DRIVE_TO_THIRD_SHOT:
                if (!follower.isBusy()) setState(State.AIM_THIRD);
                break;
            case DRIVE_TO_FLOWER_TWO:
                if (!follower.isBusy()) setState(State.INTAKE_FLOWER_TWO);
                break;
            case DRIVE_TO_FOURTH_SHOT:
                if (!follower.isBusy()) setState(State.AIM_FOURTH);
                break;
            case DRIVE_TO_LINE_UP_PARK:
                if (!follower.isBusy()) setState(State.DRIVE_TO_PARK);
                break;
            case DRIVE_TO_PARK:
                if (!follower.isBusy()) setState(State.DONE);
                break;

            // ----- aim: wait for both turrets to be ready (or give up after a while) -----
            case AIM_SECOND:
                if (turretsReady() || stateTimer.seconds() >= AIM_MAX_SECONDS) setState(State.SHOOT_SECOND);
                break;
            case AIM_THIRD:
                if (turretsReady() || stateTimer.seconds() >= AIM_MAX_SECONDS) setState(State.SHOOT_THIRD);
                break;
            case AIM_FOURTH:
                if (turretsReady() || stateTimer.seconds() >= AIM_MAX_SECONDS) setState(State.SHOOT_FOURTH);
                break;

            // ----- shoot: wait for TurretAutoLogic to finish feeding -----
            case SHOOT_SECOND:
                if (!feeder.isBusy()) setState(State.DRIVE_TO_FLOWER_ONE);
                break;
            case SHOOT_THIRD:
                if (!feeder.isBusy()) setState(State.DRIVE_TO_FLOWER_TWO);
                break;
            case SHOOT_FOURTH:
                if (!feeder.isBusy()) setState(State.DRIVE_TO_LINE_UP_PARK);
                break;

            // ----- intake: run the intake for a set time, then raise the ramp -----
            case INTAKE_FLOWER_TWO:
                if (stateTimer.seconds() >= INTAKE_SECONDS) setState(State.RAISE_RAMP);
                break;
            case RAISE_RAMP:
                if (stateTimer.seconds() >= RAMP_SECONDS) setState(State.DRIVE_TO_FOURTH_SHOT);
                break;

            case DONE:
                // Nothing left to do. The follower holds the final spot.
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("X", "%.1f", follower.pose().x());
        telemetry.addData("Y", "%.1f", follower.pose().y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Following", follower.isBusy());
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
            case DRIVE_TO_INTAKE_GARDEN:   follower.follow(toIntakeGarden); break;
            case DRIVE_TO_SECOND_SHOT:     follower.follow(toSecondShot);   break;
            case DRIVE_TO_FLOWER_ONE:      follower.follow(toFlowerOne);    break;
            case DRIVE_TO_THIRD_SHOT:      follower.follow(toThirdShot);    break;
            case DRIVE_TO_FLOWER_TWO:      follower.follow(toFlowerTwo);    break;
            case DRIVE_TO_FOURTH_SHOT:     follower.follow(toFourthShot);   break;
            case DRIVE_TO_LINE_UP_PARK:    follower.follow(toLineUpPark);   break;
            case DRIVE_TO_PARK:            follower.follow(toPark);         break;

            case AIM_SECOND:
            case AIM_THIRD:
            case AIM_FOURTH:
                setFlywheels(true);                          // spin up; the turrets aim themselves every loop
                break;

            case SHOOT_SECOND:
            case SHOOT_THIRD:
            case SHOOT_FOURTH:
                feeder.fireShots(SHOTS_PER_VOLLEY);          // TurretAutoLogic feeds the balls
                break;

            case INTAKE_FLOWER_TWO:
                rampServo.setPosition(RAMP_DOWN_POSITION);   // lower the ramp onto the flowers
                intakeMotor.setPower(INTAKE_POWER);          // and run the intake
                break;

            case RAISE_RAMP:
                intakeMotor.setPower(0);                     // stop the intake
                rampServo.setPosition(RAMP_UP_POSITION);     // raise the ramp out of the way
                break;

            case DONE:
                setFlywheels(false);
                feeder.stop();
                break;
        }
    }

    /**
     * Build all the paths once, up front. These match the nine segments in RED_2.5Tip_FarPark.pp, in order.
     *
     * About "reverse: true" on the linear segments (4, 5, 7, 8): the stored heading values in the .pp file
     * are the headings the robot should actually face. Each one matches the end heading of the segment before
     * it, so we use them as they are. Flipping them by 180 would turn the robot around at every junction.
     * Pedro's swerve drivetrain can drive in any direction while facing any heading, so no flip is needed.
     */
    private void buildPaths() {
        // 1. Turn_To_Garden: straight. Heading turns 90 -> 180.
        toFaceGarden = line(startPose, faceGarden).linear(startPose, faceGarden);

        // 2. Intake_Garden: straight. Heading follows the direction of travel.
        toIntakeGarden = line(faceGarden, intakeGarden).tangent();

        // 3. Pass_through_middle_Shoot: CURVE. Heading follows the curve backwards (reverse tangent).
        toSecondShot = bezier(intakeGarden, otherSideSecondVolley, 62.745, 7.237, 65.958, 75.892).reverseTangent();

        // 4. Intake_Flower: CURVE with one control point. Heading turns 180 -> 90.
        toFlowerOne = bezier(otherSideSecondVolley, intakeFlowerOne, 47.838, 114.531)
                .linear(poseFactory.of(57.280, 118.126, 180), intakeFlowerOne);

        // 5. Shoot_Tip_2: straight. Heading turns 90 -> 270.
        toThirdShot = line(intakeFlowerOne, otherSideShootThirdVolley).linear(intakeFlowerOne, otherSideShootThirdVolley);

        // 6. Intake_Other_Flower: CURVE with two control points. Heading turns 270 -> 180.
        toFlowerTwo = bezier(otherSideShootThirdVolley, intakeFlowerTwo, 24.790, 96.850, 41.644, 43.769)
                .linear(otherSideShootThirdVolley, intakeFlowerTwo);

        // 7. Shoot+Tip_3: straight. Heading turns 180 -> 90.
        toFourthShot = line(intakeFlowerTwo, shootFourthVolley).linear(intakeFlowerTwo, shootFourthVolley);

        // 8. Go_To_Park: straight. Heading turns 90 -> 180.
        toLineUpPark = line(shootFourthVolley, goToPark).linear(shootFourthVolley, goToPark);

        // 9. Park: straight. Heading follows the direction of travel.
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
