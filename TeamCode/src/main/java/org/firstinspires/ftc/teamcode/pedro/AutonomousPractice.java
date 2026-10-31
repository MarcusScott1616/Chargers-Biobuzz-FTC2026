package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.api.Paths.line;

/*
 * AutonomousPractice - a STATE MACHINE autonomous.
 *
 * How a state machine works:
 *   - The robot is always in exactly ONE state (the "state" variable).
 *   - Every loop() we look at the current state and ask "am I done with this step?"
 *   - When the step is done we call setState(next). setState() starts the job of the new state.
 *
 * There are two kinds of state:
 *   DRIVE_*  - setState() tells the follower to follow a path. loop() waits until
 *              follower.isBusy() is false (the path is finished), then moves on.
 *   SHOOT_*, INTAKE_* - setState() resets stateTimer. loop() waits for the timer, then moves on.
 *              These are placeholders until the shooter/intake code is wired in.
 *
 * The steps and their order match the old doubleTipFarPark() command list.
 *
 * Coordinates are in inches, heading in degrees (PoseFactory.degrees()).
 */
@Autonomous
public class AutonomousPractice extends OpMode {

    // TUNE: how long to wait in each SHOOT / INTAKE state (seconds). 0 = no wait.
    private static final double SHOOT_SECONDS = 1.0;
    private static final double INTAKE_SECONDS = 2.0;

    // ---------- every position the robot visits ----------
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(55.69, 15.431, 90);
    private final Pose faceGarden = poseFactory.of(30.8, 9.3, 180);
    private final Pose intakeGarden = poseFactory.of(7.98, 8.6, 180);
    private final Pose otherSideSecondVolley = poseFactory.of(57.3, 118.1, -79);
    private final Pose intakeFlowerOne = poseFactory.of(47.1, 132.6, 90);
    private final Pose otherSideShootThirdVolley = poseFactory.of(47.0, 108.6, 270);
    private final Pose intakeFlowerTwo = poseFactory.of(9.5, 47.0, 180);
    private final Pose shootFourthVolley = poseFactory.of(42.8, 24.4, 90);
    private final Pose goToPark = poseFactory.of(22.3, 111.7, 180);
    private final Pose parkFarSide = poseFactory.of(8.0, 111.6, 180);

    // ---------- the paths between those positions (built once in init) ----------
    private Path toFaceGarden, toIntakeGarden, toSecondShot, toFlowerOne, toThirdShot,
            toFlowerTwo, toFourthShot, toLineUpPark, toPark;

    // ---------- the states ----------
    // Read top to bottom: this is the order the robot does things in.
    private enum State {
        DRIVE_TO_FACE_GARDEN,
        DRIVE_TO_INTAKE_GARDEN,
        DRIVE_TO_SECOND_SHOT,
        SHOOT_SECOND,
        DRIVE_TO_FLOWER_ONE,
        DRIVE_TO_THIRD_SHOT,
        SHOOT_THIRD,
        DRIVE_TO_FLOWER_TWO,
        INTAKE_FLOWER_TWO,
        DRIVE_TO_FOURTH_SHOT,
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
    }

    @Override
    public void start() {
        setState(State.DRIVE_TO_FACE_GARDEN); // start the first step
    }

    @Override
    public void loop() {
        // The follower must update every loop, or the robot stops driving.
        follower.update();

        switch (state) {
            // ----- driving states: wait for the path to finish -----
            case DRIVE_TO_FACE_GARDEN:
                if (!follower.isBusy()) setState(State.DRIVE_TO_INTAKE_GARDEN);
                break;
            case DRIVE_TO_INTAKE_GARDEN:
                if (!follower.isBusy()) setState(State.DRIVE_TO_SECOND_SHOT);
                break;
            case DRIVE_TO_SECOND_SHOT:
                if (!follower.isBusy()) setState(State.SHOOT_SECOND);
                break;
            case DRIVE_TO_FLOWER_ONE:
                if (!follower.isBusy()) setState(State.DRIVE_TO_THIRD_SHOT);
                break;
            case DRIVE_TO_THIRD_SHOT:
                if (!follower.isBusy()) setState(State.SHOOT_THIRD);
                break;
            case DRIVE_TO_FLOWER_TWO:
                if (!follower.isBusy()) setState(State.INTAKE_FLOWER_TWO);
                break;
            case DRIVE_TO_FOURTH_SHOT:
                if (!follower.isBusy()) setState(State.SHOOT_FOURTH);
                break;
            case DRIVE_TO_LINE_UP_PARK:
                if (!follower.isBusy()) setState(State.DRIVE_TO_PARK);
                break;
            case DRIVE_TO_PARK:
                if (!follower.isBusy()) setState(State.DONE);
                break;

            // ----- timed states: wait for the timer -----
            case SHOOT_SECOND:
                if (stateTimer.seconds() >= SHOOT_SECONDS) setState(State.DRIVE_TO_FLOWER_ONE);
                break;
            case SHOOT_THIRD:
                if (stateTimer.seconds() >= SHOOT_SECONDS) setState(State.DRIVE_TO_FLOWER_TWO);
                break;
            case INTAKE_FLOWER_TWO:
                if (stateTimer.seconds() >= INTAKE_SECONDS) setState(State.DRIVE_TO_FOURTH_SHOT);
                break;
            case SHOOT_FOURTH:
                if (stateTimer.seconds() >= SHOOT_SECONDS) setState(State.DRIVE_TO_LINE_UP_PARK);
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
        telemetry.update();
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

            // Timed states: nothing to start yet, the timer does the work.
            case SHOOT_SECOND:
            case SHOOT_THIRD:
            case SHOOT_FOURTH:
            case INTAKE_FLOWER_TWO:
            case DONE:
                break;
        }
    }

    /** Build all the paths once, up front. */
    private void buildPaths() {
        toFaceGarden   = straight(startPose, faceGarden);
        toIntakeGarden = straight(faceGarden, intakeGarden);
        toSecondShot   = straight(intakeGarden, otherSideSecondVolley);
        toFlowerOne    = straight(otherSideSecondVolley, intakeFlowerOne);
        toThirdShot    = straight(intakeFlowerOne, otherSideShootThirdVolley);
        toFlowerTwo    = straight(otherSideShootThirdVolley, intakeFlowerTwo);
        toFourthShot   = straight(intakeFlowerTwo, shootFourthVolley);
        toLineUpPark   = straight(shootFourthVolley, goToPark);
        toPark         = straight(goToPark, parkFarSide);
    }

    /**
     * A straight path between two poses. The robot's heading turns smoothly
     * from the first pose's heading to the second pose's heading.
     */
    private static Path straight(Pose from, Pose to) {
        return line(from, to).linear(from, to);
    }
}
