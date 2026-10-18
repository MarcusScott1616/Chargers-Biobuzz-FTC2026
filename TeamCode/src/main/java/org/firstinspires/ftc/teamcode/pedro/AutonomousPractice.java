package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;

import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

// need to finish tuning, Odometry, and ensure constants are correct before fully testing this auto.
// THIS AUTO DOES: 2+1/2 tips, parks far side
// Takes up most of the space, needs to ignore the other robot.
@Autonomous
public class AutonomousPractice extends OpMode {


    private Follower follower;
    private final PoseFactory p= PoseFactory.degrees();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(55.69,15.431, 90);
    private final Pose faceGarden = poseFactory.of(30.8,9.3, 180);
    private final Pose intakeGarden = poseFactory.of(7.98,8.6, 180);
    private final Pose otherSideSecondVolley = poseFactory.of(57.3,118.1, -79);
    private final Pose intakeFlowerOne = poseFactory.of(47.1,132.6, 90);
    private final Pose otherSideShootThirdVolley = poseFactory.of(47.0,108.6, 270);
    private final Pose intakeFlowerTwo = poseFactory.of(9.5,47.0, 180);
    private final Pose shootFourthVolley = poseFactory.of(42.8,24.4, 90);
    private final Pose goToPark = poseFactory.of(22.3,111.7, 180);
    private final Pose parkFarSide = poseFactory.of(8.0,111.6, 180);



private Path startToFaceGarden(){
    return line(startPose, faceGarden).linear(startPose, faceGarden);
}
private Path facingGardenToIntakingGarden(){
    return line(faceGarden, intakeGarden).linear(faceGarden, intakeGarden);
}
private Path intakingGardenToOtherSideShot(){
    return line(intakeGarden, otherSideSecondVolley).linear(intakeGarden, otherSideSecondVolley);
}
private Path otherSideShotTointakeFlowerOne(){
    return line(otherSideSecondVolley, intakeFlowerOne).linear(otherSideSecondVolley,intakeFlowerOne);
}
private Path intakeFlowerOneToOtherSideShootThirdVolley(){
    return line(intakeFlowerOne, otherSideShootThirdVolley).linear(intakeFlowerOne, otherSideShootThirdVolley);
}
private Path otherSideShootThirdVolleyToIntakeFlowerTwo(){
    return line(otherSideShootThirdVolley, intakeFlowerTwo).linear(otherSideShootThirdVolley, intakeFlowerTwo);
}
private Path intakeFlowerTwoToShootFourthVolley(){
    return line(intakeFlowerTwo, shootFourthVolley).linear(intakeFlowerTwo, shootFourthVolley);
}
private Path shootFourthVolleyToLineUpPark(){
    return line(shootFourthVolley, goToPark).linear(shootFourthVolley, goToPark);
}
private Path lineUpParkToParkFarSide(){
    return line(goToPark, parkFarSide).linear(goToPark, parkFarSide);
}

private Command doubleTipFarPark(){
    return sequential(
            follow(follower, startToFaceGarden()),
            //either we shoot here, or at the very start before we turn and move
            // start running intake here
            follow(follower, facingGardenToIntakingGarden()),
            //end running intake here.
            follow(follower, intakingGardenToOtherSideShot()),
            //need to shoot here.
            // drop down intake ramp for flowers here
            // start running intake here
            follow(follower, otherSideShotTointakeFlowerOne()),
            //end running intake here? IDK test it out, how long does it take to intake a flower?
            follow(follower, intakeFlowerOneToOtherSideShootThirdVolley()),
            //shoot here
            //make sure intake ramp is down, and likely running
            follow(follower, otherSideShootThirdVolleyToIntakeFlowerTwo()),
            //keep running intake for set amount of seconds until flower is empty
            follow(follower, intakeFlowerTwoToShootFourthVolley()),
            // shoot here
            follow(follower, shootFourthVolleyToLineUpPark()),
            // make sure intake ramp is up,
            follow(follower, lineUpParkToParkFarSide())
            //check with other team where they park. Far side is the side closer to the edge of the hive side that starts down.
    );
}
    @Override
    public void init() {

        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);  //wherever the robot is placed at the start of a match
        follower.update();
}

    @Override
    public void start() {
    schedule(doubleTipFarPark());


    }
    @Override
    public void loop(){

        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();

    }

}
