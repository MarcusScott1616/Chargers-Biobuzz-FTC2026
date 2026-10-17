package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;

import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;

import com.pedropathing.utils.Timer;

// need to finish tuning, Odometry, and ensure constants are correct.
@Autonomous
public class AutonomousPractice extends OpMode {

    private Timer pathTimer, opModeTimer;
    private Follower follower;
    private final PoseFactory p= PoseFactory.degrees();
    private final Pose startPose = p.of(24,24,0);
    private final Pose park =p.of(48,48,90);

    public enum pathState {
        //start at:
        //end at:

    }

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        //wherever the robot is placed at the

}

    @Override
    public void start() {

    }
    @Override
    public void loop(){

    }

}
