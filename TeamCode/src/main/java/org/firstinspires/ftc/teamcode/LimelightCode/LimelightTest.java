package org.firstinspires.ftc.teamcode.LimelightCode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class LimelightTest extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch( 8);
        // alter pipeline to whatever we wanna.....
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot( RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    }
    @Override
    public void start(){
        limelight.start();
        // if theres a delay for Limelight starting, we can put it back in init.
        // apparently the Limelight draws a lot of power. It is suggested to start the limelight at start, not init.

    }

    @Override
    public void loop(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if(llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            //Trying out MT2, unsure how it will turn out.
            telemetry.addData("TX", llResult.getTx());
            telemetry.addData("TY", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
        }
    }
}
