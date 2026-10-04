package org.firstinspires.ftc.teamcode.LimelightCode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

public class LimelightTest extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch( 8);
        // alter pipeline to whatever we wanna.....
        IMU imu = hardwareMap.get(IMU.class, "imu");


    }
    @Override
    public void start(){
        limelight.start();
        // if theres a delay for Limelight starting, we can put it back in init.
        // apparently the Limelight draws a lot of power. It is suggested to start the limelight at start, not init.

    }

    @Override
    public void loop(){

    }
}
