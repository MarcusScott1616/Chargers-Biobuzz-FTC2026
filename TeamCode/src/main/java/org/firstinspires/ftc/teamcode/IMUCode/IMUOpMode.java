package org.firstinspires.ftc.teamcode.IMUCode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class IMUOpMode extends OpMode {

    IMUPractice RobotIMU = new IMUPractice();

    @Override
    public void init(){
    RobotIMU.init(hardwareMap);

    }
    @Override
    public void loop(){
    telemetry.addData("heading", RobotIMU.getHeading());
    }

}
