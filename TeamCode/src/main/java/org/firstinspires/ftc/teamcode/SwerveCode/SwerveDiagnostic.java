package org.firstinspires.ftc.teamcode.SwerveCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class SwerveDiagnostic extends OpMode {

    private SwerveDrive swerve = new SwerveDrive();

    @Override
    public void init(){
        swerve.init(hardwareMap);

        telemetry.addLine("Diagnostic Mode!");
        telemetry.addLine("D-Pad UP:        Test RED motor");
        telemetry.addLine("D-Pad Right:     Test BLUE motor");
        telemetry.addLine("D-Pad Left:      Test GREEN motor");
        telemetry.addLine("D-Pad Down:      Test YELLOW motor");
        telemetry.addLine("Bumper Left:     Center Servos (90 deg)");
        telemetry.addLine("Bumper Right:    Zero Servos (0 deg)");
        telemetry.update();
    }
    @Override
    public void loop(){
        //test the individual drive motor directions
        double redPow     =gamepad1.dpad_up      ? 0.3 : 0.0;
        double bluePow    =gamepad1.dpad_right   ? 0.3 : 0.0;
        double greenPow   =gamepad1.dpad_left    ? 0.3 : 0.0;
        double yellowPow  =gamepad1.dpad_down    ? 0.3 : 0.0;

        double servoPos = 0.5;



        if(gamepad1.left_bumper){
         servoPos = 0.5; //center / 90 degrees
        } else if (gamepad1.right_bumper){
            servoPos=0.0;
        }

        for (ServoImplEx servo : swerve.servoList){
            servo.setPosition(servoPos);
        }
        //directly command individual motorrs to spin for testing
        swerve.move(
                (float) (bluePow + redPow),   //strafeX power feed
                (float) (greenPow + yellowPow), //strafe Y power feed
                0.0f                              //no rotation
        );
        telemetry.addData("Red Power", redPow);
        telemetry.addData("Blue Power", bluePow);
        telemetry.addData("Green Power", greenPow);
        telemetry.addData("Yellow Power", yellowPow);
        telemetry.update();

    }



}

