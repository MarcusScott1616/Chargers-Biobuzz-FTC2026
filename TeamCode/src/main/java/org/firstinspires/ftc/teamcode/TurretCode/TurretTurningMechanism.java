package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Use the FTC SDK's AprilTag class (it has ftcPose; the org.openftc.apriltag one does not).
// In SDK 12, a single tag's id is on AprilTagSingleDetection, not the base AprilTagDetection.
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

public class TurretTurningMechanism {
    // AprilTag the turret aims at.
    private static final int TARGET_TAG_ID = 20;

    private DcMotorEx turret;
    // Motor that turns the turret.
    //IDK what motor we are using, double check with Design team.

    private Limelight3A limelight;

    private double kP= 0.0001;

    private double kD= 0.0000;

    private double goalX=0;
//Goal target, makes it easy to offset
// for example, if we want to shoot towards the left of the april tag, adjust the goalX
    private double lastError=0;

    private double angleTolerance= 0.2;
// adjusts how "off" the turret can be, further away=less tolerance
    private double MAX_POWER = 0.8;

    private double power = 0;

    private final ElapsedTime timer= new ElapsedTime();

    public void init(HardwareMap hwMap){
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void setkP(double newKP){
        kP=newKP;
    };
    public double getkP(){
        return kP;
    }

    public void setkD(double newKD){
        kD=newKD;
    };
    public double getkD(){
        return kD;
    }

    public void resetTimer(){
        timer.reset();
    }

    //adjust the "Apriltagdetection" here. IDK, tutorial said something about limelight detection
    // "if tag is null/true", "
    public void update(AprilTagSingleDetection curID){
        double deltaTime = timer.seconds();
        timer.reset();

        //adjust ID number here.
        // adjust for limelight Identification here.

        // No tag, the wrong tag, or no pose info: stop the turret and wait.
        if (curID == null || curID.id != TARGET_TAG_ID || curID.ftcPose == null){
            turret.setPower(0);
            lastError = 0;
            return;
        }

        //------------- Starting PD controlling -----------------


        double error = goalX - curID.ftcPose.bearing;
                // "With A Limelight this might just be your TX"
        double pTerm = error * kP;

        double dTerm = 0;

        if (deltaTime > 0){
            dTerm=((error- lastError) /deltaTime) *kD;

        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else{
            power= Range.clip(pTerm+dTerm, -MAX_POWER, MAX_POWER);

            // IF we dont use a slip ring, we will need to
            // Code a way to stop the turret when it gets to about 200-300 degrees, so
            // it doesnt pull it's own wires out.

        }
        
        turret.setPower(power);
        lastError = error;
        

    }
}
