package org.firstinspires.ftc.teamcode.SwerveCode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Electric Mayhem (Nichols School)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/


/**
 *
 */
public class SwerveDrive {

    /* 
     * RED--------BLU
     *  |          |
     *  |          |
     *  |          |
     *  |          |
     * GRN--------YLW
     */


    private ServoImplEx redServo, blueServo, greenServo, yellowServo;
    private final ServoImplEx[] servoList = new ServoImplEx[]{redServo, blueServo, greenServo, yellowServo};

    private IMU.Parameters imuParams;
    private IMU imu;
    private DcMotor redMotor, blueMotor, greenMotor, yellowMotor;
    private final DcMotor[] motorList = new DcMotor[]{redMotor, blueMotor, greenMotor, yellowMotor};

    public int WheelbaseLength = 17;
    public int WheelbaseWidth = 17;

    public void init(HardwareMap hMap) {
        // sample code did not grab hardware map properly.... Replaced:
        // HardwareMap hMap = new HardwareMap(); with the above parameter

        imu = hMap.get(IMU.class, "imu");


        redServo = hMap.get(ServoImplEx.class, "redServo");
        blueServo = hMap.get(ServoImplEx.class, "blueServo");
        yellowServo = hMap.get(ServoImplEx.class, "yellowServo");
        greenServo = hMap.get(ServoImplEx.class, "greenServo");

        redMotor = hMap.get(DcMotor.class, "redMotor");
        blueMotor = hMap.get(DcMotor.class, "blueMotor");
        yellowMotor = hMap.get(DcMotor.class, "yellowMotor");
        greenMotor = hMap.get(DcMotor.class, "greenMotor");

        imuParams = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
           )
        );

        imu.initialize(imuParams);
        imu.resetYaw();

        redServo = hMap.get(ServoImplEx.class, "redServo");
        blueServo = hMap.get(ServoImplEx.class, "blueServo");
        greenServo = hMap.get(ServoImplEx.class, "greenServo");
        yellowServo = hMap.get(ServoImplEx.class, "yellowServo");

        for (ServoImplEx servo : servoList) {
            servo.setPwmEnable();
            servo.setPwmRange(new PwmControl.PwmRange(505, 2495));
            servo.setDirection(Servo.Direction.FORWARD);
        }

        for (DcMotor motor : motorList){
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        }
    }
	
    public void resetYaw() {
        imu.resetYaw();
    }

    public void moveWithEncoder(double degreeAngle, double distance){
        //placeholder for later
        double meterToEncoder = 500;

        int position = ((int)degreeAngle + 180) / 360;
        int encoderTick = (int)distance * (int)meterToEncoder;
        
        //setting powers and positions
        for (DcMotor motor : motorList) {
            motor.setTargetPosition(encoderTick);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }
        for (ServoImplEx servo : servoList) {
           servo.setPosition(position);
        }

    }

    /**
     * Moves the Drivetrain with setPower()
     *
     * @strafeX Power for Left-Right Locomotion
     * @strafeY Power for Forward-Back Locomotion
     * @rotate The extent to which the drivetrain rotates
     */

	public void move(float strafeX, float strafeY, float rotate) {

        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        
        double yaw = robotOrientation.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);

        //Calculations
        // Making every wheel rotate uniformly would turn at unequal angles, not pivoting around center point
		//float theta1 = (float) Math.atan2(strafeX + rotate, strafeY + rotate) - yaw;

        //float theta2 = (float) Math.atan2(strafeX - rotate, strafeY + rotate) - yaw;
        // Making every wheel rotate uniformly would turn at unequal angles, not pivoting around center point


        //float power1 = (float) Math.sqrt(Math.pow((strafeY + rotate) / 2, 2) + Math.pow((strafeX + rotate) / 2, 2));
        //float power2 = (float) Math.sqrt(Math.pow((strafeY + rotate) / 2, 2) + Math.pow((strafeX - rotate) / 2, 2));
        //Sending Powers and Angles to Motors


        // Instead of calculations like that, calculate for each module.
        // Diagram again:
        /*
         * RED--------BLU
         *  |          |
         *  |          |
         *  |          |
         *  |          |
         * GRN--------YLW
         */


        double fieldX= strafeX*cos(-yaw)-strafeY*sin(-yaw);
        double fieldY = strafeX *sin(-yaw)+strafeY*cos(-yaw);

        double  L = WheelbaseLength; // Length of Chassis
        double  W = WheelbaseWidth; //Width of Chassis
        double R = Math.hypot(L,W); // the diagonal of the chassis

        // Component Vectors
        float red = fieldX-rotate* ( L / R);
        float blue = fieldX+rotate* ( L / R);
        float green = fieldY+rotate* ( W / R);
        float yellow = fieldY-rotate* ( W / R);

        double redPower = Math.hypot(blue, yellow);
        double bluePower = Math.hypot(blue, green);
        double greenPower = Math.hypot(red, yellow);
        double yellowPower = Math.hypot(red, green);

        double redTheta     = Math.atan2(blue, yellow);
        double blueTheta    = Math.atan2(blue, green);
        double greenTheta   = Math.atan2(red, yellow);
        double yellowTheta  = Math.atan2(red, green);

        double maxPower = Math.max(1.0, Math.max(Math.max(redPower, bluePower), Math.max(greenPower, yellowPower)));

        redPower    /=maxPower;
        bluePower   /=maxPower;
        greenPower  /=maxPower;
        yellowPower /=maxPower;

        redServo.setPosition(thetaToServo((float) redTheta));
        blueServo.setPosition(thetaToServo((float) blueTheta));
        greenServo.setPosition(thetaToServo((float) greenTheta));
        yellowServo.setPosition(thetaToServo((float) yellowTheta));

        redMotor.setPower(redPower);
        blueMotor.setPower(bluePower);
        greenMotor.setPower(greenPower);
        yellowMotor.setPower(yellowPower);
        //Template code
        //Servos
        /* for (ServoImplEx servo : servoList) {
         if (servo.equals(redServo) || servo.equals(yellowServo)) {
         servo.setPosition(thetaToServo(theta1));
         } else {
         servo.setPosition(thetaToServo(theta2));
         }
         }
         for (DcMotor motor : motorList) {
         if (motor.equals(redMotor) || motor.equals(yellowMotor)) {
         motor.setPower(power1);
         } else {
         motor.setPower(power2);
         }
         *
         */
    }
}

/*private float thetaToServo(float theta){
        float thetaInDegrees = (float) ((theta*180) / (Math.PI));
        //position of 1 means PWM of 2495 and 360-ish degree rotation
        //position of 0 means PWM of 505 and 0-ish degree rotation
        float degreesToServo = (thetaInDegrees + 180) / 360; // map range formula (simplified)
        return degreesToServo;
   }
 *
 */


}