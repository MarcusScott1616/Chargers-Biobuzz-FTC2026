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


/*
 *
 * I altered:
 * Biggest thing was Swerve Modules. Rather than calculating the power and degrees for
 * one swerve module and copy/pasting it to all of them,
 * I created individual programming for each module
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
    public ServoImplEx[] servoList;

    private DcMotor redMotor, blueMotor, greenMotor, yellowMotor;
    private DcMotor[] motorList;
    private IMU.Parameters imuParams;
    private IMU imu;

    public int WheelbaseLength = 17;
    public int WheelbaseWidth = 17;

    private double currentRedTheta = 0, currentBlueTheta = 0, currentGreenTheta =0, currentYellowTheta = 0;

    public static class SwerveModuleState {
        public double angle;
        public double power;

        public SwerveModuleState(double angle, double power) {
            this.angle = angle;
            this.power = power;
        }
    }

    public void init(HardwareMap hMap) {
        // sample code did not grab hardware map properly.... Replaced:
        // HardwareMap hMap = new HardwareMap(); with the above parameter

        imu = hMap.get(IMU.class, "imu");


        redServo = hMap.get(ServoImplEx.class, "redServo");
        blueServo = hMap.get(ServoImplEx.class, "blueServo");
        greenServo = hMap.get(ServoImplEx.class, "greenServo");
        yellowServo = hMap.get(ServoImplEx.class, "yellowServo");

        servoList = new ServoImplEx[]{redServo, blueServo, greenServo, yellowServo};

        redMotor = hMap.get(DcMotor.class, "redMotor");
        blueMotor = hMap.get(DcMotor.class, "blueMotor");
        yellowMotor = hMap.get(DcMotor.class, "yellowMotor");
        greenMotor = hMap.get(DcMotor.class, "greenMotor");

        motorList = new DcMotor[]{redMotor, blueMotor, greenMotor, yellowMotor};


        imuParams = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
           )
        );

        imu.initialize(imuParams);
        imu.resetYaw();



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
     * @param strafeX Power for Left-Right Locomotion
     * @param strafeY Power for Forward-Back Locomotion
     * @param rotate The extent to which the drivetrain rotates
     */

	public void move(float strafeX, float strafeY, float rotate) {

        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double yaw = robotOrientation.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);
        //These are the template Calculations, Improper
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

        double fieldX = strafeX * cos(-yaw) - strafeY * sin(-yaw);
        double fieldY = strafeX * sin(-yaw) + strafeY * cos(-yaw);

        double L = WheelbaseLength; // Length of Chassis
        double W = WheelbaseWidth; //Width of Chassis
        double R = Math.hypot(L, W); // the diagonal of the chassis

        // Component Vectors
        double red = fieldX - rotate * (L / R);
        double blue = fieldX + rotate * (L / R);
        double green = fieldY + rotate * (W / R);
        double yellow = fieldY - rotate * (W / R);

        double redPower = Math.hypot(blue, yellow);
        double bluePower = Math.hypot(blue, green);
        double greenPower = Math.hypot(red, yellow);
        double yellowPower = Math.hypot(red, green);

        double redTheta = Math.atan2(blue, yellow);
        double blueTheta = Math.atan2(blue, green);
        double greenTheta = Math.atan2(red, yellow);
        double yellowTheta = Math.atan2(red, green);

        double maxPower = Math.max(1.0, Math.max(Math.max(redPower, bluePower), Math.max(greenPower, yellowPower)));

        redPower /= maxPower;
        bluePower /= maxPower;
        greenPower /= maxPower;
        yellowPower /= maxPower;

        SwerveModuleState redState      = optimize(redTheta, redPower, currentRedTheta);
        SwerveModuleState blueState     = optimize(blueTheta, bluePower, currentBlueTheta);
        SwerveModuleState greenState    = optimize(greenTheta, greenPower, currentGreenTheta);
        SwerveModuleState yellowState   = optimize(yellowTheta, yellowPower, currentYellowTheta);

        currentRedTheta      = redState.angle;
        currentBlueTheta     = blueState.angle;
        currentGreenTheta    = greenState.angle;
        currentYellowTheta   = yellowState.angle;

        redServo.setPosition(thetaToServo(redState.angle));
        blueServo.setPosition(thetaToServo(blueState.angle));
        greenServo.setPosition(thetaToServo(greenState.angle));
        yellowServo.setPosition(thetaToServo(yellowState.angle));

        redMotor.setPower(redState.power);
        blueMotor.setPower(blueState.power);
        greenMotor.setPower(greenState.power);
        yellowMotor.setPower(yellowState.power);

    }
            public SwerveModuleState optimize(double targetAngle, double targetPower, double currentAngle) {
                double delta = targetAngle - currentAngle;

                while (delta > Math.PI) delta -= 2 * Math.PI;
                while (delta < -Math.PI) delta += 2 * Math.PI;

                if (Math.abs(delta) > Math.PI / 2.0) {
                    targetPower *= -1.0;
                    targetAngle += (delta > 0) ? -Math.PI : Math.PI;
                }

                return new SwerveModuleState(targetAngle, targetPower);
            }

            private double thetaToServo(double theta){
                double thetaInDegrees = Math.toDegrees(theta);
                return (thetaInDegrees + 180.0) /360.0;
            }

}
