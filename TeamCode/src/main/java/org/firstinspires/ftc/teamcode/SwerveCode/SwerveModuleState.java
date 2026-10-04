package org.firstinspires.ftc.teamcode.SwerveCode;

public class SwerveModuleState {
    public double angle; // Angle in radians
    public double power; // motor power.

    public SwerveModuleState(double angle, double power) {
        this.angle = angle;
        this.power = power;
    }

        /*
         * Optimizes the target angle and motor power so the wheel never rotates more than 90 degrees.
         *
         * @param targetAngle Desired module heading in radians (-PI to PI)
         * @param targetPower Desired motor power
         * @param currentAngle Current wheel position in radians
         * @return Optimized SwerveModuleState containing adjusted angle and power
         */
    public SwerveModuleState optimize(double targetAngle, double targetPower, double currentPower, double currentAngle){
        double delta = targetAngle-currentAngle;

        while (delta > Math.PI) delta -= 2 * Math.PI;
        while (delta < -Math.PI) delta += 2 * Math.PI;

        if (Math.abs(delta) > Math.PI / 2.0 ){
            targetPower *= -1.0;
            targetAngle += (delta > 0) ? -Math.PI : Math.PI;

        }
        return new SwerveModuleState(targetAngle, targetPower);

    }

}
