package org.firstinspires.ftc.teamcode.pedro;

public class Tuning {

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
        // WHILE connected to robot controller, open this link into browser
        // http://192.168.43.1:10158
        //unsure if we can tune Swerve the same way as we do Mechanum
    }
}
