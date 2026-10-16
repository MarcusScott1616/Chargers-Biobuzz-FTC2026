package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

// No tuners are registered yet. Pedro's mecanum tuner measures mecanum wheels,
// so it does not apply to this swerve robot. Swerve tuning still needs a plan.
public class Tuning {
    @Tuner
    public static Procedure pinpointTuner() {
        //return new PinpointTuner();
        //I like the pinpoint tuner, it seems like the easiest one to setup.
    }
}

