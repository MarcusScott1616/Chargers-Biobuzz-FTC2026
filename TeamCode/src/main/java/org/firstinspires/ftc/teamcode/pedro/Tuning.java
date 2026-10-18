package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

public class Tuning {

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
        //I like the pinpoint tuner, it seems like the easiest one to set up.
    }
}

