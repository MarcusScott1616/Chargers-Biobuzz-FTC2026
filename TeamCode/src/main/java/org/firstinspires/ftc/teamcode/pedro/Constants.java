package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;

public class Constants {

    // Path-following config. Pedro gives no defaults for the braking, heading and
    // deceleration values, so they stay unset until they are tuned on the robot.
    public static ForesightConfig foresightConfig = new ForesightConfig(config -> { });

    /**
     * Builds a Follower from a localizer and drivetrain.
     * Both are passed in because the swerve pod wiring and localizer hardware
     * are not set up yet.
     */
    public static Follower createFollower(Localizer localizer, Drivetrain drivetrain) {
        return new Follower(localizer, drivetrain, new Foresight(foresightConfig));
    }
}
