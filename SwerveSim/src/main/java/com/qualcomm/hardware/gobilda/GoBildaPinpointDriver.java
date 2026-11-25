package com.qualcomm.hardware.gobilda;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * FAKE GoBildaPinpointDriver for SwerveSim.
 *
 * In SwerveSim on a laptop, no physical Pinpoint computer is present.
 * Throwing an exception in the constructor allows SwerveDriveV2 to catch it
 * and fall back to the simulated REV IMU, which Physics.java drives.
 */
public class GoBildaPinpointDriver {

    public GoBildaPinpointDriver() {
        throw new UnsupportedOperationException("No physical Pinpoint in SwerveSim - falling back to simulated IMU.");
    }

    public void update() {}

    public double getHeading(AngleUnit unit) {
        return 0.0;
    }
}
