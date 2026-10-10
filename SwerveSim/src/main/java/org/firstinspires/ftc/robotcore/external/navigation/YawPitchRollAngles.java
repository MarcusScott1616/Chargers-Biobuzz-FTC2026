package org.firstinspires.ftc.robotcore.external.navigation;

/* FAKE YawPitchRollAngles for SwerveSim. Stores angles in radians internally. */
public class YawPitchRollAngles {
    private final double yaw, pitch, roll;
    private final long acquisitionTime;

    public YawPitchRollAngles(AngleUnit unit, double yaw, double pitch, double roll, long acquisitionTime) {
        this.yaw = unit.toRadians(yaw);
        this.pitch = unit.toRadians(pitch);
        this.roll = unit.toRadians(roll);
        this.acquisitionTime = acquisitionTime;
    }

    public double getYaw(AngleUnit unit) { return unit.fromRadians(yaw); }
    public double getPitch(AngleUnit unit) { return unit.fromRadians(pitch); }
    public double getRoll(AngleUnit unit) { return unit.fromRadians(roll); }
    public long getAcquisitionTime() { return acquisitionTime; }
}
