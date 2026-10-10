package org.firstinspires.ftc.robotcore.external.navigation;

/* FAKE AngleUnit for SwerveSim. */
public enum AngleUnit {
    DEGREES, RADIANS;

    public double fromRadians(double radians) { return this == RADIANS ? radians : Math.toDegrees(radians); }
    public double toRadians(double angle) { return this == RADIANS ? angle : Math.toRadians(angle); }
}
