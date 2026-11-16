package com.qualcomm.robotcore.hardware;

/* FAKE PwmControl for SwerveSim. */
public interface PwmControl {
    class PwmRange {
        public final double usPulseLower;
        public final double usPulseUpper;

        public PwmRange(double usPulseLower, double usPulseUpper) {
            this.usPulseLower = usPulseLower;
            this.usPulseUpper = usPulseUpper;
        }
    }

    void setPwmEnable();
    void setPwmDisable();
    boolean isPwmEnabled();
}
