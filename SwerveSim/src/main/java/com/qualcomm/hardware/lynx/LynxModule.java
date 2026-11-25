package com.qualcomm.hardware.lynx;

/*
 * FAKE version of LynxModule used only by SwerveSim.
 * On the real robot, the FTC SDK provides this class to control bulk read caching.
 */
public class LynxModule {
    public enum BulkCachingMode {
        OFF,
        AUTO,
        MANUAL
    }

    private BulkCachingMode mode = BulkCachingMode.OFF;

    public void setBulkCachingMode(BulkCachingMode mode) {
        this.mode = mode;
    }

    public BulkCachingMode getBulkCachingMode() {
        return mode;
    }

    public void clearBulkCache() {}
}
