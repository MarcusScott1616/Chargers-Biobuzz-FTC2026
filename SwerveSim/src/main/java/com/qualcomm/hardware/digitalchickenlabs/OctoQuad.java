package com.qualcomm.hardware.digitalchickenlabs;

/*
 * FAKE OctoQuad for SwerveSim. The simulator uses motor encoder ports (USE_OCTOQUAD = false),
 * so this interface/class exists so that SwerveDriveV2.java compiles cleanly in SwerveSim.
 */
public interface OctoQuad {
    enum EncoderDirection { FORWARD, REVERSE }
    enum CachingMode { NONE, AUTO, MANUAL }

    void setCachingMode(CachingMode mode);
    int readSinglePosition(int channel);
    int readSinglePosition_Caching(int channel);
    void resetSinglePosition(int channel);
    void resetAllPositions();
}
