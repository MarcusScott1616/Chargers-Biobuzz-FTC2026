package com.qualcomm.robotcore.hardware;

/* FAKE "extended" servo for SwerveSim (adds PWM range settings). */
public class ServoImplEx extends Servo {
    private PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(600, 2400);
    private boolean pwmEnabled = true;

    public void setPwmEnable() { pwmEnabled = true; }
    public void setPwmDisable() { pwmEnabled = false; }
    public boolean isPwmEnabled() { return pwmEnabled; }

    public void setPwmRange(PwmControl.PwmRange range) { this.pwmRange = range; }
    public PwmControl.PwmRange getPwmRange() { return pwmRange; }
}
