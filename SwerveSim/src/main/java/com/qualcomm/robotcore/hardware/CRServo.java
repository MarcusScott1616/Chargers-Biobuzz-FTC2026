package com.qualcomm.robotcore.hardware;

/*
 * FAKE continuous rotation servo for SwerveSim.
 */
public class CRServo implements PwmControl {
    public enum Direction { FORWARD, REVERSE }

    private Direction direction = Direction.FORWARD;
    private double power = 0;
    private boolean pwmEnabled = true;

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public Direction getDirection() {
        return direction;
    }

    public void setPower(double power) {
        this.power = Math.max(-1, Math.min(1, power));
    }

    public double getPower() {
        return pwmEnabled ? power : 0;
    }

    @Override
    public void setPwmEnable() {
        this.pwmEnabled = true;
    }

    @Override
    public void setPwmDisable() {
        this.pwmEnabled = false;
    }

    @Override
    public boolean isPwmEnabled() {
        return pwmEnabled;
    }
}
