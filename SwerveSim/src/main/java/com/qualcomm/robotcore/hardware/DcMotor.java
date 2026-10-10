package com.qualcomm.robotcore.hardware;

/*
 * FAKE drive motor for SwerveSim. It doesn't spin anything - it just remembers
 * what SwerveDrive told it so the simulator can read it back.
 *
 * If SwerveDrive starts using a motor method that isn't here, the sim won't compile.
 * Fix: add that method below (it usually only needs to store or return a value).
 */
public class DcMotor {
    public enum Direction { FORWARD, REVERSE }
    public enum ZeroPowerBehavior { UNKNOWN, BRAKE, FLOAT }
    public enum RunMode {
        RUN_WITHOUT_ENCODER, RUN_USING_ENCODER, RUN_TO_POSITION, STOP_AND_RESET_ENCODER,
        @Deprecated RUN_WITHOUT_ENCODERS, @Deprecated RUN_USING_ENCODERS, @Deprecated RESET_ENCODERS
    }

    private Direction direction = Direction.FORWARD;
    private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.FLOAT;
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private double power = 0;
    private int targetPosition = 0;

    public void setDirection(Direction direction) { this.direction = direction; }
    public Direction getDirection() { return direction; }

    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) { this.zeroPowerBehavior = behavior; }
    public ZeroPowerBehavior getZeroPowerBehavior() { return zeroPowerBehavior; }

    public void setMode(RunMode mode) { this.mode = mode; }
    public RunMode getMode() { return mode; }

    // Like the real motor, power is limited to -1..1.
    public void setPower(double power) { this.power = Math.max(-1, Math.min(1, power)); }
    public double getPower() { return power; }

    public void setTargetPosition(int position) { this.targetPosition = position; }
    public int getTargetPosition() { return targetPosition; }
}
