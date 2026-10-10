package com.qualcomm.robotcore.hardware;

/* FAKE servo for SwerveSim. Remembers the position it was told to go to. */
public class Servo {
    public enum Direction { FORWARD, REVERSE }

    private Direction direction = Direction.FORWARD;
    private double position = 0.5;
    private double requestedPosition = 0.5;

    public void setDirection(Direction direction) { this.direction = direction; }
    public Direction getDirection() { return direction; }

    // The real servo clips to 0..1, so we do too. We also keep the un-clipped
    // request so the simulator can warn when code asks for an impossible position.
    public void setPosition(double position) {
        this.requestedPosition = position;
        this.position = Math.max(0, Math.min(1, position));
    }
    public double getPosition() { return position; }
    public double getRequestedPosition() { return requestedPosition; }
}
