package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * TurretAutoLogic - the FEEDING sequence for both turrets:
 *   wait until the turrets are ready -> feed one ball -> pause -> repeat until out of shots.
 *
 * It does NOT aim, set the hood, or spin the flywheels. Turret.java does all of that
 * (it owns the flywheel motors). The autonomous tells us when they're ready, using
 * update(turretsReady), and we only run the index motor.
 *
 * Hardware (names must match the Driver Station configuration):
 *   "indexMotor" - motor that pushes balls into the turrets
 */
public class TurretAutoLogic {

    // TUNE: how long to wait for the turrets before feeding anyway. Only used if they never report ready.
    private static final double SPIN_UP_MAX_SECONDS = 2.0;
    private static final double INDEX_POWER = 0.5;     // index motor power while feeding a ball
    private static final double INDEX_SECONDS = 0.5;   // how long to feed one ball
    // TUNE: pause after each ball before the next one. Keep this short for faster volleys;
    // raise it if balls jam because the flywheel hasn't recovered speed yet.
    private static final double RESET_SECONDS = 0.75;

    public enum FeedState {
        IDLE,      // waiting for fireShots()
        WAIT,      // waiting for the turrets to be ready
        LAUNCH,    // index motor running, feeding one ball
        RESET      // pause, then either shoot again or go idle
    }

    private DcMotor indexMotor;

    private FeedState feedState = FeedState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private int shotsRemaining = 0;

    public void init(HardwareMap hwMap) {
        indexMotor = hwMap.get(DcMotor.class, "indexMotor");
        indexMotor.setPower(0);
    }

    /**
     * Call once every loop. Moves the feeding state machine along.
     * @param turretsReady true when both turrets are at speed and aimed (from Turret.readyToShoot())
     */
    public void update(boolean turretsReady) {
        switch (feedState) {
            case IDLE:
                if (shotsRemaining > 0) startState(FeedState.WAIT);
                break;

            case WAIT:
                // Feed as soon as the turrets are ready, or give up waiting after a while.
                if (turretsReady || stateTimer.seconds() > SPIN_UP_MAX_SECONDS) {
                    indexMotor.setPower(INDEX_POWER);
                    startState(FeedState.LAUNCH);
                }
                break;

            case LAUNCH:
                if (stateTimer.seconds() > INDEX_SECONDS) {
                    shotsRemaining--;
                    indexMotor.setPower(0);
                    startState(FeedState.RESET);
                }
                break;

            case RESET:
                if (stateTimer.seconds() > RESET_SECONDS) {
                    startState(shotsRemaining > 0 ? FeedState.WAIT : FeedState.IDLE);
                }
                break;
        }
    }

    /** Start a feeding sequence. Ignored if one is already running. */
    public void fireShots(int numberOfShots) {
        if (feedState == FeedState.IDLE) {
            shotsRemaining = numberOfShots;
        }
    }

    /** True while a feeding sequence is running. */
    public boolean isBusy() {
        return feedState != FeedState.IDLE;
    }

    /** Stop the index motor right away (for example, when the OpMode stops). */
    public void stop() {
        shotsRemaining = 0;
        indexMotor.setPower(0);
        startState(FeedState.IDLE);
    }

    /** Change state and restart the timer. */
    private void startState(FeedState next) {
        feedState = next;
        stateTimer.reset();
    }
}
