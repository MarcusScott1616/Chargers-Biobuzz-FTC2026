package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * TurretAutoLogic - the shooting sequence for both turrets:
 *   spin up the flywheels -> feed one ball -> pause -> repeat until out of shots.
 *
 * It does NOT aim or set the hood. NectarTurret and PollenTurret do that.
 *
 * Hardware (names must match the Driver Station configuration):
 *   "nectarFlywheel" - goBILDA 5203 6000 RPM motor, 1:1 to the flywheel
 *   "pollenFlywheel" - goBILDA 5203 6000 RPM motor, 1:1 to the flywheel
 *   "indexMotor"     - motor that pushes balls into the turrets
 */
public class TurretAutoLogic {

    // goBILDA 5203 encoder: 28 ticks per motor turn. Converts between ticks/sec and RPM.
    private static final double TICKS_PER_REV = 28;

    // TUNE: placeholder numbers from the first draft. The shot tables in NectarTurret and
    // PollenTurret use 2500-4600 RPM, so set these to match the distance you shoot from.
    private static final double TARGET_RPM = 1100;  // speed to shoot at
    private static final double READY_RPM = 1050;   // both flywheels must reach this before feeding (keep below TARGET_RPM)
    private static final double IDLE_RPM = 600;     // speed to hold between shooting sequences

    private static final double INDEX_POWER = 0.5;     // index motor power while feeding a ball
    private static final double SPIN_UP_MAX_SECONDS = 2.0; // stop waiting for speed after this long
    private static final double INDEX_SECONDS = 0.5;   // how long to feed one ball
    private static final double RESET_SECONDS = 2.0;   // pause after each ball before the next one

    public enum FlywheelState {
        IDLE,      // waiting for fireShots()
        SPIN_UP,   // flywheels heading to speed
        LAUNCH,    // index motor running, feeding one ball
        RESET      // pause, then either shoot again or go idle
    }

    private DcMotorEx nectarFlywheel, pollenFlywheel;
    private DcMotor indexMotor;

    private FlywheelState flywheelState = FlywheelState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private int shotsRemaining = 0;

    public void init(HardwareMap hwMap) {
        nectarFlywheel = hwMap.get(DcMotorEx.class, "nectarFlywheel");
        pollenFlywheel = hwMap.get(DcMotorEx.class, "pollenFlywheel");
        indexMotor = hwMap.get(DcMotor.class, "indexMotor");

        // RUN_USING_ENCODER lets setVelocity() hold a speed in ticks/sec.
        nectarFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        indexMotor.setPower(0);
        setFlywheelRpm(IDLE_RPM);
    }

    /** Call once every loop. Moves the shooting state machine along. */
    public void update() {
        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    setFlywheelRpm(TARGET_RPM);
                    startState(FlywheelState.SPIN_UP);
                }
                break;

            case SPIN_UP:
                // Feed as soon as both flywheels are fast enough, or give up waiting after a while.
                if (bothAtSpeed() || stateTimer.seconds() > SPIN_UP_MAX_SECONDS) {
                    indexMotor.setPower(INDEX_POWER);
                    startState(FlywheelState.LAUNCH);
                }
                break;

            case LAUNCH:
                if (stateTimer.seconds() > INDEX_SECONDS) {
                    shotsRemaining--;
                    indexMotor.setPower(0);
                    startState(FlywheelState.RESET);
                }
                break;

            case RESET:
                if (stateTimer.seconds() > RESET_SECONDS) {
                    if (shotsRemaining > 0) {
                        startState(FlywheelState.SPIN_UP);
                    } else {
                        setFlywheelRpm(IDLE_RPM);
                        startState(FlywheelState.IDLE);
                    }
                }
                break;
        }
    }

    /** Start a shooting sequence. Ignored if one is already running. */
    public void fireShots(int numberOfShots) {
        if (flywheelState == FlywheelState.IDLE) {
            shotsRemaining = numberOfShots;
        }
    }

    /** True while a shooting sequence is running. */
    public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
    }

    // ---------- helpers ----------

    /** Change state and restart the timer. */
    private void startState(FlywheelState next) {
        flywheelState = next;
        stateTimer.reset();
    }

    /** Ask both flywheels to hold a speed. setVelocity() takes ticks per second, so convert from RPM. */
    private void setFlywheelRpm(double rpm) {
        double ticksPerSecond = rpm * TICKS_PER_REV / 60.0;
        nectarFlywheel.setVelocity(ticksPerSecond);
        pollenFlywheel.setVelocity(ticksPerSecond);
    }

    private boolean bothAtSpeed() {
        return rpm(nectarFlywheel) >= READY_RPM && rpm(pollenFlywheel) >= READY_RPM;
    }

    /** Measured speed of one flywheel in RPM. */
    private static double rpm(DcMotorEx flywheel) {
        return flywheel.getVelocity() / TICKS_PER_REV * 60.0;
    }
}
