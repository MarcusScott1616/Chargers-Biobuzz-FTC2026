package turretsim;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/*
 * The physics of one shot from a single-flywheel + hood shooter, seen from the side.
 *
 * Everything the user can change lives in Settings (inches, degrees, RPM).
 * simulate() flies the ball step by step and reports what happened.
 *
 * How the shooter launches the ball (single flywheel + hood):
 *   The ball is squeezed between the spinning wheel and the hood, which doesn't move.
 *   The side touching the wheel moves at the wheel's surface speed; the side touching
 *   the hood doesn't move. So the ball ROLLS along the hood: its center moves at about
 *   HALF the wheel's surface speed, and it leaves spinning.
 *     - Wheel under the ball, hood on top  -> BACKSPIN (ball tends to float / carry farther)
 *     - Wheel over the ball, hood underneath -> TOPSPIN (ball dives down sooner)
 *
 * Compression = how much the gap is smaller than the ball (% of the ball's diameter).
 *   Too little: the wheel slips and the ball comes out slow.
 *   Some: good grip.
 *   Too much: the ball gets crushed and loses energy.
 *   The grip curve below is MADE UP to show that idea. Tune it to match real test shots!
 *
 * Air: drag slows the ball down, and spin bends its path (Magnus effect - the same thing
 * that makes a curveball curve). Both use typical numbers for a foam ball; they are estimates.
 *
 * Staying in: after the ball goes through the opening it bounces around inside the cell
 * (back wall, floor, ceiling). A fast, flat shot can hit the back wall and bounce right
 * back out. "Bounciness" = how much speed the ball keeps on each bounce (0 = dead, 1 = superball). The game balls are rigid plastic like pickleballs, about 0.64.
 *
 * Odds: a real robot never shoots exactly the same twice - the flywheel speed sags, the
 * angle wiggles, the driver stops in a slightly different spot. odds() fires practice
 * shots with that "wobble" mixed in and counts how many score AND stay in.
 */
public class ShotPhysics {

    public enum Ball {
        NECTAR("Nectar", 3.6, 30, new Color(40, 110, 230)),
        POLLEN("Pollen", 2.8, 15, new Color(235, 200, 20));

        public final String label;
        public final double diameterIn;
        public final double defaultMassGrams; // GUESS - weigh the real balls and fix this!
        public final Color color;

        Ball(String label, double diameterIn, double defaultMassGrams, Color color) {
            this.label = label; this.diameterIn = diameterIn; this.defaultMassGrams = defaultMassGrams; this.color = color;
        }

        @Override public String toString() { return label + " (" + diameterIn + " in)"; }
    }

    public enum Spin {
        BACKSPIN("Backspin", 1),   // wheel under the ball, hood on top
        TOPSPIN("Topspin", -1),    // wheel over the ball, hood underneath
        NONE("No spin", 0);

        public final String label;
        final int sign;
        Spin(String label, int sign) { this.label = label; this.sign = sign; }
        @Override public String toString() { return label; }
    }

    /** Everything you can adjust. Inches, degrees, RPM. */
    public static class Settings {
        // Ball
        public Ball ball = Ball.NECTAR;
        public double ballMassGrams = Ball.NECTAR.defaultMassGrams;

        // Shooter
        public double distanceIn = 72;          // floor distance from where the ball leaves the shooter to the bottom edge of the opening
        public double launchHeightIn = 14;      // height where the ball leaves the shooter
        public double launchAngleDeg = 55;      // 0 = flat, 90 = straight up
        public double flywheelRpm = 3000;
        public double flywheelDiameterIn = 4;
        public double compressionPct = 15;
        public Spin spin = Spin.BACKSPIN;
        public boolean airEffects = true;       // drag + spin curve

        // HIVE (defaults from the game manual, Figure 9-10)
        public double openingBottomIn = 53.5;   // bottom edge of the opening above the tiles
        public double openingTopIn = 65.6;      // top edge of the opening above the tiles
        public double openingTiltDeg = 30;      // tilt of the opening from vertical; + = top edge leans away from the shooter
        public double cellDepthIn = 12.04;        // how deep the cell box is behind the opening
        public double hiveBottomIn = 30.6;      // bottom of the HIVE above the tiles
        public double bounciness = 0.64;        // rigid plastic, like a pickleball (dropped from 78 in, bounces ~32 in). Measure on the real HIVE!

        // Wobble: how far each shot is typically off from the setting (about 2 out of 3 shots are within this)
        public double rpmWobble = 75;
        public double angleWobbleDeg = 1;
        public double distanceWobbleIn = 3;
    }

    public enum Outcome { SCORED, BOUNCED_OUT, RIM, HIT_HIVE, SHORT, LONG, NO_SHOT }

    public static class Result {
        public final List<double[]> path = new ArrayList<>(); // {time s, x in, z in, ball spin angle rad}
        public Outcome outcome = Outcome.NO_SHOT;
        public String message = "";
        public double surfaceSpeedInPerSec, exitSpeedInPerSec, ballSpinRpm, grip;
        public double apexIn, entryAngleDeg, flightTime;
        public boolean risingAtTarget;
        public double hitX, hitZ;          // where the ball reached the HIVE (or the floor), inches
    }

    /** The result of many practice shots with wobble. */
    public static class Odds {
        public int shots, stayed, bouncedOut, rim, missed;
        public final List<double[]> hits = new ArrayList<>(); // {x in, z in, outcome number} for each practice shot
        public double chance() { return shots == 0 ? 0 : 100.0 * stayed / shots; }
    }

    // Physics constants (SI units: meters, kg, seconds)
    private static final double G = 9.81;
    private static final double AIR_DENSITY = 1.2;
    private static final double DRAG_COEFFICIENT = 0.5;  // typical for a ball
    private static final double MAX_LIFT_COEFFICIENT = 0.35;
    private static final double IN = 0.0254;             // meters per inch
    private static final double DT = 0.001;
    private static final double ROLL_KEEP = 0.6;           // hollow ball: keeps 3/5 of its sliding speed on a bounce

    /** Made-up grip curve: how much of the ideal speed the ball actually gets (0..1). */
    public static double grip(double compressionPct) {
        if (compressionPct <= 0) return 0.3;                                  // barely touching: lots of slip
        if (compressionPct < 20) return 0.3 + 0.7 * compressionPct / 20;       // more squeeze = more grip
        return Math.max(0.4, 1.0 - 0.015 * (compressionPct - 20));             // over-squeezed = energy lost
    }

    public static Result simulate(Settings s) {
        Result r = new Result();

        // Launch
        r.surfaceSpeedInPerSec = s.flywheelRpm / 60.0 * Math.PI * s.flywheelDiameterIn;
        r.grip = grip(s.compressionPct);
        r.exitSpeedInPerSec = 0.5 * r.surfaceSpeedInPerSec * r.grip;   // ball rolls along the hood
        double ballRadius = s.ball.diameterIn / 2 * IN;
        double speed = r.exitSpeedInPerSec * IN;
        double spinRadPerSec = s.spin.sign * speed / ballRadius;       // rolling: spin = speed / radius
        r.ballSpinRpm = Math.abs(spinRadPerSec) * 60 / (2 * Math.PI);

        double angle = Math.toRadians(s.launchAngleDeg);
        double x = 0, z = s.launchHeightIn * IN;
        double vx = speed * Math.cos(angle), vz = speed * Math.sin(angle);
        double spinAngle = 0;
        double mass = Math.max(1, s.ballMassGrams) / 1000.0;
        double area = Math.PI * ballRadius * ballRadius;

        // HIVE opening, as a line from its bottom edge B to its top edge T.
        double tilt = Math.toRadians(s.openingTiltDeg);
        double bx = s.distanceIn * IN, bz = s.openingBottomIn * IN;
        double openingLength = (s.openingTopIn - s.openingBottomIn) / Math.cos(tilt) * IN;
        double ux = Math.sin(tilt), uz = Math.cos(tilt);   // along the opening, bottom -> top
        double nx = Math.cos(tilt), nz = -Math.sin(tilt);  // into the cell, away from the shooter
        double hiveFaceBelow = (s.openingBottomIn - s.hiveBottomIn) / Math.cos(tilt) * IN;

        r.apexIn = s.launchHeightIn;
        r.path.add(new double[]{0, x / IN, z / IN, 0});
        if (speed <= 0.01) {
            r.message = "Flywheel isn't spinning fast enough to launch.";
            return r;
        }

        double t = 0;
        boolean wentUnder = false;
        double prevDepth = (x - bx) * nx + (z - bz) * nz;
        while (t < 5) {
            // Forces -> acceleration
            double ax = 0, az = -G;
            if (s.airEffects) {
                double v = Math.hypot(vx, vz);
                double drag = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * area * v / mass;
                ax -= drag * vx;
                az -= drag * vz;
                if (spinRadPerSec != 0 && v > 0.1) {
                    double spinRatio = Math.abs(spinRadPerSec) * ballRadius / v;
                    double lift = 0.5 * AIR_DENSITY * Math.min(MAX_LIFT_COEFFICIENT, spinRatio) * area * v / mass;
                    double sign = Math.signum(spinRadPerSec);   // backspin pushes up-and-back, topspin down
                    ax += sign * lift * -vz;
                    az += sign * lift * vx;
                }
            }

            vx += ax * DT;
            vz += az * DT;
            x += vx * DT;
            z += vz * DT;
            t += DT;
            spinAngle += spinRadPerSec * DT;
            r.apexIn = Math.max(r.apexIn, z / IN);
            r.path.add(new double[]{t, x / IN, z / IN, spinAngle});

            // Did the ball's center cross the plane of the opening this step?
            double depth = (x - bx) * nx + (z - bz) * nz;
            if (prevDepth < 0 && depth >= 0) {
                r.hitX = x / IN; r.hitZ = z / IN;
                double along = (x - bx) * ux + (z - bz) * uz;   // where along the opening (0 = bottom edge)
                r.flightTime = t;
                r.risingAtTarget = vz > 0;
                r.entryAngleDeg = Math.toDegrees(Math.atan2(-vz, vx));
                if (along >= ballRadius && along <= openingLength - ballRadius) {
                    if (bounceInCell(r, s, x, z, vx, vz, spinAngle, spinRadPerSec, t,
                                     bx, bz, ux, uz, nx, nz, openingLength, ballRadius)) {
                        r.outcome = Outcome.SCORED;
                        r.message = "SCORED - and it stayed in!";
                    } else {
                        r.outcome = Outcome.BOUNCED_OUT;
                        r.message = "Went in, but BOUNCED OUT - too fast. Try a slower, loopier shot.";
                    }
                    return r;
                }
                if (along > -ballRadius && along < openingLength + ballRadius) {
                    r.outcome = Outcome.RIM;
                    r.message = along < openingLength / 2 ? "Hit the BOTTOM rim of the opening (a bit short)."
                                                          : "Hit the TOP rim of the opening (a bit long).";
                    return r;
                }
                if (along < 0 && along > -hiveFaceBelow) {
                    r.outcome = Outcome.HIT_HIVE;
                    r.message = "Hit the HIVE below the opening - too low.";
                    return r;
                }
                if (along <= -hiveFaceBelow) wentUnder = true;
            }
            prevDepth = depth;

            if (z <= ballRadius) {
                r.flightTime = t;
                r.hitX = x / IN; r.hitZ = z / IN;
                if (wentUnder) {
                    r.outcome = Outcome.SHORT;
                    r.message = "Went UNDER the HIVE - way too low.";
                } else {
                    r.outcome = x < bx ? Outcome.SHORT : Outcome.LONG;
                    r.message = x < bx ? "Landed SHORT of the HIVE." : "Flew OVER the HIVE (long).";
                }
                return r;
            }
        }
        r.message = "Still flying after 5 s?";
        return r;
    }

    /**
     * The ball just went through the opening. Bounce it around inside the cell to see if it STAYS.
     * The cell is a box behind the opening: a back wall, a floor (starting at the opening's bottom edge)
     * and a ceiling (starting at its top edge). On each bounce the ball keeps only "bounciness" of its
     * speed into that wall. If its center comes back out through the opening, it bounced out.
     * (Spin and air are ignored inside the cell - the ball is only in there for a moment.)
     * Returns true if the ball stayed in.
     */
    private static boolean bounceInCell(Result r, Settings s, double x, double z, double vx, double vz,
                                        double spinAngle, double spinRate, double t,
                                        double bx, double bz, double ux, double uz, double nx, double nz,
                                        double openingLength, double ballRadius) {
        double backWall = s.cellDepthIn * IN - ballRadius;     // deepest the ball's center can go
        double e = Math.max(0, Math.min(1, s.bounciness));
        int stillSteps = 0;
        for (int i = 0; i < 3000; i++) {                       // watch for up to 3 seconds
            vz -= G * DT;
            x += vx * DT;
            z += vz * DT;
            t += DT;
            spinAngle += spinRate * DT;

            // Split position and velocity into "into the cell" (n) and "along the opening" (u) parts.
            double depth = (x - bx) * nx + (z - bz) * nz;
            double along = (x - bx) * ux + (z - bz) * uz;
            double vIn = vx * nx + vz * nz;
            double vUp = vx * ux + vz * uz;
            // Each bounce: the speed INTO the wall flips and keeps "bounciness" of itself, and the
            // speed ALONG the wall drops to 3/5 - friction grabs the ball and turns that speed into
            // spin (that's what happens to a hollow ball like a pickleball when it starts rolling).
            if (depth > backWall && vIn > 0) { vIn = -e * vIn; vUp *= ROLL_KEEP; spinRate *= e; }   // back wall
            if (along < ballRadius && vUp < 0) { vUp = -e * vUp; vIn *= ROLL_KEEP; }                // floor
            if (along > openingLength - ballRadius && vUp > 0) { vUp = -e * vUp; vIn *= ROLL_KEEP; } // ceiling
            vx = vIn * nx + vUp * ux;
            vz = vIn * nz + vUp * uz;
            r.path.add(new double[]{t, x / IN, z / IN, spinAngle});

            if (depth < 0) return false;                       // came back out through the opening
            stillSteps = Math.hypot(vx, vz) < 0.1 ? stillSteps + 1 : 0;
            if (stillSteps > 150) return true;                 // settled in the corner of the cell
        }
        return true;
    }

    /**
     * Fire many practice shots, each a little off (random wobble in RPM, angle and distance),
     * and count how many score AND stay in. Uses the same random numbers every time, so the
     * percentage only changes when you change a setting.
     */
    public static Odds odds(Settings s, int shots) {
        Random random = new Random(2026);
        Settings test = copy(s);
        Odds odds = new Odds();
        for (int i = 0; i < shots; i++) {
            // nextGaussian(): usually between -1 and 1, sometimes more - like real shot-to-shot error.
            test.flywheelRpm = Math.max(0, s.flywheelRpm + random.nextGaussian() * s.rpmWobble);
            test.launchAngleDeg = s.launchAngleDeg + random.nextGaussian() * s.angleWobbleDeg;
            test.distanceIn = s.distanceIn + random.nextGaussian() * s.distanceWobbleIn;
            Result r = simulate(test);
            odds.shots++;
            switch (r.outcome) {
                case SCORED: odds.stayed++; break;
                case BOUNCED_OUT: odds.bouncedOut++; break;
                case RIM: odds.rim++; break;
                default: odds.missed++;
            }
            // Shift each dot by how far that shot's robot was off, so all dots line up on the drawn HIVE.
            odds.hits.add(new double[]{r.hitX - (test.distanceIn - s.distanceIn), r.hitZ, r.outcome.ordinal()});
        }
        return odds;
    }

    /** Try every flywheel speed with the other settings unchanged. Returns {fromRpm, toRpm} ranges that score. */
    public static List<double[]> scoringRpmRanges(Settings s, double maxRpm, double step) {
        Settings test = copy(s);
        List<double[]> ranges = new ArrayList<>();
        double[] current = null;
        for (double rpm = step; rpm <= maxRpm; rpm += step) {
            test.flywheelRpm = rpm;
            boolean scored = simulate(test).outcome == Outcome.SCORED;
            if (scored && current == null) current = new double[]{rpm, rpm};
            else if (scored) current[1] = rpm;
            else if (current != null) { ranges.add(current); current = null; }
        }
        if (current != null) ranges.add(current);
        return ranges;
    }

    private static Settings copy(Settings s) {
        Settings c = new Settings();
        c.ball = s.ball; c.ballMassGrams = s.ballMassGrams;
        c.distanceIn = s.distanceIn; c.launchHeightIn = s.launchHeightIn; c.launchAngleDeg = s.launchAngleDeg;
        c.flywheelRpm = s.flywheelRpm; c.flywheelDiameterIn = s.flywheelDiameterIn; c.compressionPct = s.compressionPct;
        c.spin = s.spin; c.airEffects = s.airEffects;
        c.openingBottomIn = s.openingBottomIn; c.openingTopIn = s.openingTopIn; c.openingTiltDeg = s.openingTiltDeg;
        c.cellDepthIn = s.cellDepthIn; c.hiveBottomIn = s.hiveBottomIn; c.bounciness = s.bounciness;
        c.rpmWobble = s.rpmWobble; c.angleWobbleDeg = s.angleWobbleDeg; c.distanceWobbleIn = s.distanceWobbleIn;
        return c;
    }
}
