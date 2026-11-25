package turretsim;

import java.util.List;

/**
 * Standalone table generator - prints optimised angle+RPM for each distance.
 * Run via:  ./gradlew :SwerveSim:runTable
 */
public class TableGen {

    public static void main(String[] args) {
        int[] distancesFt = {4, 5, 6, 7, 8, 9, 10, 11, 12};

        System.out.println("=".repeat(97));
        System.out.println("NECTAR TURRET  -- Front of robot (intake side) | flywheel height 6 in | Nectar ball 3.6 in");
        System.out.println("=".repeat(97));
        System.out.printf("  %-9s %-9s %-8s %-6s %-9s %-8s %-9s %-7s  %s%n",
                "Dist(ft)", "Dist(in)", "Angle", "RPM", "ExitSpd", "Apex", "Entry", "Time", "Result");
        for (int ft : distancesFt) {
            printBest(ft, 6, ShotPhysics.Ball.NECTAR);
        }

        System.out.println();
        System.out.println("=".repeat(97));
        System.out.println("POLLEN TURRET  -- Rear of robot                | flywheel height 8 in | Pollen ball 2.8 in");
        System.out.println("=".repeat(97));
        System.out.printf("  %-9s %-9s %-8s %-6s %-9s %-8s %-9s %-7s  %s%n",
                "Dist(ft)", "Dist(in)", "Angle", "RPM", "ExitSpd", "Apex", "Entry", "Time", "Result");
        for (int ft : distancesFt) {
            printBest(ft, 8, ShotPhysics.Ball.POLLEN);
        }
    }

    private static void printBest(int distFt, double heightIn, ShotPhysics.Ball ball) {
        int distIn = distFt * 12;
        double bestScore = Double.NEGATIVE_INFINITY;
        ShotPhysics.Result best = null;
        double bestAngle = 0, bestRpm = 0;

        for (int angTenths = 200; angTenths <= 500; angTenths += 5) {   // 20.0 .. 50.0 deg in 0.5 steps
            double angle = angTenths / 10.0;
            for (int rpm = 2500; rpm <= 5500; rpm += 25) {
                ShotPhysics.Settings s = makeSettings(distIn, heightIn, angle, rpm, ball);
                ShotPhysics.Result r = ShotPhysics.simulate(s);
                if (r.outcome == ShotPhysics.Outcome.SCORED) {
                    // Favour steeper entry (ball stays in better) over lower RPM.
                    double score = r.entryAngleDeg - rpm * 0.001;
                    if (score > bestScore) {
                        bestScore = score;
                        best = r;
                        bestAngle = angle;
                        bestRpm = rpm;
                    }
                    break;   // first scoring RPM for this angle found; move to next angle
                }
            }
        }

        if (best == null) {
            System.out.printf("  %-9d %-9d  -- no solution found --%n", distFt, distIn);
        } else {
            System.out.printf("  %d ft       %-9d %-8.1f %-6.0f %-9.1f %-8.1f %-9.1f %-7.3f  SCORED%n",
                    distFt, distIn, bestAngle, bestRpm,
                    best.exitSpeedInPerSec, best.apexIn, best.entryAngleDeg, best.flightTime);
        }
    }

    private static ShotPhysics.Settings makeSettings(int distIn, double heightIn, double angleDeg,
                                                      double rpm, ShotPhysics.Ball ball) {
        ShotPhysics.Settings s = new ShotPhysics.Settings();
        s.ball = ball;
        s.ballMassGrams = ball.defaultMassGrams;
        s.distanceIn = distIn;
        s.launchHeightIn = heightIn;
        s.launchAngleDeg = angleDeg;
        s.flywheelRpm = rpm;
        s.flywheelDiameterIn = 4;
        s.compressionPct = 15;
        s.spin = ShotPhysics.Spin.BACKSPIN;
        s.airEffects = true;
        return s;
    }
}
