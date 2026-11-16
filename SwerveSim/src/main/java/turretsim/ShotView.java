package turretsim;

import javax.swing.JPanel;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.util.List;

/*
 * Draws the shots from the side: shooters on the left, HIVE on the right.
 * Like SwerveSim's FieldView, we draw in INCHES with +z (height) pointing up the screen.
 *
 * The robot has two turrets, and both balls fly at the same time. Each turret is drawn at its
 * own spot along the floor (its "origin"), with its own ball color. The back turret is at x = 0.
 */
public class ShotView extends JPanel {
    public static final double RPM_BAR_MAX = 6000;

    /** One turret on the robot: its settings, where it sits, and the results TurretSim worked out for it. */
    public static class Turret {
        public final ShotPhysics.Settings settings;
        public double originXIn;                 // where its launch point sits along the floor (0 = back turret)
        public ShotPhysics.Result result;
        public List<double[]> scoringRanges;
        public ShotPhysics.Odds odds;
        public double flywheelAngle;             // where the drawn flywheel is in its spin

        public Turret(ShotPhysics.Settings settings, double originXIn) {
            this.settings = settings;
            this.originXIn = originXIn;
        }

        /** When this turret's ball finishes its flight, in seconds. */
        public double endTime() { return result.path.get(result.path.size() - 1)[0]; }
    }

    private final List<Turret> turrets;     // index 0 = back turret, 1 = front turret
    private double animTime = -1;           // -1 = not animating, show each ball at the end of its shot

    public ShotView(List<Turret> turrets) {
        this.turrets = turrets;
        setBackground(new Color(30, 32, 40));
    }

    public void setAnimTime(double t) { animTime = t; }

    @Override
    protected void paintComponent(Graphics graphics) {
        super.paintComponent(graphics);
        for (Turret t : turrets) if (t.result == null) return;
        Graphics2D g = (Graphics2D) graphics.create();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // The HIVE is shared, so its position comes from the back turret.
        ShotPhysics.Settings hive = turrets.get(0).settings;

        // Pick a zoom so both turrets, the HIVE, and every flight path fit.
        double minX = -20, maxX = hive.distanceIn + hive.cellDepthIn + 30;
        double maxZ = hive.openingTopIn + 20;
        for (Turret t : turrets) {
            maxX = Math.max(maxX, t.originXIn + 30);
            maxZ = Math.max(maxZ, t.result.apexIn + 10);
            for (double[] p : t.result.path) maxX = Math.max(maxX, Math.min(p[1] + t.originXIn + 6, hive.distanceIn + 150));
        }
        int top = 262, bottom = 125, side = 20;   // top: room for the text. bottom: room for the two RPM bars.
        double scale = Math.min((getWidth() - 2 * side) / (maxX - minX), (getHeight() - top - bottom) / maxZ);

        AffineTransform screen = g.getTransform();
        g.translate(side - minX * scale, getHeight() - bottom);
        g.scale(scale, -scale);

        drawGround(g, minX, maxX);
        drawHive(g, hive);
        drawChassis(g);
        for (Turret t : turrets) drawTurret(g, t);

        g.setTransform(screen);
        drawText(g);
        for (int i = 0; i < turrets.size(); i++) {
            drawRpmBar(g, turrets.get(i), i, side, getHeight() - 108 + i * 46, getWidth() - 2 * side);
        }
        g.dispose();
    }

    private void drawGround(Graphics2D g, double minX, double maxX) {
        g.setColor(new Color(120, 120, 128));
        g.fill(new Rectangle2D.Double(minX, -3, maxX - minX, 3));
        g.setColor(new Color(90, 90, 98));
        g.setStroke(new BasicStroke(0.3f));
        for (double x = 0; x < maxX; x += 24) g.draw(new Line2D.Double(x, -3, x, 0)); // 2 ft tiles
    }

    private void drawHive(Graphics2D g, ShotPhysics.Settings s) {
        double tilt = Math.toRadians(s.openingTiltDeg);
        double ux = Math.sin(tilt), uz = Math.cos(tilt);
        double nx = Math.cos(tilt), nz = -Math.sin(tilt);
        double len = (s.openingTopIn - s.openingBottomIn) / Math.cos(tilt);
        double below = (s.openingBottomIn - s.hiveBottomIn) / Math.cos(tilt);
        double bx = s.distanceIn, bz = s.openingBottomIn;
        double tx = bx + ux * len, tz = bz + uz * len;
        double d = s.cellDepthIn;

        // Legs (A-frame) from the bottom of the HIVE down to the tiles.
        double legTopX = bx - ux * below + nx * d / 2, legTopZ = s.hiveBottomIn;
        g.setColor(new Color(170, 170, 180));
        g.setStroke(new BasicStroke(0.8f));
        g.draw(new Line2D.Double(legTopX, legTopZ, legTopX - 14, 0));
        g.draw(new Line2D.Double(legTopX, legTopZ, legTopX + 14, 0));

        // HIVE body below the opening.
        Path2D.Double body = new Path2D.Double();
        body.moveTo(bx, bz);
        body.lineTo(bx - ux * below, bz - uz * below);
        body.lineTo(bx - ux * below + nx * d, bz - uz * below + nz * d);
        body.lineTo(bx + nx * d, bz + nz * d);
        body.closePath();
        g.setColor(new Color(150, 40, 40));
        g.fill(body);

        // The CELL behind the opening.
        Path2D.Double cell = new Path2D.Double();
        cell.moveTo(bx, bz);
        cell.lineTo(bx + nx * d, bz + nz * d);
        cell.lineTo(tx + nx * d, tz + nz * d);
        cell.lineTo(tx, tz);
        g.setColor(new Color(200, 200, 210, 70));
        g.fill(cell);
        g.setColor(new Color(220, 220, 230));
        g.setStroke(new BasicStroke(0.7f));
        g.draw(cell);

        // The opening itself (green), with rim dots.
        g.setColor(new Color(60, 220, 100));
        g.setStroke(new BasicStroke(0.8f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10f, new float[]{1.5f, 1f}, 0f));
        g.draw(new Line2D.Double(bx, bz, tx, tz));
        g.fill(new Ellipse2D.Double(bx - 0.8, bz - 0.8, 1.6, 1.6));
        g.fill(new Ellipse2D.Double(tx - 0.8, tz - 0.8, 1.6, 1.6));
    }

    /** The robot's chassis: one box under both turrets. */
    private void drawChassis(Graphics2D g) {
        double frontX = 0;
        for (Turret t : turrets) frontX = Math.max(frontX, t.originXIn);
        double h = turrets.get(0).settings.launchHeightIn;
        g.setColor(new Color(70, 75, 90));
        g.fill(new Rectangle2D.Double(-18, 1.5, 18 + frontX + 6, Math.max(3, Math.min(8, h - 4))));
    }

    /** Draws one turret and its ball, in its own frame (its launch point at x = 0). */
    private void drawTurret(Graphics2D g, Turret t) {
        AffineTransform before = g.getTransform();
        g.translate(t.originXIn, 0);
        drawShooter(g, t);
        drawOddsDots(g, t);
        drawPath(g, t);
        drawBall(g, t);
        g.setTransform(before);
    }

    private void drawShooter(Graphics2D g, Turret t) {
        ShotPhysics.Settings s = t.settings;
        double h = s.launchHeightIn;
        double a = Math.toRadians(s.launchAngleDeg);
        double ballR = s.ball.diameterIn / 2, wheelR = s.flywheelDiameterIn / 2;

        // Flywheel sits beside the ball's path: under it for backspin, over it for topspin.
        double side = s.spin == ShotPhysics.Spin.TOPSPIN ? 1 : -1;
        double px = -Math.sin(a) * side, pz = Math.cos(a) * side;      // sideways to the shot
        double back = wheelR + 1;                                        // wheel sits a bit behind the exit
        double cx = -Math.cos(a) * back + px * (ballR + wheelR * 0.9);
        double cz = h - Math.sin(a) * back + pz * (ballR + wheelR * 0.9);

        // Hood: a plate on the other side of the ball.
        g.setColor(new Color(160, 165, 180));
        g.setStroke(new BasicStroke(0.8f));
        double hx = -px * (ballR + 0.5), hz = -pz * (ballR + 0.5);
        g.draw(new Line2D.Double(hx - Math.cos(a) * 8, h + hz - Math.sin(a) * 8, hx, h + hz));

        // Flywheel with spokes, turning at the RPM (slowed down so you can see it).
        g.setColor(new Color(90, 200, 255));
        g.fill(new Ellipse2D.Double(cx - wheelR, cz - wheelR, 2 * wheelR, 2 * wheelR));
        g.setColor(new Color(20, 40, 60));
        g.setStroke(new BasicStroke(0.35f));
        for (int i = 0; i < 4; i++) {
            double sa = t.flywheelAngle + i * Math.PI / 4;
            g.draw(new Line2D.Double(cx - Math.cos(sa) * wheelR, cz - Math.sin(sa) * wheelR,
                    cx + Math.cos(sa) * wheelR, cz + Math.sin(sa) * wheelR));
        }
    }

    private void drawPath(Graphics2D g, Turret t) {
        if (t.result.path.size() < 2) return;
        Path2D.Double p = new Path2D.Double();
        p.moveTo(t.result.path.get(0)[1], t.result.path.get(0)[2]);
        for (double[] pt : t.result.path) p.lineTo(pt[1], pt[2]);
        g.setColor(new Color(255, 255, 255, 120));
        g.setStroke(new BasicStroke(0.4f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_ROUND, 10f, new float[]{1.5f, 1.2f}, 0f));
        g.draw(p);
    }

    /** One small dot per practice shot, where it reached the HIVE (or the floor). */
    private void drawOddsDots(Graphics2D g, Turret t) {
        if (t.odds == null) return;
        for (double[] h : t.odds.hits) {
            ShotPhysics.Outcome o = ShotPhysics.Outcome.values()[(int) h[2]];
            g.setColor(o == ShotPhysics.Outcome.SCORED ? new Color(80, 230, 110, 170)
                    : o == ShotPhysics.Outcome.BOUNCED_OUT || o == ShotPhysics.Outcome.RIM ? new Color(255, 170, 50, 170)
                    : new Color(255, 90, 90, 170));
            g.fill(new Ellipse2D.Double(h[0] - 0.5, h[1] - 0.5, 1, 1));
        }
    }

    private void drawBall(Graphics2D g, Turret t) {
        double[] pt = t.result.path.get(t.result.path.size() - 1);
        if (animTime >= 0) {
            for (double[] candidate : t.result.path) {
                pt = candidate;
                if (candidate[0] >= animTime) break;
            }
        }
        double r = t.settings.ball.diameterIn / 2;
        g.setColor(t.settings.ball.color);
        g.fill(new Ellipse2D.Double(pt[1] - r, pt[2] - r, 2 * r, 2 * r));
        // A line across the ball so you can see it spin.
        g.setColor(Color.BLACK);
        g.setStroke(new BasicStroke(0.3f));
        double sa = pt[3];
        g.draw(new Line2D.Double(pt[1] - Math.cos(sa) * r, pt[2] + Math.sin(sa) * r,
                pt[1] + Math.cos(sa) * r, pt[2] - Math.sin(sa) * r));
    }

    /** Text: one column per turret, so both turrets' numbers are on screen at once. */
    private void drawText(Graphics2D g) {
        int columnWidth = getWidth() / turrets.size();
        for (int i = 0; i < turrets.size(); i++) {
            drawTurretText(g, i, turrets.get(i), 20 + i * columnWidth);
        }
    }

    private void drawTurretText(Graphics2D g, int index, Turret t, int x) {
        ShotPhysics.Settings s = t.settings;
        ShotPhysics.Result r = t.result;
        Font bold = new Font(Font.SANS_SERIF, Font.BOLD, 13);
        Font mono = new Font(Font.MONOSPACED, Font.PLAIN, 12);

        int y = 30;
        g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 16));
        g.setColor(s.ball.color);
        g.drawString((index == 0 ? "BACK turret" : "FRONT turret") + "  -  " + s.ball.label, x, y);

        g.setFont(bold);
        g.setColor(outcomeColor(r.outcome));
        g.drawString(r.message, x, y += 22);

        if (t.odds != null) {
            // The big number: out of many slightly-off shots, how many score and stay in?
            double chance = t.odds.chance();
            g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 15));
            g.setColor(chance >= 80 ? new Color(80, 230, 110) : chance >= 50 ? new Color(255, 190, 60) : new Color(255, 100, 100));
            g.drawString(String.format("Chance to score AND stay in: %.0f%%", chance), x, y += 24);
            g.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 11));
            g.setColor(new Color(200, 200, 210));
            g.drawString(String.format("(%d practice shots: %d stayed in, %d bounced, %d rim, %d missed)",
                    t.odds.shots, t.odds.stayed, t.odds.bouncedOut, t.odds.rim, t.odds.missed), x, y += 17);
        }

        g.setFont(mono);
        g.setColor(new Color(150, 200, 255));
        g.drawString(String.format("dist %.0f in   angle %.1f deg   height %.1f in", s.distanceIn, s.launchAngleDeg, s.launchHeightIn), x, y += 20);
        g.drawString(String.format("squeeze %.0f%%   %s%s", s.compressionPct, s.spin.label.toLowerCase(),
                s.airEffects ? "" : "   no air"), x, y += 18);

        g.setColor(new Color(225, 225, 230));
        g.drawString(String.format("Flywheel %5.0f RPM   surface %5.1f ft/s   grip %3.0f%%",
                s.flywheelRpm, r.surfaceSpeedInPerSec / 12, r.grip * 100), x, y += 18);
        g.drawString(String.format("Ball exit %5.1f ft/s   spin %5.0f RPM", r.exitSpeedInPerSec / 12, r.ballSpinRpm), x, y += 18);
        g.drawString(String.format("Highest %5.1f in   flight %4.2f s", r.apexIn, r.flightTime), x, y += 18);
        if (r.outcome == ShotPhysics.Outcome.SCORED || r.outcome == ShotPhysics.Outcome.BOUNCED_OUT
                || r.outcome == ShotPhysics.Outcome.RIM) {
            g.drawString(String.format("At the HIVE: ball is %s, coming in %4.1f deg %s",
                    r.risingAtTarget ? "still RISING" : "falling",
                    Math.abs(r.entryAngleDeg), r.entryAngleDeg >= 0 ? "downward" : "upward"), x, y += 18);
        }
        String window = t.scoringRanges == null || t.scoringRanges.isEmpty()
                ? "No flywheel speed scores with these settings."
                : "Scores at: " + describe(t.scoringRanges);
        g.drawString(window, x, y + 18);
    }

    private static Color outcomeColor(ShotPhysics.Outcome o) {
        return o == ShotPhysics.Outcome.SCORED ? new Color(80, 230, 110)
                : o == ShotPhysics.Outcome.RIM || o == ShotPhysics.Outcome.BOUNCED_OUT ? new Color(255, 190, 60)
                : new Color(255, 100, 100);
    }

    private static String describe(List<double[]> ranges) {
        StringBuilder sb = new StringBuilder();
        for (double[] range : ranges) {
            if (sb.length() > 0) sb.append(",  ");
            sb.append(String.format("%.0f-%.0f RPM", range[0], range[1]));
        }
        return sb.toString();
    }

    /** A bar from 0 to 6000 RPM for one turret: green where its shot scores, white line at its current speed. */
    private void drawRpmBar(Graphics2D g, Turret t, int index, int x, int y, int width) {
        int height = 14;
        g.setColor(new Color(60, 62, 72));
        g.fillRect(x, y, width, height);
        if (t.scoringRanges != null) {
            g.setColor(new Color(60, 200, 90));
            for (double[] range : t.scoringRanges) {
                int x1 = x + (int) (range[0] / RPM_BAR_MAX * width);
                int x2 = x + (int) (range[1] / RPM_BAR_MAX * width);
                g.fillRect(x1, y, Math.max(2, x2 - x1), height);
            }
        }
        int cur = x + (int) (Math.min(t.settings.flywheelRpm, RPM_BAR_MAX) / RPM_BAR_MAX * width);
        g.setColor(Color.WHITE);
        g.fillRect(cur - 1, y - 4, 3, height + 8);
        g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 11));
        g.setColor(t.settings.ball.color);
        g.drawString((index == 0 ? "Back " : "Front ") + t.settings.ball.label
                + " flywheel speed - green = scores (0 to " + (int) RPM_BAR_MAX + " RPM)", x, y + height + 13);
    }
}
