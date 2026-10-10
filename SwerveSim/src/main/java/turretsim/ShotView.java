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
 * Draws the shot from the side: shooter on the left, HIVE on the right.
 * Like SwerveSim's FieldView, we draw in INCHES with +z (height) pointing up the screen.
 */
public class ShotView extends JPanel {
    public static final double RPM_BAR_MAX = 6000;

    private final ShotPhysics.Settings settings;
    private ShotPhysics.Result result;
    private List<double[]> scoringRanges;
    private ShotPhysics.Odds odds;
    private double animTime = -1;      // -1 = not animating, show the ball at the end of the shot
    private double flywheelAngle = 0;

    public ShotView(ShotPhysics.Settings settings) {
        this.settings = settings;
        setBackground(new Color(30, 32, 40));
    }

    public void setResult(ShotPhysics.Result result, List<double[]> scoringRanges, ShotPhysics.Odds odds) {
        this.result = result;
        this.scoringRanges = scoringRanges;
        this.odds = odds;
    }

    public void setAnimTime(double t) { animTime = t; }
    public void setFlywheelAngle(double a) { flywheelAngle = a; }

    @Override
    protected void paintComponent(Graphics graphics) {
        super.paintComponent(graphics);
        if (result == null) return;
        Graphics2D g = (Graphics2D) graphics.create();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // Pick a zoom so the shooter, the HIVE, and the whole flight path fit.
        double minX = -20, maxX = settings.distanceIn + settings.cellDepthIn + 30;
        double maxZ = Math.max(settings.openingTopIn + 20, result.apexIn + 10);
        for (double[] p : result.path) maxX = Math.max(maxX, Math.min(p[1] + 6, settings.distanceIn + 150));
        int top = 215, bottom = 70, side = 20;
        double scale = Math.min((getWidth() - 2 * side) / (maxX - minX), (getHeight() - top - bottom) / maxZ);

        AffineTransform screen = g.getTransform();
        g.translate(side - minX * scale, getHeight() - bottom);
        g.scale(scale, -scale);

        drawGround(g, minX, maxX);
        drawHive(g);
        drawShooter(g);
        drawOddsDots(g);
        drawPath(g);
        drawBall(g);

        g.setTransform(screen);
        drawText(g);
        drawRpmBar(g, side, getHeight() - 45, getWidth() - 2 * side);
        g.dispose();
    }

    private void drawGround(Graphics2D g, double minX, double maxX) {
        g.setColor(new Color(120, 120, 128));
        g.fill(new Rectangle2D.Double(minX, -3, maxX - minX, 3));
        g.setColor(new Color(90, 90, 98));
        g.setStroke(new BasicStroke(0.3f));
        for (double x = 0; x < maxX; x += 24) g.draw(new Line2D.Double(x, -3, x, 0)); // 2 ft tiles
    }

    private void drawHive(Graphics2D g) {
        ShotPhysics.Settings s = settings;
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

    private void drawShooter(Graphics2D g) {
        ShotPhysics.Settings s = settings;
        double h = s.launchHeightIn;
        double a = Math.toRadians(s.launchAngleDeg);
        double ballR = s.ball.diameterIn / 2, wheelR = s.flywheelDiameterIn / 2;

        // Robot chassis (just a box under the shooter).
        g.setColor(new Color(70, 75, 90));
        g.fill(new Rectangle2D.Double(-18, 1.5, 18, Math.max(3, Math.min(8, h - 4))));

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
            double sa = flywheelAngle + i * Math.PI / 4;
            g.draw(new Line2D.Double(cx - Math.cos(sa) * wheelR, cz - Math.sin(sa) * wheelR,
                    cx + Math.cos(sa) * wheelR, cz + Math.sin(sa) * wheelR));
        }
    }

    private void drawPath(Graphics2D g) {
        if (result.path.size() < 2) return;
        Path2D.Double p = new Path2D.Double();
        p.moveTo(result.path.get(0)[1], result.path.get(0)[2]);
        for (double[] pt : result.path) p.lineTo(pt[1], pt[2]);
        g.setColor(new Color(255, 255, 255, 120));
        g.setStroke(new BasicStroke(0.4f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_ROUND, 10f, new float[]{1.5f, 1.2f}, 0f));
        g.draw(p);
    }

    /** One small dot per practice shot, where it reached the HIVE (or the floor). */
    private void drawOddsDots(Graphics2D g) {
        if (odds == null) return;
        for (double[] h : odds.hits) {
            ShotPhysics.Outcome o = ShotPhysics.Outcome.values()[(int) h[2]];
            g.setColor(o == ShotPhysics.Outcome.SCORED ? new Color(80, 230, 110, 170)
                    : o == ShotPhysics.Outcome.BOUNCED_OUT || o == ShotPhysics.Outcome.RIM ? new Color(255, 170, 50, 170)
                    : new Color(255, 90, 90, 170));
            g.fill(new Ellipse2D.Double(h[0] - 0.5, h[1] - 0.5, 1, 1));
        }
    }

    private void drawBall(Graphics2D g) {
        double[] pt = result.path.get(result.path.size() - 1);
        if (animTime >= 0) {
            for (double[] candidate : result.path) {
                pt = candidate;
                if (candidate[0] >= animTime) break;
            }
        }
        double r = settings.ball.diameterIn / 2;
        g.setColor(settings.ball.color);
        g.fill(new Ellipse2D.Double(pt[1] - r, pt[2] - r, 2 * r, 2 * r));
        // A line across the ball so you can see it spin.
        g.setColor(Color.BLACK);
        g.setStroke(new BasicStroke(0.3f));
        double sa = pt[3];
        g.draw(new Line2D.Double(pt[1] - Math.cos(sa) * r, pt[2] + Math.sin(sa) * r,
                pt[1] + Math.cos(sa) * r, pt[2] - Math.sin(sa) * r));
    }

    private void drawText(Graphics2D g) {
        ShotPhysics.Result r = result;
        Color outcomeColor = r.outcome == ShotPhysics.Outcome.SCORED ? new Color(80, 230, 110)
                : r.outcome == ShotPhysics.Outcome.RIM || r.outcome == ShotPhysics.Outcome.BOUNCED_OUT ? new Color(255, 190, 60) : new Color(255, 100, 100);
        g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 22));
        g.setColor(outcomeColor);
        g.drawString(r.message, 20, 32);

        int y = 58;
        if (odds != null) {
            // The big number: out of many slightly-off shots, how many score and stay in?
            double chance = odds.chance();
            g.setColor(chance >= 80 ? new Color(80, 230, 110) : chance >= 50 ? new Color(255, 190, 60) : new Color(255, 100, 100));
            g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 18));
            g.drawString(String.format("Chance to score AND stay in: %.0f%%", chance), 20, y);
            g.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 13));
            g.setColor(new Color(200, 200, 210));
            g.drawString(String.format("(%d practice shots with wobble:  %d stayed in,  %d bounced out,  %d hit the rim,  %d missed)",
                    odds.shots, odds.stayed, odds.bouncedOut, odds.rim, odds.missed), 20, y += 20);
            y += 24;
        }

        g.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 13));

        // Summary of the main settings, so you always know what you're looking at.
        ShotPhysics.Settings s = settings;
        g.setColor(new Color(150, 200, 255));
        g.drawString(String.format("SETTINGS  %s  |  distance %.0f in  |  angle %.1f deg  |  height %.1f in  |  %.0f%% squeeze  |  %s%s",
                s.ball.label, s.distanceIn, s.launchAngleDeg, s.launchHeightIn, s.compressionPct,
                s.spin.label.toLowerCase(), s.airEffects ? "" : "  |  no air"), 20, y);
        y += 18;

        g.setColor(new Color(225, 225, 230));
        g.drawString(String.format("Flywheel  %5.0f RPM   surface speed %5.1f ft/s   grip %3.0f%%",
                settings.flywheelRpm, r.surfaceSpeedInPerSec / 12, r.grip * 100), 20, y);
        g.drawString(String.format("Ball exit %5.1f ft/s   ball spin %5.0f RPM %s",
                r.exitSpeedInPerSec / 12, r.ballSpinRpm, settings.spin == ShotPhysics.Spin.NONE ? "" : settings.spin.name().toLowerCase()), 20, y += 18);
        g.drawString(String.format("Highest point %5.1f in   flight time %4.2f s", r.apexIn, r.flightTime), 20, y += 18);
        if (r.outcome == ShotPhysics.Outcome.SCORED || r.outcome == ShotPhysics.Outcome.BOUNCED_OUT
                || r.outcome == ShotPhysics.Outcome.RIM) {
            g.drawString(String.format("At the HIVE: ball is %s, coming in %4.1f deg %s",
                    r.risingAtTarget ? "still RISING" : "falling",
                    Math.abs(r.entryAngleDeg), r.entryAngleDeg >= 0 ? "downward" : "upward"), 20, y += 18);
        }
        String window = scoringRanges == null || scoringRanges.isEmpty()
                ? "No flywheel speed scores with these settings - change the angle or distance."
                : "Speeds that score and stay in (exact shot, no wobble): " + describe(scoringRanges);
        g.drawString(window, 20, y + 18);
    }

    private static String describe(List<double[]> ranges) {
        StringBuilder sb = new StringBuilder();
        for (double[] range : ranges) {
            if (sb.length() > 0) sb.append(",  ");
            sb.append(String.format("%.0f-%.0f RPM", range[0], range[1]));
        }
        return sb.toString();
    }

    /** A bar from 0 to 6000 RPM: green where the shot scores, white line at the current speed. */
    private void drawRpmBar(Graphics2D g, int x, int y, int width) {
        int height = 14;
        g.setColor(new Color(60, 62, 72));
        g.fillRect(x, y, width, height);
        if (scoringRanges != null) {
            g.setColor(new Color(60, 200, 90));
            for (double[] range : scoringRanges) {
                int x1 = x + (int) (range[0] / RPM_BAR_MAX * width);
                int x2 = x + (int) (range[1] / RPM_BAR_MAX * width);
                g.fillRect(x1, y, Math.max(2, x2 - x1), height);
            }
        }
        int cur = x + (int) (Math.min(settings.flywheelRpm, RPM_BAR_MAX) / RPM_BAR_MAX * width);
        g.setColor(Color.WHITE);
        g.fillRect(cur - 1, y - 4, 3, height + 8);
        g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 11));
        g.setColor(new Color(200, 200, 205));
        g.drawString("0 RPM", x, y + height + 13);
        g.drawString("Flywheel speed - green = scores", x + width / 2 - 80, y + height + 13);
        g.drawString((int) RPM_BAR_MAX + " RPM", x + width - 60, y + height + 13);
    }
}
