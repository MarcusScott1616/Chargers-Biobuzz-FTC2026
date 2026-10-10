package swervesim;

import javax.swing.JPanel;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;

/*
 * Draws the field and robot from above.
 *
 * Trick: we set up the drawing so 1 unit = 1 inch and +y points UP the screen.
 * Then everything can be drawn in inches using the same directions as Physics.
 */
public class FieldView extends JPanel {
    private static final double WHEEL_ARROW_LENGTH = 14; // inches of arrow at motor power 1.0

    private final Physics physics;
    private final double robotLength, robotWidth;

    public FieldView(Physics physics, double robotLength, double robotWidth) {
        this.physics = physics;
        this.robotLength = robotLength;
        this.robotWidth = robotWidth;
        setBackground(new Color(40, 40, 46));
    }

    @Override
    protected void paintComponent(Graphics graphics) {
        super.paintComponent(graphics);
        Graphics2D g = (Graphics2D) graphics.create();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        int margin = 20;
        double size = Math.min(getWidth(), getHeight()) - 2 * margin;
        double scale = size / Physics.FIELD_SIZE;

        g.setColor(Color.LIGHT_GRAY);
        g.drawString("Top-down field view. Arrows = which way each wheel pushes (longer = more power).", margin, 14);

        // Switch to "inches, +y up" coordinates.
        g.translate(margin, margin + size);
        g.scale(scale, -scale);

        drawField(g);
        drawTrail(g);
        drawRobot(g);
        g.dispose();
    }

    private void drawField(Graphics2D g) {
        g.setColor(new Color(150, 150, 155));
        g.fill(new Rectangle2D.Double(0, 0, Physics.FIELD_SIZE, Physics.FIELD_SIZE));
        g.setColor(new Color(125, 125, 130));
        g.setStroke(new BasicStroke(0.3f));
        for (int i = 24; i < Physics.FIELD_SIZE; i += 24) { // 2 ft foam tiles
            g.draw(new Line2D.Double(i, 0, i, Physics.FIELD_SIZE));
            g.draw(new Line2D.Double(0, i, Physics.FIELD_SIZE, i));
        }
    }

    private void drawTrail(Graphics2D g) {
        if (physics.trail.size() < 2) return;
        Path2D.Double path = new Path2D.Double();
        double[] first = physics.trail.get(0);
        path.moveTo(first[0], first[1]);
        for (double[] p : physics.trail) path.lineTo(p[0], p[1]);
        g.setColor(new Color(255, 255, 255, 140));
        g.setStroke(new BasicStroke(0.4f));
        g.draw(path);
    }

    private void drawRobot(Graphics2D g) {
        AffineTransform fieldTransform = g.getTransform();
        g.translate(physics.x, physics.y);
        g.rotate(physics.heading); // counter-clockwise, same as the IMU

        // Chassis
        g.setColor(new Color(60, 60, 70, 200));
        g.fill(new Rectangle2D.Double(-robotWidth / 2, -robotLength / 2, robotWidth, robotLength));

        // White triangle marks the FRONT of the robot.
        Path2D.Double front = new Path2D.Double();
        front.moveTo(-3, robotLength / 2 - 4);
        front.lineTo(3, robotLength / 2 - 4);
        front.lineTo(0, robotLength / 2 - 0.5);
        front.closePath();
        g.setColor(Color.WHITE);
        g.fill(front);

        for (Physics.Module m : physics.modules) drawModule(g, m);

        // Black arrow from the center = which way the whole robot is sliding.
        g.setColor(Color.BLACK);
        g.setStroke(new BasicStroke(0.6f));
        drawArrow(g, 0, 0, physics.vx / Physics.MAX_WHEEL_SPEED * WHEEL_ARROW_LENGTH,
                physics.vy / Physics.MAX_WHEEL_SPEED * WHEEL_ARROW_LENGTH);

        g.setTransform(fieldTransform);
    }

    private void drawModule(Graphics2D g, Physics.Module m) {
        AffineTransform robotTransform = g.getTransform();
        g.translate(m.x, m.y);

        // Thin dashed line = where SwerveDrive TOLD the wheel to point. The wheel turns to catch up.
        AffineTransform moduleTransform = g.getTransform();
        g.rotate(-m.commandedAngle()); // minus because wheel angles are clockwise-positive
        g.setColor(new Color(255, 255, 255, 160));
        g.setStroke(new BasicStroke(0.25f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10f, new float[]{0.8f, 0.6f}, 0f));
        g.draw(new Line2D.Double(0, -4, 0, 4));
        g.setTransform(moduleTransform);

        // The wheel itself, at the angle it's ACTUALLY pointing right now.
        g.rotate(-m.angle());
        g.setColor(m.color);
        g.fill(new Rectangle2D.Double(-1, -2.5, 2, 5));

        // Arrow = which way this wheel pushes the robot. Negative power points it backwards.
        g.setStroke(new BasicStroke(0.6f));
        drawArrow(g, 0, 0, 0, m.wheelPower() * WHEEL_ARROW_LENGTH);

        g.setTransform(robotTransform);
    }

    private static void drawArrow(Graphics2D g, double x1, double y1, double x2, double y2) {
        double len = Math.hypot(x2 - x1, y2 - y1);
        if (len < 0.3) return;
        g.draw(new Line2D.Double(x1, y1, x2, y2));
        double ux = (x2 - x1) / len, uy = (y2 - y1) / len;
        double head = Math.min(2.5, len * 0.5);
        Path2D.Double tip = new Path2D.Double();
        tip.moveTo(x2, y2);
        tip.lineTo(x2 - ux * head - uy * head * 0.6, y2 - uy * head + ux * head * 0.6);
        tip.lineTo(x2 - ux * head + uy * head * 0.6, y2 - uy * head - ux * head * 0.6);
        tip.closePath();
        g.fill(tip);
    }
}
