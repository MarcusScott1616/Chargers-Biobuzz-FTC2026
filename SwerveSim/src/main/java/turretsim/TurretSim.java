package turretsim;

import javax.swing.AbstractAction;
import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSpinner;
import javax.swing.JTabbedPane;
import javax.swing.KeyStroke;
import javax.swing.Scrollable;
import javax.swing.SpinnerNumberModel;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

/*
 * TurretSim - try out shooter settings on a laptop before testing on the robot.
 *
 * Change any number in the tabs on the right and the dashed path updates right away.
 * Press "Shoot!" (or Space) to watch the ball fly.
 * The bar at the bottom shows every flywheel speed that would score with the other settings.
 *
 * Run it: Gradle panel -> SwerveSim -> Tasks -> application -> runTurret
 *     or: ./gradlew :SwerveSim:runTurret
 *
 * The physics lives in ShotPhysics.java - read the comments there to see what's estimated.
 */
public class TurretSim {
    private static final int TICK_MS = 16;
    private static final int PANEL_WIDTH = 300;
    private static final int PRACTICE_SHOTS = 200;

    private final ShotPhysics.Settings settings = new ShotPhysics.Settings();
    private final ShotView view = new ShotView(settings);
    private JSpinner massSpinner;
    private JCheckBox slowMotion;

    private ShotPhysics.Result result;
    private double animTime = -1;
    private double flywheelAngle = 0;
    private long lastTick = System.nanoTime();

    public static void main(String[] args) {
        SwingUtilities.invokeLater(TurretSim::new);
    }

    private TurretSim() {
        JFrame frame = new JFrame("TurretSim - Chargers Biobuzz");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLayout(new BorderLayout());
        frame.add(view, BorderLayout.CENTER);
        frame.add(buildSidePanel(), BorderLayout.EAST);
        frame.setSize(1300, 800);
        frame.setLocationRelativeTo(null);

        // Space = shoot, from anywhere in the window.
        JComponent root = frame.getRootPane();
        root.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("SPACE"), "shoot");
        root.getActionMap().put("shoot", new AbstractAction() {
            @Override public void actionPerformed(ActionEvent e) { shoot(); }
        });

        recalculate();
        frame.setVisible(true);
        new Timer(TICK_MS, e -> tick()).start();
    }

    /** Shoot button on top, then one tab per group of settings. */
    private JPanel buildSidePanel() {
        JButton shootButton = new JButton("Shoot!  (Space)");
        shootButton.setFont(shootButton.getFont().deriveFont(Font.BOLD, 16f));
        shootButton.addActionListener(e -> shoot());
        slowMotion = new JCheckBox("Slow motion (4x slower)");

        JPanel top = new JPanel(new BorderLayout(0, 4));
        top.setBorder(BorderFactory.createEmptyBorder(10, 10, 6, 10));
        top.add(shootButton, BorderLayout.CENTER);
        top.add(slowMotion, BorderLayout.SOUTH);

        JTabbedPane tabs = new JTabbedPane();
        tabs.addTab("Shooter", scroll(buildShooterTab()));
        tabs.addTab("Ball", scroll(buildBallTab()));
        tabs.addTab("HIVE", scroll(buildHiveTab()));
        tabs.addTab("Odds", scroll(buildOddsTab()));

        JPanel side = new JPanel(new BorderLayout());
        side.setPreferredSize(new Dimension(PANEL_WIDTH, 760));
        side.add(top, BorderLayout.NORTH);
        side.add(tabs, BorderLayout.CENTER);
        return side;
    }

    private Form buildShooterTab() {
        Form f = new Form();
        f.number("Distance to HIVE (in)", settings.distanceIn, 0, 300, 1, v -> settings.distanceIn = v);
        f.number("Launch angle (deg)  0 = flat, 90 = straight up", settings.launchAngleDeg, 0, 89, 0.5, v -> settings.launchAngleDeg = v);
        f.number("Launch height (in)", settings.launchHeightIn, 2, 40, 0.5, v -> settings.launchHeightIn = v);
        f.number("Flywheel speed (RPM)", settings.flywheelRpm, 0, 6000, 25, v -> settings.flywheelRpm = v);
        f.number("Flywheel diameter (in)", settings.flywheelDiameterIn, 1, 8, 0.1, v -> settings.flywheelDiameterIn = v);
        f.number("Compression (% of ball diameter)", settings.compressionPct, 0, 60, 1, v -> settings.compressionPct = v);
        f.combo("Spin", ShotPhysics.Spin.values(), settings.spin, v -> settings.spin = v);
        f.note("Backspin = wheel under the ball, hood on top. Topspin = wheel over the ball, hood underneath.");
        f.check("Air drag + spin curve", settings.airEffects, v -> settings.airEffects = v);
        f.note("Distance is measured on the floor from where the ball leaves the shooter to the bottom edge of the opening.");
        return f;
    }

    private Form buildBallTab() {
        Form f = new Form();
        f.combo("Ball", ShotPhysics.Ball.values(), settings.ball, v -> {
            settings.ball = v;
            settings.ballMassGrams = v.defaultMassGrams;
            massSpinner.setValue(v.defaultMassGrams);
        });
        massSpinner = f.number("Mass (grams)", settings.ballMassGrams, 1, 500, 1, v -> settings.ballMassGrams = v);
        f.note("The masses are GUESSES. Weigh the real Nectar and Pollen and type the real numbers in.");
        return f;
    }

    private Form buildHiveTab() {
        Form f = new Form();
        f.number("Opening bottom height (in)", settings.openingBottomIn, 0, 120, 0.1, v -> settings.openingBottomIn = v);
        f.number("Opening top height (in)", settings.openingTopIn, 0, 130, 0.1, v -> settings.openingTopIn = v);
        f.number("Opening tilt (deg)", settings.openingTiltDeg, -60, 60, 1, v -> settings.openingTiltDeg = v);
        f.note("Tilt + leans the top of the opening away from the shooter.");
        f.number("Cell depth (in)", settings.cellDepthIn, 2, 30, 0.5, v -> settings.cellDepthIn = v);
        f.number("HIVE bottom height (in)", settings.hiveBottomIn, 0, 120, 0.1, v -> settings.hiveBottomIn = v);
        f.number("Bounciness (0 = dead, 1 = superball)", settings.bounciness, 0, 1, 0.05, v -> settings.bounciness = v);
        f.note("Bounciness is a GUESS. Drop a ball on the HIVE from 24 in: if it bounces back 6 in high, bounciness = square root of (6 / 24) = 0.5.");
        f.note("Defaults come from the game manual, Figure 9-10.");
        return f;
    }

    private Form buildOddsTab() {
        Form f = new Form();
        f.note("No robot shoots exactly the same twice. The sim fires " + PRACTICE_SHOTS
                + " practice shots, each a bit off by these amounts, and counts how many score AND stay in.");
        f.number("Flywheel wobble (+/- RPM)", settings.rpmWobble, 0, 1000, 5, v -> settings.rpmWobble = v);
        f.number("Angle wobble (+/- deg)", settings.angleWobbleDeg, 0, 10, 0.1, v -> settings.angleWobbleDeg = v);
        f.number("Distance wobble (+/- in)", settings.distanceWobbleIn, 0, 24, 0.5, v -> settings.distanceWobbleIn = v);
        f.note("About 2 out of 3 shots land inside the +/- amount. Measure yours: log the flywheel RPM over 10 real shots and see how much it changes.");
        f.note("Dots on the HIVE: green = stayed in, orange = bounced out or hit the rim, red = missed.");
        return f;
    }

    private static JScrollPane scroll(Form form) {
        JScrollPane scroll = new JScrollPane(form, JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
        scroll.setBorder(null);
        scroll.getVerticalScrollBar().setUnitIncrement(16);
        return scroll;
    }

    /**
     * One tab's worth of settings. Each setting is a label with its box right underneath,
     * so the name of a number never scrolls out of view. It always fits the tab's width
     * (no sideways scrolling).
     */
    private class Form extends JPanel implements Scrollable {
        Form() {
            setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
            setBorder(BorderFactory.createEmptyBorder(8, 10, 8, 10));
        }

        JSpinner number(String label, double value, double min, double max, double step, DoubleConsumer setter) {
            JSpinner spinner = new JSpinner(new SpinnerNumberModel(value, min, max, step));
            spinner.addChangeListener(e -> {
                setter.accept(((Number) spinner.getValue()).doubleValue());
                recalculate();
            });
            field(label, spinner);
            return spinner;
        }

        <T> void combo(String label, T[] options, T selected, Consumer<T> setter) {
            JComboBox<T> box = new JComboBox<>(options);
            box.setSelectedItem(selected);
            box.addActionListener(e -> {
                @SuppressWarnings("unchecked") T value = (T) box.getSelectedItem();
                setter.accept(value);
                recalculate();
            });
            field(label, box);
        }

        void check(String label, boolean value, Consumer<Boolean> setter) {
            JCheckBox box = new JCheckBox(label, value);
            box.addActionListener(e -> {
                setter.accept(box.isSelected());
                recalculate();
            });
            add(box, 6);
        }

        void note(String text) {
            JLabel label = new JLabel("<html>" + text + "</html>");
            label.setForeground(Color.GRAY);
            add(label, 8);
        }

        private void field(String label, JComponent input) {
            JLabel name = new JLabel(label);
            name.setFont(name.getFont().deriveFont(Font.BOLD));
            add(name, 8);
            input.setMaximumSize(new Dimension(Integer.MAX_VALUE, input.getPreferredSize().height));
            add(input, 2);
        }

        private void add(JComponent c, int gapAbove) {
            Component gap = Box.createVerticalStrut(gapAbove);
            ((JComponent) gap).setAlignmentX(Component.LEFT_ALIGNMENT);
            add(gap);
            c.setAlignmentX(Component.LEFT_ALIGNMENT);
            add(c);
        }

        // Scrollable: always match the tab's width, scroll only up and down.
        @Override public boolean getScrollableTracksViewportWidth() { return true; }
        @Override public boolean getScrollableTracksViewportHeight() { return false; }
        @Override public Dimension getPreferredScrollableViewportSize() { return getPreferredSize(); }
        @Override public int getScrollableUnitIncrement(Rectangle r, int orientation, int direction) { return 16; }
        @Override public int getScrollableBlockIncrement(Rectangle r, int orientation, int direction) { return r.height; }
    }

    // ---------- simulation ----------

    /** Re-run the physics whenever a setting changes. */
    private void recalculate() {
        result = ShotPhysics.simulate(settings);
        view.setResult(result, ShotPhysics.scoringRpmRanges(settings, ShotView.RPM_BAR_MAX, 25),
                ShotPhysics.odds(settings, PRACTICE_SHOTS));
        animTime = -1;
        view.setAnimTime(-1);
        view.repaint();
    }

    private void shoot() {
        animTime = 0;
    }

    private void tick() {
        long now = System.nanoTime();
        double dt = (now - lastTick) / 1e9;
        lastTick = now;

        // Spin the drawn flywheel at its real speed slowed down 50x, so you can see it turning.
        flywheelAngle -= settings.flywheelRpm / 60 * 2 * Math.PI * dt / 50;
        view.setFlywheelAngle(flywheelAngle);

        if (animTime >= 0) {
            animTime += dt * (slowMotion.isSelected() ? 0.25 : 1);
            double end = result.path.get(result.path.size() - 1)[0];
            if (animTime > end + 0.6) animTime = -1;  // hold at the end briefly, then show the final spot
            view.setAnimTime(animTime < 0 ? -1 : Math.min(animTime, end));
        }
        view.repaint();
    }
}
