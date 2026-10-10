package turretsim;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSpinner;
import javax.swing.KeyStroke;
import javax.swing.SpinnerNumberModel;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import javax.swing.AbstractAction;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.util.function.DoubleConsumer;

/*
 * TurretSim - try out shooter settings on a laptop before testing on the robot.
 *
 * Change any number on the right and the dashed path updates right away.
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

    private final ShotPhysics.Settings settings = new ShotPhysics.Settings();
    private final ShotView view = new ShotView(settings);
    private final JPanel controls = new JPanel(new GridBagLayout());
    private int row = 0;
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
        buildControls();

        JFrame frame = new JFrame("TurretSim - Chargers Biobuzz");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLayout(new BorderLayout());
        frame.add(view, BorderLayout.CENTER);
        JScrollPane scroll = new JScrollPane(controls);
        scroll.setPreferredSize(new Dimension(360, 760));
        scroll.setBorder(null);
        frame.add(scroll, BorderLayout.EAST);
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

    private void buildControls() {
        controls.setBorder(BorderFactory.createEmptyBorder(10, 12, 10, 12));

        JButton shootButton = new JButton("Shoot!  (Space)");
        shootButton.setFont(shootButton.getFont().deriveFont(Font.BOLD, 16f));
        shootButton.addActionListener(e -> shoot());
        addWide(shootButton);
        slowMotion = new JCheckBox("Slow motion (4x slower)");
        addWide(slowMotion);

        header("BALL");
        JComboBox<ShotPhysics.Ball> ballBox = new JComboBox<>(ShotPhysics.Ball.values());
        ballBox.addActionListener(e -> {
            settings.ball = (ShotPhysics.Ball) ballBox.getSelectedItem();
            massSpinner.setValue(settings.ball.defaultMassGrams); // triggers recalculate()
            recalculate();
        });
        addRow("Ball", ballBox);
        massSpinner = number("Mass (g) - GUESS, weigh it!", settings.ballMassGrams, 1, 500, 1, v -> settings.ballMassGrams = v);

        header("SHOOTER");
        number("Distance to HIVE (in)", settings.distanceIn, 0, 300, 1, v -> settings.distanceIn = v);
        number("Launch angle (deg)", settings.launchAngleDeg, 0, 89, 0.5, v -> settings.launchAngleDeg = v);
        number("Launch height (in)", settings.launchHeightIn, 2, 40, 0.5, v -> settings.launchHeightIn = v);
        number("Flywheel speed (RPM)", settings.flywheelRpm, 0, 6000, 25, v -> settings.flywheelRpm = v);
        number("Flywheel diameter (in)", settings.flywheelDiameterIn, 1, 8, 0.1, v -> settings.flywheelDiameterIn = v);
        number("Compression (% of ball)", settings.compressionPct, 0, 60, 1, v -> settings.compressionPct = v);
        JComboBox<ShotPhysics.Spin> spinBox = new JComboBox<>(ShotPhysics.Spin.values());
        spinBox.addActionListener(e -> {
            settings.spin = (ShotPhysics.Spin) spinBox.getSelectedItem();
            recalculate();
        });
        addRow("Spin", spinBox);
        JCheckBox air = new JCheckBox("Air drag + spin curve", settings.airEffects);
        air.addActionListener(e -> {
            settings.airEffects = air.isSelected();
            recalculate();
        });
        addWide(air);

        header("HIVE  (game manual Fig. 9-10)");
        number("Opening bottom height (in)", settings.openingBottomIn, 0, 120, 0.1, v -> settings.openingBottomIn = v);
        number("Opening top height (in)", settings.openingTopIn, 0, 130, 0.1, v -> settings.openingTopIn = v);
        number("Opening tilt (deg)", settings.openingTiltDeg, -60, 60, 1, v -> settings.openingTiltDeg = v);
        number("Cell depth (in)", settings.cellDepthIn, 2, 30, 0.5, v -> settings.cellDepthIn = v);
        number("HIVE bottom height (in)", settings.hiveBottomIn, 0, 120, 0.1, v -> settings.hiveBottomIn = v);

        JLabel help = new JLabel("<html><br>Tilt: + leans the top of the opening away from the shooter."
                + "<br><br>Distance is measured on the floor from where the ball leaves the shooter"
                + " to the bottom edge of the opening.</html>");
        help.setForeground(Color.GRAY);
        addWide(help);

        // Push everything to the top.
        GridBagConstraints filler = new GridBagConstraints();
        filler.gridy = row++;
        filler.weighty = 1;
        controls.add(new JLabel(), filler);
    }

    // ---------- small helpers to lay out the control panel ----------

    private JSpinner number(String label, double value, double min, double max, double step, DoubleConsumer setter) {
        JSpinner spinner = new JSpinner(new SpinnerNumberModel(value, min, max, step));
        spinner.addChangeListener(e -> {
            setter.accept(((Number) spinner.getValue()).doubleValue());
            recalculate();
        });
        addRow(label, spinner);
        return spinner;
    }

    private void header(String text) {
        JLabel label = new JLabel(text);
        label.setFont(label.getFont().deriveFont(Font.BOLD));
        label.setBorder(BorderFactory.createEmptyBorder(14, 0, 4, 0));
        addWide(label);
    }

    private void addRow(String label, JComponent field) {
        GridBagConstraints c = new GridBagConstraints();
        c.gridy = row++;
        c.insets = new Insets(2, 0, 2, 6);
        c.anchor = GridBagConstraints.WEST;
        controls.add(new JLabel(label), c);
        c.gridx = 1;
        c.fill = GridBagConstraints.HORIZONTAL;
        c.weightx = 1;
        controls.add(field, c);
    }

    private void addWide(JComponent component) {
        GridBagConstraints c = new GridBagConstraints();
        c.gridy = row++;
        c.gridwidth = 2;
        c.insets = new Insets(2, 0, 2, 0);
        c.fill = GridBagConstraints.HORIZONTAL;
        controls.add(component, c);
    }

    // ---------- simulation ----------

    /** Re-run the physics whenever a setting changes. */
    private void recalculate() {
        result = ShotPhysics.simulate(settings);
        view.setResult(result, ShotPhysics.scoringRpmRanges(settings, ShotView.RPM_BAR_MAX, 25));
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
            view.setAnimTime(Math.min(animTime, end));
            if (animTime < 0) view.setAnimTime(-1);
        }
        view.repaint();
    }
}
