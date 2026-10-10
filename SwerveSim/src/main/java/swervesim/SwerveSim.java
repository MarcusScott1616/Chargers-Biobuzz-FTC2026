package swervesim;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.SwerveCode.SwerveDrive;
import org.firstinspires.ftc.teamcode.SwerveCode.SwerveDriveV2;

import javax.swing.BorderFactory;
import javax.swing.JFrame;
import javax.swing.JTextArea;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.HashSet;
import java.util.Set;

/*
 * SwerveSim - drive the REAL SwerveDrive code on your laptop and watch what it does.
 *
 * How it works:
 *   - We create a fake HardwareMap (see SwerveSim/src/main/java/com/qualcomm/...).
 *   - We call the real SwerveDrive.init() and SwerveDrive.move() from TeamCode, exactly
 *     like an OpMode would on the robot.
 *   - The fake servos and motors remember what they were told. Physics reads those
 *     values and moves a pretend robot, and FieldView draws it.
 *
 * So if you change SwerveDrive.java and run this again, you see your change.
 *
 * Run it: Gradle panel -> SwerveSim -> Tasks -> application -> run
 *     or: ./gradlew :SwerveSim:run
 */
public class SwerveSim {
    private static final int TICK_MS = 20; // 50 updates per second, similar to an OpMode loop

    /** The two versions of the drive code we can switch between. Both are the real TeamCode classes. */
    private final SwerveDrive oldSwerve = new SwerveDrive();
    private final SwerveDriveV2 newSwerve = new SwerveDriveV2();
    private boolean useNew = false;

    private final Physics physics;
    private final FieldView fieldView;
    private final JTextArea info = new JTextArea();
    private final Set<Integer> keysDown = new HashSet<>();
    private long lastTickNanos = System.nanoTime();
    private String lastError = null;

    public static void main(String[] args) {
        SwingUtilities.invokeLater(SwerveSim::new);
    }

    private SwerveSim() {
        HardwareMap hardwareMap = new HardwareMap();
        // The real robot code! Both versions share the same fake motors and servos.
        oldSwerve.init(hardwareMap);
        newSwerve.init(hardwareMap);

        physics = new Physics(hardwareMap, oldSwerve.WheelbaseLength, oldSwerve.WheelbaseWidth);
        fieldView = new FieldView(physics, oldSwerve.WheelbaseLength, oldSwerve.WheelbaseWidth);

        info.setEditable(false);
        info.setFocusable(false);
        info.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 13));
        info.setBackground(new Color(28, 28, 32));
        info.setForeground(new Color(225, 225, 225));
        info.setBorder(BorderFactory.createEmptyBorder(12, 12, 12, 12));
        info.setPreferredSize(new Dimension(430, 700));

        JFrame frame = new JFrame("SwerveSim - Chargers Biobuzz");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLayout(new BorderLayout());
        frame.add(fieldView, BorderLayout.CENTER);
        frame.add(info, BorderLayout.EAST);
        frame.setSize(1180, 760);
        frame.setLocationRelativeTo(null);

        // Track which keys are held down. Clear them if the window loses focus so the robot doesn't run away.
        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(e -> {
            if (e.getID() == KeyEvent.KEY_PRESSED) {
                keysDown.add(e.getKeyCode());
                if (e.getKeyCode() == KeyEvent.VK_R) {
                    physics.reset();
                    oldSwerve.resetYaw();
                    newSwerve.resetYaw();
                } else if (e.getKeyCode() == KeyEvent.VK_T) {
                    useNew = !useNew;
                } else if (e.getKeyCode() == KeyEvent.VK_G) {
                    Physics.SERVO_RANGE_DEG = Physics.SERVO_RANGE_DEG == 300 ? 360 : 300;
                }
            } else if (e.getID() == KeyEvent.KEY_RELEASED) {
                keysDown.remove(e.getKeyCode());
            }
            return false;
        });
        frame.addWindowListener(new WindowAdapter() {
            @Override public void windowDeactivated(WindowEvent e) { keysDown.clear(); }
        });

        frame.setVisible(true);
        new Timer(TICK_MS, e -> tick()).start();
    }

    private void tick() {
        long now = System.nanoTime();
        double dt = Math.min(0.05, (now - lastTickNanos) / 1e9);
        lastTickNanos = now;

        // Keyboard acts like the gamepad sticks: full push (1.0) or nothing.
        float strafeX = key(KeyEvent.VK_D) - key(KeyEvent.VK_A);
        float strafeY = key(KeyEvent.VK_W) - key(KeyEvent.VK_S);
        float rotate  = key(KeyEvent.VK_E) - key(KeyEvent.VK_Q);

        // Call the real robot code. If it crashes, show the error instead of closing the window.
        try {
            if (useNew) newSwerve.move(strafeX, strafeY, rotate);
            else oldSwerve.move(strafeX, strafeY, rotate);
            lastError = null;
        } catch (RuntimeException ex) {
            lastError = ex.toString();
        }

        physics.step(dt);
        fieldView.repaint();
        info.setText(buildInfoText(strafeX, strafeY, rotate));
    }

    private float key(int keyCode) {
        return keysDown.contains(keyCode) ? 1f : 0f;
    }

    private String buildInfoText(float strafeX, float strafeY, float rotate) {
        StringBuilder s = new StringBuilder();
        s.append("CONTROLS\n");
        s.append("  W / S   forward / back     (strafeY)\n");
        s.append("  A / D   left / right       (strafeX)\n");
        s.append("  Q / E   rotate -1 / +1     (rotate)\n");
        s.append("  R       reset robot + IMU\n");
        s.append("  T       swap old / new drive code\n");
        s.append("  G       swap servo model 300 / 360 deg\n\n");

        s.append(String.format("CODE    %s%n", useNew ? "SwerveDriveV2 (fixed)" : "SwerveDrive (old)"));
        s.append(String.format("SERVOS  %.0f deg %s%n%n", Physics.SERVO_RANGE_DEG,
                Physics.SERVO_RANGE_DEG == 300 ? "(real goBILDA)" : "(what old code assumes)"));

        s.append("SENT TO move()\n");
        s.append(String.format("  strafeX %+5.2f  strafeY %+5.2f  rotate %+5.2f%n%n", strafeX, strafeY, rotate));

        s.append("ROBOT\n");
        s.append(String.format("  heading      %+7.1f deg  (CCW = +)%n", norm180(Math.toDegrees(physics.heading))));
        s.append(String.format("  sliding fwd  %+7.1f in/s%n", physics.vy));
        s.append(String.format("  sliding rt   %+7.1f in/s%n", physics.vx));
        s.append(String.format("  spinning     %+7.1f deg/s (CCW = +)%n", Math.toDegrees(physics.omega)));
        s.append(String.format("  wheel fight  %7.1f in/s  %s%n%n", physics.wheelFight,
                physics.wheelFight > 5 ? "<-- WHEELS FIGHTING!" : "(ok)"));

        s.append("MODULES   servo  wheel  target   power\n");
        for (Physics.Module m : physics.modules) {
            s.append(String.format("  %-6s  %5.3f  %+5.0f  %+5.0f   %+5.2f%n",
                    m.name, m.servo.getPosition(),
                    Math.toDegrees(m.angle()), Math.toDegrees(m.commandedAngle()),
                    m.motor.getPower()));
        }
        s.append("  (angles in deg: 0 = forward, + = toward right)\n\n");

        s.append("HOW TO READ THIS\n");
        s.append("  Colored block = wheel. Arrow = its push.\n");
        s.append("  Dashed line = where the code told the\n");
        s.append("  wheel to point (it turns to catch up).\n");
        s.append("  Black arrow = whole robot's slide.\n");
        s.append("  Wheel fight near 0 = wheels agree.\n");
        s.append("  Big wheel fight = they're pushing\n");
        s.append("  against each other: something's wrong.\n");

        if (lastError != null) {
            s.append("\nmove() CRASHED:\n  ").append(lastError).append('\n');
        }
        return s.toString();
    }

    private static double norm180(double deg) {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(deg)), Math.cos(Math.toRadians(deg))));
    }
}
