package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/*
 * HuskyColorCamera - HuskyLens configured for Color Recognition mode.
 *
 * Designed for ground detection of game elements (e.g., Nectar and Pollen).
 *
 * HOW TO TRAIN HUSKYLENS FOR MULTI-COLOR RECOGNITION:
 *   1. On the physical HuskyLens, dial the top wheel to "Color Recognition".
 *   2. Long-press the function wheel to open the settings menu.
 *   3. Navigate to "Learn Multiple" and click to set it to ON.
 *   4. Save & return to the main screen.
 *   5. Point the crosshairs at your Alliance Nectar:
 *      - Press the top learn button once. A box labeled "ID 1" will appear.
 *   6. Point the crosshairs at Pollen:
 *      - Press the learn button again. A box labeled "ID 2" will appear.
 *   7. (Optional) Point at Opponent Nectar as "ID 3" so you know to ignore it.
 *
 * In code:
 *   Call update() ONCE per loop, then query getPrioritizedTarget(), getBestByColorId(), etc.
 */
public class HuskyColorCamera {

    public static final double CENTER_X = 160.0;
    public static final double CENTER_Y = 120.0;

    // Field of view calibration: ~35 deg horizontal x ~25 deg vertical across 320x240 px
    public static double DEG_PER_PIXEL_X = 35.0 / 320.0;   // ~0.109 deg/px
    public static double DEG_PER_PIXEL_Y = 25.0 / 240.0;   // ~0.104 deg/px

    // Minimum bounding box area in pixels to reject specks / visual noise
    public static int MIN_DETECTION_AREA = 150;

    /** One detected color blob with angles relative to the camera crosshairs. */
    public static class ColorTarget {
        public final int id;
        public final int x;
        public final int y;
        public final int width;
        public final int height;
        public final int area;

        /** Horizontal angular error from center (+ = target is to the right, - = left). */
        public final double txDeg;

        /** Vertical angular error from center (+ = target is higher/further, - = lower/closer). */
        public final double tyDeg;

        public ColorTarget(HuskyLens.Block block) {
            this.id = block.id;
            this.x = block.x;
            this.y = block.y;
            this.width = block.width;
            this.height = block.height;
            this.area = block.width * block.height;
            this.txDeg = (block.x - CENTER_X) * DEG_PER_PIXEL_X;
            this.tyDeg = (CENTER_Y - block.y) * DEG_PER_PIXEL_Y; // screen y increases downward
        }

        @Override
        public String toString() {
            return String.format("ID:%d [%dx%d at (%d,%d) tx:%+.1f° ty:%+.1f°]",
                    id, width, height, x, y, txDeg, tyDeg);
        }
    }

    private final HuskyLens husky;
    private HuskyLens.Block[] rawBlocks = new HuskyLens.Block[0];
    private final List<ColorTarget> targets = new ArrayList<>();

    private HuskyColorCamera(HuskyLens husky) {
        this.husky = husky;
    }

    /**
     * Initializes the HuskyLens with the given configuration name, verifies communication,
     * and switches algorithm to COLOR_RECOGNITION.
     *
     * @param hardwareMap FTC hardware map
     * @param configName  Device name in the Robot Configuration (e.g. "groundHusky")
     */
    public static HuskyColorCamera init(HardwareMap hardwareMap, String configName) {
        HuskyLens husky = hardwareMap.get(HuskyLens.class, configName);
        if (!husky.knock()) {
            throw new RuntimeException("HuskyLens '" + configName + "' not responding! Check I2C wiring.");
        }
        husky.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        return new HuskyColorCamera(husky);
    }

    /**
     * Reads all blocks currently visible. Call this once at the start of each OpMode loop().
     */
    public void update() {
        rawBlocks = husky.blocks();
        targets.clear();
        if (rawBlocks != null) {
            for (HuskyLens.Block b : rawBlocks) {
                if (b.width * b.height >= MIN_DETECTION_AREA) {
                    targets.add(new ColorTarget(b));
                }
            }
        }
    }

    /** Returns all valid detected targets from the most recent update(). */
    public List<ColorTarget> getAllTargets() {
        return targets;
    }

    /** Returns all targets matching a specific color ID (e.g. 1 = Nectar, 2 = Pollen). */
    public List<ColorTarget> getTargetsById(int id) {
        List<ColorTarget> matching = new ArrayList<>();
        for (ColorTarget t : targets) {
            if (t.id == id) matching.add(t);
        }
        return matching;
    }

    /**
     * Finds the "best" target of a given color ID (the largest/closest one in view).
     * Returns null if no target with that ID is visible.
     */
    public ColorTarget getBestByColorId(int id) {
        ColorTarget best = null;
        for (ColorTarget t : targets) {
            if (t.id == id) {
                if (best == null || t.area > best.area) {
                    best = t;
                }
            }
        }
        return best;
    }

    /**
     * Hard priority selector:
     *   1. If any primary ID (e.g. Nectar) is visible, returns the best primary target.
     *   2. If no primary is visible, returns the best secondary ID (e.g. Pollen) target.
     *   3. If neither is visible, returns null.
     *
     * @param primaryId   ID of highest priority element (e.g. 1 for Nectar)
     * @param secondaryId ID of fallback element (e.g. 2 for Pollen)
     */
    public ColorTarget getPrioritizedTarget(int primaryId, int secondaryId) {
        ColorTarget primary = getBestByColorId(primaryId);
        if (primary != null) return primary;

        return getBestByColorId(secondaryId);
    }

    /** Returns true if at least one valid target of the given ID is in view. */
    public boolean hasTarget(int id) {
        return getBestByColorId(id) != null;
    }

    /** Returns total number of valid targets seen. */
    public int getTargetCount() {
        return targets.size();
    }
}
