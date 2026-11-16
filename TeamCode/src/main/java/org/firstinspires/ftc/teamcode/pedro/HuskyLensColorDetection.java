package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import java.util.ArrayList;
import java.util.List;

/*
 * HuskyLensColorDetection - identifies game elements (Nectar and Pollen) from HuskyLens color blocks.
 *
 * HOW TO TRAIN YOUR HUSKYLENS:
 *   1. Switch HuskyLens to "Color Recognition" mode.
 *   2. In menu settings, enable "Learn Multiple".
 *   3. Train in this order to match the IDs below:
 *        - ID 1: Pollen
 *        - ID 2: Red Nectar
 *        - ID 3: Blue Nectar
 *
 * Call findTarget() or identifyAll() passing the blocks from your camera.
 */
public class HuskyLensColorDetection {

    public static final double CENTER_X = 160.0;
    public static final double CENTER_Y = 120.0;

    // Field of view: ~35 deg horizontal x ~25 deg vertical across 320x240 px
    public static double DEG_PER_PIXEL_X = 35.0 / 320.0;
    public static double DEG_PER_PIXEL_Y = 25.0 / 240.0;

    // Reject specks/reflections smaller than this
    public static int MIN_PIXEL_AREA = 150;

    public static class ScoringElement {
        public final String name;
        public final int[] elementIDs;

        public ScoringElement(String name, int... elementIDs) {
            this.name = name;
            this.elementIDs = elementIDs;
        }

        public boolean matches(int id) {
            for (int eid : elementIDs) {
                if (eid == id) return true;
            }
            return false;
        }
    }

    public static final ScoringElement POLLEN = new ScoringElement("pollen", 1);
    public static final ScoringElement RED_NECTAR = new ScoringElement("redNectar", 2);
    public static final ScoringElement BLUE_NECTAR = new ScoringElement("blueNectar", 3);

    public static final ScoringElement[] ELEMENTS = {
            POLLEN,
            RED_NECTAR,
            BLUE_NECTAR
    };

    /** A single detected element with pixel and angle data. */
    public static class DetectedElement {
        public final String name;
        public final int id;
        public final int x;
        public final int y;
        public final int width;
        public final int height;
        public final int area;
        public final double txDeg; // + = right of crosshair
        public final double tyDeg; // + = above crosshair / farther away

        public DetectedElement(String name, HuskyLens.Block block) {
            this.name = name;
            this.id = block.id;
            this.x = block.x;
            this.y = block.y;
            this.width = block.width;
            this.height = block.height;
            this.area = block.width * block.height;
            this.txDeg = (block.x - CENTER_X) * DEG_PER_PIXEL_X;
            this.tyDeg = (CENTER_Y - block.y) * DEG_PER_PIXEL_Y;
        }

        public boolean isNectar() {
            return name.toLowerCase().contains("nectar");
        }

        public boolean isPollen() {
            return name.equalsIgnoreCase("pollen");
        }

        @Override
        public String toString() {
            return String.format("[%s (ID %d) at (%d,%d) tx:%+.1f° ty:%+.1f° area:%d]",
                    name, id, x, y, txDeg, tyDeg, area);
        }
    }

    /**
     * Converts raw HuskyLens blocks into detected game elements, filtering out noise.
     *
     * @param blocks   Blocks returned by huskyLens.blocks()
     * @param alliance "red" or "blue" (ignores the opponent's nectar). null = detect all.
     */
    public static List<DetectedElement> identifyAll(HuskyLens.Block[] blocks, String alliance) {
        List<DetectedElement> list = new ArrayList<>();
        if (blocks == null) return list;

        String opponentNectar = (alliance == null) ? null :
                (alliance.equalsIgnoreCase("red") ? "blueNectar" : "redNectar");

        for (HuskyLens.Block b : blocks) {
            if (b.width * b.height < MIN_PIXEL_AREA) continue;

            for (ScoringElement elem : ELEMENTS) {
                if (elem.matches(b.id)) {
                    // Skip opponent nectar
                    if (opponentNectar != null && elem.name.equalsIgnoreCase(opponentNectar)) {
                        continue;
                    }
                    list.add(new DetectedElement(elem.name, b));
                    break;
                }
            }
        }
        return list;
    }

    /**
     * Finds the best target to pursue, strictly prioritizing Alliance Nectar over Pollen.
     * If multiple targets of the winning type are seen, returns the largest/closest one.
     *
     * @param blocks   Blocks returned by huskyLens.blocks()
     * @param alliance "red" or "blue"
     * @return Best target, or null if nothing valid is in view.
     */
    public static DetectedElement findBestTarget(HuskyLens.Block[] blocks, String alliance) {
        List<DetectedElement> visible = identifyAll(blocks, alliance);
        if (visible.isEmpty()) return null;

        DetectedElement bestNectar = null;
        DetectedElement bestPollen = null;

        for (DetectedElement elem : visible) {
            if (elem.isNectar()) {
                if (bestNectar == null || elem.area > bestNectar.area) {
                    bestNectar = elem;
                }
            } else if (elem.isPollen()) {
                if (bestPollen == null || elem.area > bestPollen.area) {
                    bestPollen = elem;
                }
            }
        }

        // Hard priority: Nectar > Pollen
        if (bestNectar != null) return bestNectar;
        return bestPollen;
    }
}
