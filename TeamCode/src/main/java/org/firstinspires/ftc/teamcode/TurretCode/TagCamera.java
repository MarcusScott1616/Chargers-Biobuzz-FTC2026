package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * TagCamera - one HuskyLens in AprilTag mode, plus the math that turns a tag's
 * pixel position into angles (the Limelight used to give these angles directly).
 *
 * Call update() ONCE per loop, then findTag() / findFirstTag() as many times as you like.
 * The HuskyLens is slow to read over I2C, so this keeps it to one read per camera per
 * loop, even when two turrets share the same camera.
 *
 * The HuskyLens picture is 320 x 240 pixels. Center = (160, 120).
 */
public class TagCamera {

    public static final double CENTER_X = 160;
    public static final double CENTER_Y = 120;

    // TUNE: degrees per pixel. The HuskyLens FOV is roughly 35 deg horizontal x 25 deg vertical.
    // Calibrate: put a tag at a known angle from the camera and adjust until the angle
    // printed by HuskyTest matches.
    public static double DEG_PER_PIXEL_X = 35.0 / 320;   // ~0.109
    public static double DEG_PER_PIXEL_Y = 25.0 / 240;   // ~0.104

    /** One tag seen this loop, with its position as angles from the center of the picture. */
    public static class Tag {
        public final int id;
        public final double txDeg;    // + = right of center (same meaning as the Limelight's tx)
        public final double tyDeg;    // + = above center (same meaning as the Limelight's ty)
        public final int widthPx;
        public final int heightPx;

        Tag(HuskyLens.Block block) {
            this.id = block.id;
            this.txDeg = (block.x - CENTER_X) * DEG_PER_PIXEL_X;
            this.tyDeg = (CENTER_Y - block.y) * DEG_PER_PIXEL_Y;   // screen y grows DOWN, so flip it
            this.widthPx = block.width;
            this.heightPx = block.height;
        }
    }

    private final HuskyLens husky;
    private HuskyLens.Block[] blocks = new HuskyLens.Block[0];

    private TagCamera(HuskyLens husky) {
        this.husky = husky;
    }

    /**
     * Find the HuskyLens named configName in the robot configuration and switch it to
     * AprilTag mode. Throws a RuntimeException if the camera doesn't respond (loose cable, etc.).
     */
    public static TagCamera init(HardwareMap hardwareMap, String configName) {
        HuskyLens husky = hardwareMap.get(HuskyLens.class, configName);
        if (!husky.knock()) {
            throw new RuntimeException("HuskyLens '" + configName + "' not responding! Check the I2C cable.");
        }
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        return new TagCamera(husky);
    }

    /** Read what the camera sees right now. Call once per loop. */
    public void update() {
        blocks = husky.blocks();
    }

    /** Everything seen at the last update(). Used by HuskyTest to list the tag IDs. */
    public HuskyLens.Block[] getBlocks() {
        return blocks;
    }

    /** The tag with this ID as seen at the last update(), or null if it isn't in view. */
    public Tag findTag(int id) {
        for (HuskyLens.Block block : blocks) {
            if (block.id == id) return new Tag(block);
        }
        return null;
    }

    /**
     * The first tag from the given list of IDs that is visible, or null if none are.
     * Use this for cluster-based lookups: pass the cluster's tag ID array and get back
     * whichever tag from that cluster the camera can actually see.
     */
    public Tag findFirstTag(int[] ids) {
        for (int id : ids) {
            for (HuskyLens.Block block : blocks) {
                if (block.id == id) return new Tag(block);
            }
        }
        return null;
    }
}
