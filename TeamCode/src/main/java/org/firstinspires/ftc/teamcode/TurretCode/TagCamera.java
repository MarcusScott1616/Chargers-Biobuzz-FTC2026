package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * TagCamera - one HuskyLens in AprilTag mode, plus the math that turns a tag's
 * pixel position into angles (the Limelight used to give these angles directly).
 *
 * Call update() ONCE per loop, then findTag() as many times as you like. The HuskyLens is
 * slow to read over I2C, so this keeps it to one read per camera per loop, even when two
 * turrets share the same camera.
 *
 * The HuskyLens picture is 320 x 240 pixels. Center = (160, 120).
 */
public class TagCamera {

    private final String DESIRED_CLUSTER_NAME = null;

    public static final double CENTER_X = 160;
    public static final double CENTER_Y = 120;

    // TUNE: degrees per pixel. These are a rough guess (about 64 degrees across 320 px and
    // 48 degrees across 240 px). Calibrate: put a tag at a known angle from the camera and
    // adjust until the angle printed by HuskyTest matches.
    public static double DEG_PER_PIXEL_X = 0.2;
    public static double DEG_PER_PIXEL_Y = 0.2;



    final double DESIRED_DISTANCE = 30.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftDrive   = null;  //  Used to control the left drive wheel
    private DcMotor rightDrive  = null;  //  Used to control the right drive wheel

    private final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private final int DESIRED_TAG_ID = -1;    // The tag you want to approach, or set to -1 for ANY tag.

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    private boolean targetFound  = false;    // Set to true when an AprilTag/Cluster target is detected
    private String targetName    = "none";
    private int    targetID      = 0;
    private double targetRange   = 0;
    private double targetBearing = 0;
    private double targetYaw     = 0;

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

    /** Find the HuskyLens named configName in the robot configuration and switch it to AprilTag mode. */
    public static TagCamera init(HardwareMap hardwareMap, String configName) {
        HuskyLens husky = hardwareMap.get(HuskyLens.class, configName);
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
}
