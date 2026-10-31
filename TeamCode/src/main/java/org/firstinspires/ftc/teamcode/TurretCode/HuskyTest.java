package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Husky Tag Test - hardware setup check for both HuskyLenses.
 *
 * Shows EVERY block each camera sees, with its tag ID, so you can confirm which AprilTags
 * the HuskyLens can track. Point a camera at a tag and check that its ID shows up here.
 *
 * Hardware names (must match the Driver Station robot configuration):
 *   "stationaryHusky" - fixed HuskyLens (distance to the HIVE)
 *   "turretHusky"     - HuskyLens on the turret (aiming)
 */
@TeleOp(name = "Husky Tag Test", group = "Turret")
public class HuskyTest extends OpMode {

    private TagCamera stationary, turret;

    @Override
    public void init() {
        stationary = TagCamera.init(hardwareMap, "stationaryHusky");
        turret = TagCamera.init(hardwareMap, "turretHusky");

        telemetry.addLine("Husky Tag Test ready. Press PLAY.");
    }

    @Override
    public void loop() {
        stationary.update();
        turret.update();

        listCamera("STATIONARY", stationary);
        listCamera("TURRET", turret);
        telemetry.update();
    }

    /** Print every block this camera sees: tag ID, angles from center, and size in pixels. */
    private void listCamera(String name, TagCamera camera) {
        HuskyLens.Block[] blocks = camera.getBlocks();
        telemetry.addLine(name + ": " + blocks.length + " seen");

        for (HuskyLens.Block b : blocks) {
            telemetry.addLine(String.format("  ID %d   tx %+.1f   ty %+.1f   %dx%d px",
                    b.id,
                    (b.x - TagCamera.CENTER_X) * TagCamera.DEG_PER_PIXEL_X,
                    (TagCamera.CENTER_Y - b.y) * TagCamera.DEG_PER_PIXEL_Y,
                    b.width, b.height));
        }
    }
}
