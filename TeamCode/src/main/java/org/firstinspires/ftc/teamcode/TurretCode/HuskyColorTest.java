package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Husky Color Test - Diagnostic OpMode for ground color detection.
 *
 * Tests color detection for Nectar (ID 1) and Pollen (ID 2).
 *
 * HARDWARE CONFIGURATION:
 *   Ensure the camera name in your Robot Configuration matches CAMERA_CONFIG_NAME below.
 *   Default: "groundHusky" (or change to "stationaryHusky" if reusing that camera).
 *
 * CONTROLS & DISPLAY:
 *   - Displays all detected color blocks on telemetry with their ID, bounding box, area,
 *     and angular offset (txDeg, tyDeg).
 *   - Shows the prioritized target (Nectar if visible, else Pollen).
 */
@TeleOp(name = "Husky Color Test", group = "Vision")
public class HuskyColorTest extends OpMode {

    // Set to your camera's hardware configuration name
    public static final String CAMERA_CONFIG_NAME = "groundHusky";

    // Color ID assignments (trained on the physical HuskyLens)

    public int ALLIANCENECTAR_ID = 0;
   public static final int BLUENECTAR_ID =2;
    public static final int POLLEN_ID = 1;
public static final int REDNECTAR_ID =3;
    private HuskyColorCamera colorCamera;
    private boolean cameraOk = false;

    @Override
    public void init() {
        try {
            colorCamera = HuskyColorCamera.init(hardwareMap, CAMERA_CONFIG_NAME);
            cameraOk = true;
            telemetry.addLine("HuskyLens Color Camera initialized successfully!");
        } catch (Exception e) {
            cameraOk = false;
            telemetry.addData("ERROR", e.getMessage());
            telemetry.addLine("Check that '" + CAMERA_CONFIG_NAME + "' is plugged into an I2C port");
            telemetry.addLine("and configured in the Robot Controller configuration.");
        }

        if(gamepad1.dpadUpWasPressed()){
            ALLIANCENECTAR_ID = BLUENECTAR_ID;
        }
        if(gamepad1.dpadDownWasPressed()){
            ALLIANCENECTAR_ID = REDNECTAR_ID;
        }

        telemetry.update();
    }

    @Override
    public void loop() {
        if (!cameraOk) {
            telemetry.addLine("Camera not available. Restart OpMode after fixing wiring/config.");
            telemetry.update();
            return;
        }

        colorCamera.update();

        telemetry.addLine("=== HUSKYLENS COLOR DETECTION ===");
        telemetry.addData("Total Blobs Seen", colorCamera.getTargetCount());

        // Prioritized target check
        HuskyColorCamera.ColorTarget bestTarget = colorCamera.getPrioritizedTarget(ALLIANCENECTAR_ID, POLLEN_ID);
        if (bestTarget != null) {
            String name = (bestTarget.id == ALLIANCENECTAR_ID) ? "NECTAR (PRIORITY)" : "POLLEN";
            telemetry.addLine(String.format("--> LOCKED ON: %s <--", name));
            telemetry.addData("  Error tx", String.format("%+.1f° (%s)",
                    bestTarget.txDeg, bestTarget.txDeg > 0 ? "RIGHT" : "LEFT"));
            telemetry.addData("  Error ty", String.format("%+.1f° (%s)",
                    bestTarget.tyDeg, bestTarget.tyDeg > 0 ? "FARTHER" : "CLOSER"));
            telemetry.addData("  Size Area", "%d px (%dx%d)", bestTarget.area, bestTarget.width, bestTarget.height);
        } else {
            telemetry.addLine("--> LOCKED ON: None (searching...)");
        }

        telemetry.addLine("--- ALL VISIBLE TARGETS ---");
        for (HuskyColorCamera.ColorTarget target : colorCamera.getAllTargets()) {
            String label = (target.id == ALLIANCENECTAR_ID) ? "Nectar" :
                           (target.id == POLLEN_ID) ? "Pollen" : "ID " + target.id;
            telemetry.addLine(String.format("  [%s] tx:%+.1f° ty:%+.1f° area:%d",
                    label, target.txDeg, target.   tyDeg, target.area));
        }

        telemetry.update();
    }
}
