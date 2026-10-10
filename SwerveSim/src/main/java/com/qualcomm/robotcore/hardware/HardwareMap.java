package com.qualcomm.robotcore.hardware;

import java.util.HashMap;
import java.util.Map;

/*
 * FAKE version of the FTC HardwareMap, used only by SwerveSim on a laptop.
 *
 * On the real robot, hardwareMap.get() looks up a device in the robot's configuration
 * and throws an error if the name is missing. Here we just create a fake device the
 * first time a name is asked for, and hand back the same object every time after that.
 * That way the simulator can ask for "redServo" too and see what SwerveDrive told it.
 */
public class HardwareMap {
    private final Map<String, Object> devices = new HashMap<>();

    @SuppressWarnings("unchecked")
    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        Object device = devices.get(deviceName);
        if (device == null) {
            try {
                device = classOrInterface.getDeclaredConstructor().newInstance();
            } catch (ReflectiveOperationException e) {
                throw new IllegalArgumentException("SwerveSim has no fake for " + classOrInterface.getSimpleName()
                        + " (device \"" + deviceName + "\"). Add one in SwerveSim/src/main/java.", e);
            }
            devices.put(deviceName, device);
        }
        return (T) device;
    }
}
