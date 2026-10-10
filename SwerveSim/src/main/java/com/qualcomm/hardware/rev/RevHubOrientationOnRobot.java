package com.qualcomm.hardware.rev;

/* FAKE hub-orientation settings for SwerveSim. Stored, but the fake IMU ignores them. */
public class RevHubOrientationOnRobot {
    public enum LogoFacingDirection { UP, DOWN, FORWARD, BACKWARD, LEFT, RIGHT }
    public enum UsbFacingDirection { UP, DOWN, FORWARD, BACKWARD, LEFT, RIGHT }

    public final LogoFacingDirection logoFacingDirection;
    public final UsbFacingDirection usbFacingDirection;

    public RevHubOrientationOnRobot(LogoFacingDirection logo, UsbFacingDirection usb) {
        this.logoFacingDirection = logo;
        this.usbFacingDirection = usb;
    }
}
