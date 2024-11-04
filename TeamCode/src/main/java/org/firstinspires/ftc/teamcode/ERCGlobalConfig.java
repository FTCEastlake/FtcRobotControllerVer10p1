package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class ERCGlobalConfig {

    // Static variable (shared across all instances)
    private static ERCGlobalConfig instance = null;

    // Private constructor prevents direct instantiation
    private ERCGlobalConfig() { }

    // Static method to get the single instance
    public static ERCGlobalConfig getInstance() {
        if (instance == null) {
            instance = new ERCGlobalConfig();
        }
        return instance;
    }

    public enum RobotType {
        Standard,
        HolyCrab
    }

    //***********************************************************************
    // Global configurations start below.
    //***********************************************************************

    // Robot Type
    public RobotType robotType = RobotType.HolyCrab;

    // Control hub configurations
    RevHubOrientationOnRobot.LogoFacingDirection hubLogoFacingDir = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection hubUsbFacingDir = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // IMU Configurations
    public boolean useNavxImu = true;

    // Vision configurations
    public boolean useGoBuildaCamera = true;
    public boolean enableVisionColorSensor = true;

    // Arm configurations
    // Back off encoder min and max values so the gears won't grind.
    public int minEncoderVal = 400;     // min is zero
    public int maxEncoderVal = 8800;    // max looks like it's 9200


}
