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
    public boolean useNavxImu = false;

    // Vision configurations
    public boolean useGoBuildaCamera = true;
    public boolean enableVisionColorSensor = true;
    public double visionImageRegionLeft = -0.1;     // 0 = center, -1 = left
    public double visionImageRegionRight = 0.1;     // 0 = center, +1 = right
    public double visionImageRegionTop = 0.1;       // 0 = center, +1 = top
    public double visionImageRegionBottom = -0.1;   // 0 = center, -1 = bottom

    // Arm configurations
    // Back off encoder min and max values so the gears won't grind.
    public int minSliderEncoderVal = 50;     // min is zero
    public int maxSliderEncoderVal = 8800;    // max looks like it's 9200
    public double maxArmPosition = 0.31;
    public double maxClawPosition = 0.29;

}
