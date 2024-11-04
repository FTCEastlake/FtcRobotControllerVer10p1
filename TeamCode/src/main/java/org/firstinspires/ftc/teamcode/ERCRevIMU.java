package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ERCRevIMU implements ERCImuInterface {

    ERCGlobalConfig _glbConfig = ERCGlobalConfig.getInstance();

    // Note: Logitech 310 button mappings can be referenced here
    // https://gm0.org/en/latest/docs/software/tutorials/gamepad.html
    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private ERCParameterLogger _logger;
    private IMU _imu = null;

    public ERCRevIMU(LinearOpMode opMode, ERCParameterLogger logger){

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _logger = logger;

        init();
    }

    private void init() {

        // Retrieves the (first) device with the indicated name which is also an instance of the indicated class or interface.
        // First IMU is on the controller hub (version BHI260AP)
        _imu = _hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(_glbConfig.hubLogoFacingDir, _glbConfig.hubUsbFacingDir));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        _imu.initialize(parameters);
    }

    public void resetYaw()
    {
        _imu.resetYaw();
    }

    public double getBotHeading() {
        return _imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
