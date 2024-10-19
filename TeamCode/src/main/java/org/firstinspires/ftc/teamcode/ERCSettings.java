package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class ERCSettings {

    LinearOpMode _opMode;
    HardwareMap _hardwareMap;
    ERCParameterLogger _logger;
    Gamepad _gamepad1;
    IMU _imu = null;
    ERCVision _vision = null;

    String _paramCameraExposureMs = "Camera Exposure Ms";
    String _paramCameraGain = "Camera Gain";

    public ERCSettings(LinearOpMode opMode, ERCParameterLogger logger, ERCVision vision){
        _opMode = opMode;
        _hardwareMap = _opMode.hardwareMap;
        _gamepad1 = _opMode.gamepad1;
        _logger = logger;
        _vision = vision;

        _logger.addParameter(_paramCameraExposureMs);
        _logger.addParameter(_paramCameraGain);
    }

    // While holding left bumper do the following:
    // exposure: press X to increment, A to decrement
    // gain: press Y to increment, B to decrement
    public void ConfigSettings()
    {
        //_vision.waitForCameraReady();

        int exposureMs = _vision.getCameraExposureMs();
        int gain = _vision.getCameraGain();

        if (_gamepad1.left_bumper)
        {
            boolean needAdjustment = false;
            if (_gamepad1.x) { exposureMs += 1; needAdjustment = true; }
            else if (_gamepad1.a) { exposureMs -= 1; needAdjustment = true; }
            else if (_gamepad1.y) { gain += 1; needAdjustment = true; }
            else if (_gamepad1.b) { gain -= 1; needAdjustment = true; }
            if (needAdjustment)
                _vision.setManualExposure(exposureMs, gain);
            _opMode.sleep(200); // debounce
        }

        _logger.updateParameter(_paramCameraExposureMs, exposureMs);
        _logger.updateParameter(_paramCameraGain, gain);
        _logger.updateAll();
    }

}
