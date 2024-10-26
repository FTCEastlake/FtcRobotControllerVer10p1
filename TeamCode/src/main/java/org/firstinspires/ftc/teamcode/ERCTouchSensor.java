package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ERCTouchSensor {

    LinearOpMode _opMode;
    HardwareMap _hardwareMap;
    ERCParameterLogger _logger;
    RevTouchSensor _touchSensor;

    String _paramTouch     = "Touch is Pressed";

    boolean _isPressed;

    public ERCTouchSensor(LinearOpMode opMode, ERCParameterLogger logger) {

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _logger = logger;

        init();
    }

    private void init() {
        // Note: Rev touch sensor should be configured as a digital device on the N+1 (odd) channel.
        // For example, if you connect it to digital channel 6-7 then you need to configure it on the 7th channel.
        _touchSensor = _hardwareMap.get(RevTouchSensor.class, "touch sensor");

        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramTouch);
    }

    public boolean isPressed()
    {
        _isPressed = _touchSensor.getValue() > 0 ? true : false;
        _logger.updateParameter(_paramTouch, _isPressed);
        _logger.updateAll();
        return _isPressed;
    }

}
