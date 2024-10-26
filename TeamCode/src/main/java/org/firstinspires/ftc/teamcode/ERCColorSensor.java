package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ERCColorSensor {

    LinearOpMode _opMode;
    HardwareMap _hardwareMap;
    ERCParameterLogger _logger;
    RevColorSensorV3 _colorSensor;  // Note: RevColorSensorV3 combines both ColorSensor and DistanceSensor into one class

    int _blueVal;
    int _redVal;
    int _greenVal;
    int _alphaVal;
    double _distance;

    String _paramRedVal      = "Color Red";
    String _paramGreenVal    = "Color Green";
    String _paramBlueVal     = "Color Blue";
    String _paramAlphaVal    = "Color Alpha";
    String _paramDistanceVal = "Distance (inch)";


    public ERCColorSensor(LinearOpMode opMode, ERCParameterLogger logger) {

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _logger = logger;

        init();
    }

    private void init() {
        _colorSensor = _hardwareMap.get(RevColorSensorV3.class, "color sensor v3");

        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramRedVal);
        _logger.addParameter(_paramGreenVal);
        _logger.addParameter(_paramBlueVal);
        _logger.addParameter(_paramAlphaVal);
        _logger.addParameter(_paramDistanceVal);
    }

    public void getColor()
    {
        _redVal = _colorSensor.red();
        _greenVal = _colorSensor.green();
        _blueVal = _colorSensor.blue();
        _alphaVal = _colorSensor.alpha();
        _distance = _colorSensor.getDistance(DistanceUnit.INCH);


        _logger.updateParameter(_paramRedVal, _redVal);
        _logger.updateParameter(_paramGreenVal, _greenVal);
        _logger.updateParameter(_paramBlueVal, _blueVal);
        _logger.updateParameter(_paramAlphaVal, _alphaVal);
        _logger.updateParameter(_paramDistanceVal, _distance);
        _logger.updateAll();
    }
}
