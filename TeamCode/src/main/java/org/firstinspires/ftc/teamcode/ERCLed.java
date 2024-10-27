package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ERCLed {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private ERCParameterLogger _logger;
    private Gamepad _gamepad2;
    RevBlinkinLedDriver _blinkinLed;

    //private String _paramLedColor= "LED color";

    public ERCLed(LinearOpMode opMode, ERCParameterLogger logger) {

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _gamepad2 = _opMode.gamepad2;
        _logger = logger;

        init();
    }

    private void init() {

        _blinkinLed = _hardwareMap.get(RevBlinkinLedDriver.class, "blinkin led");

        // Add all of the parameters you want to see on the driver hub display.
        //_logger.addParameter(_paramLedColor);
    }

    public void setLedColor()
    {
        if (_gamepad2.y)
            _blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        else if (_gamepad2.b)
            _blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        else if (_gamepad2.a)
            _blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        else if (_gamepad2.x)
            _blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        else
            _blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
}
