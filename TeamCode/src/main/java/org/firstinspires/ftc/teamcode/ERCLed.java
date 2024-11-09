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
    RevBlinkinLedDriver _blinkinLed;

    //private String _paramLedColor= "LED color";

    public ERCLed(LinearOpMode opMode, ERCParameterLogger logger) {

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _logger = logger;

        init();
    }

    private void init() {

        _blinkinLed = _hardwareMap.get(RevBlinkinLedDriver.class, "blinkin led");

        // Add all of the parameters you want to see on the driver hub display.
        //_logger.addParameter(_paramLedColor);
    }

    public void setLedColor(RevBlinkinLedDriver.BlinkinPattern color)
    {
        _blinkinLed.setPattern(color);
    }

    public void setLedColorRed() {
        _blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public void setLedColorBlue() {
        _blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void setLedColorGreen() {
        _blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void setLedColorYellow() {
        _blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void setLedColorBlack() {
        _blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void setLedColorWhite() {
        _blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    // There is a way to program the strip to 5V or 12V but it's only for FRC APIs right now.
    // Hopefully this API will come to FTC someday.
    // https://www.chiefdelphi.com/t/rev-blinkin-resetting-strip-mode-randomly/432510/12
//    public void setStripTo12V() {
//        _blinkinLed.setPulseTime(2.145);    // 12V strip = 2145 us
//    }
//    public void setStripTo5V() {
//        _blinkinLed.setPulseTime(2.125);    // 5V strip = 2125 us
//    }
}
