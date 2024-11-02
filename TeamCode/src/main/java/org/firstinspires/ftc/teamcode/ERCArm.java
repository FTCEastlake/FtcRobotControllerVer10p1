package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ERCArm {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private ERCParameterLogger _logger;
    private Gamepad _gamepad2;

    private DcMotor _armLeft = null;
    private DcMotor _armRight = null;

    private int _leftEncoderVal;
    private int _rightEncoderVal;

    private String _paramLeftEncoder = "Arm Left Encoder";
    private String _paramRightEncoder = "Arm Right Encoder";
    private String _paramArmLsy = "Arm lsy";

    // Give a buffer of 200 to lessen gear grinding.
    private int _minEncoderVal = 500;
    private int _maxEncoderVal = 11000; // max looks like it's 11500
    private int _currentEncoderVal;

    private DcMotor.RunMode _runMode;
    private double _lsy;

    public ERCArm(LinearOpMode opMode, ERCParameterLogger logger) {

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _gamepad2 = _opMode.gamepad2;
        _logger = logger;

        init();
    }

    private void init() {

        _armLeft = _hardwareMap.dcMotor.get("armLeft");
        _armRight = _hardwareMap.dcMotor.get("armRight");

        _armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Note: you must set target position before setting the mode to RUN_TO_POSITION.
        _currentEncoderVal = 0;
        _armLeft.setTargetPosition(_currentEncoderVal);
        _armRight.setTargetPosition(_currentEncoderVal);
        _runMode = DcMotor.RunMode.RUN_TO_POSITION;
        _armLeft.setMode(_runMode);
        _armRight.setMode(_runMode);


        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramLeftEncoder);
        _logger.addParameter(_paramRightEncoder);
        _logger.addParameter(_paramArmLsy);
    }

    public void setArmWithButtons() {

//        // max right encoder val = -11500
//        double lsy = _gamepad2.left_stick_y;
//        _armLeft.setPower(-1 * lsy * 1.0);  // real code
//        _armRight.setPower(lsy * 1.0);

        boolean isButtonPressed = false;
        int encoderVal = 0;
        if (_gamepad2.y) {
            isButtonPressed = true;
            encoderVal = 2000;//_maxEncoderVal;
        }
        else if (_gamepad2.x) {

            isButtonPressed = true;
            encoderVal = 1000; //_maxEncoderVal / 2;
        }
        else if (_gamepad2.a)
        {
            isButtonPressed = true;
            encoderVal = 0;
        }

        if (isButtonPressed)
        {
            _armLeft.setTargetPosition(encoderVal);
            _armRight.setTargetPosition(-1 * encoderVal);
            _armLeft.setPower(1.0);
            _armRight.setPower(1.0);
        }



        _leftEncoderVal = _armLeft.getCurrentPosition();
        _rightEncoderVal = _armRight.getCurrentPosition();

        DcMotor.RunMode runMode =  _armLeft.getMode();
        _logger.updateStatus("Arm run mode = " + runMode);
        _logger.updateParameter(_paramLeftEncoder, _leftEncoderVal);
        _logger.updateParameter(_paramRightEncoder, _rightEncoderVal);
        //_logger.updateParameter(_paramArmLsy, lsy);
        _logger.updateAll();
    }

    public void setArm() {

//        if (_gamepad2.a)
//            setManualOverride();
//        else
//            setNormalMode();

        if (_gamepad2.a)
            setNormalMode();
        else
            setManualOverride();

        _leftEncoderVal = _armLeft.getCurrentPosition();
        _rightEncoderVal = _armRight.getCurrentPosition();

        DcMotor.RunMode runMode =  _armLeft.getMode();
        _logger.updateStatus("Arm run mode = " + runMode);
        _logger.updateParameter(_paramLeftEncoder, _leftEncoderVal);
        _logger.updateParameter(_paramRightEncoder, _rightEncoderVal);
        _logger.updateParameter(_paramArmLsy, _lsy);
        _logger.updateAll();
    }

    public void setNormalMode() {

        if (_runMode != DcMotor.RunMode.RUN_TO_POSITION)
        {
            _runMode = DcMotor.RunMode.RUN_TO_POSITION;
            _armLeft.setMode(_runMode);
            _armRight.setMode(_runMode);
        }

        _lsy = _gamepad2.left_stick_y;  // down direction is positive value
        _currentEncoderVal = _lsy < 0 ? _maxEncoderVal : _minEncoderVal;
        _armLeft.setTargetPosition(_currentEncoderVal);
        _armRight.setTargetPosition(-_currentEncoderVal);

        _armLeft.setPower(_lsy);
        _armRight.setPower(_lsy);
    }

    public void setManualOverride() {
        if (_runMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            _runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            _armLeft.setMode(_runMode);
            _armRight.setMode(_runMode);
        }

        _lsy = _gamepad2.left_stick_y;
        _armLeft.setPower(-_lsy);
        _armRight.setPower(_lsy);
    }
}
