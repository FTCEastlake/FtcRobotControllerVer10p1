package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ERCArm {

    //Gamepad2:
    // Arm Slider:
    // left stick Y: raise or lower arm slider
    // right bumper: manual override hard limits
    //
    // Arm:
    // Y: arm up
    // X: arm down
    // B: claw open
    // A: claw close

    ERCGlobalConfig _glbConfig = ERCGlobalConfig.getInstance();

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private ERCParameterLogger _logger;
    private Gamepad _gamepad2;

    private DcMotor _slideLeft = null;
    private DcMotor _slideRight = null;
    private Servo _armServo;
    private Servo _clawServo;

    private int _leftEncoderVal;
    private int _rightEncoderVal;
    private double _armPosition;
    private double _clawPosition;

    private String _paramLeftEncoder = "Arm Slider Left Encoder";
    private String _paramRightEncoder = "Arm Slider Right Encoder";
    private String _paramArmPosition= "Arm Position";
    private String _paramClawPosition = "Claw Position";
//    private String _paramArmLsy = "Arm lsy";

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

        _slideLeft = _hardwareMap.dcMotor.get("slideLeft");
        _slideRight = _hardwareMap.dcMotor.get("slideRight");
        _armServo = _hardwareMap.get(Servo.class, "armServo");
        _clawServo = _hardwareMap.get(Servo.class, "clawServo");

        _slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Note: you must set target position before setting the mode to RUN_TO_POSITION.
        _currentEncoderVal = 0;
        _slideLeft.setTargetPosition(_currentEncoderVal);
        _slideRight.setTargetPosition(_currentEncoderVal);
        _runMode = DcMotor.RunMode.RUN_TO_POSITION;
        _slideLeft.setMode(_runMode);
        _slideRight.setMode(_runMode);

        _armServo.setPosition(0.0);
        _clawServo.setPosition(0.0);
        _armPosition = _armServo.getPosition();
        _clawPosition = _clawServo.getPosition();


        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramLeftEncoder);
        _logger.addParameter(_paramRightEncoder);
        _logger.addParameter(_paramArmPosition);
        _logger.addParameter(_paramClawPosition);
        _logger.updateAll();;
    }

    public void setArm() throws InterruptedException {

        if (_gamepad2.right_bumper) setSliderManualOverride(); else setSliderNormalMode();

        if (_gamepad2.y) setArmUp();
        if (_gamepad2.x) setArmDown();
        if (_gamepad2.b) setClawOpen();
        if (_gamepad2.a) setClawClose();

        _armPosition = _armServo.getPosition();
        _clawPosition = _clawServo.getPosition();

//        _leftEncoderVal = _slideLeft.getCurrentPosition();
//        _rightEncoderVal = _slideRight.getCurrentPosition();

        DcMotor.RunMode runMode =  _slideLeft.getMode();
        _logger.updateStatus("Arm run mode = " + runMode);
//        _logger.updateParameter(_paramLeftEncoder, _leftEncoderVal);
//        _logger.updateParameter(_paramRightEncoder, _rightEncoderVal);
        _logger.updateParameter(_paramArmPosition, _armPosition);
        _logger.updateParameter(_paramClawPosition, _clawPosition);
        _logger.updateAll();
    }

    public void setArmUp() {
        _armServo.setPosition(0.0);
    }
    public void setArmDown() {
        _armServo.setPosition(_glbConfig.maxArmPosition);
    }
    public void setArmPosition(double position) {
        // Don't use this unless you really know what you're doing!
        _armServo.setPosition(position);
    }

    public void setClawOpen() {
        _clawServo.setPosition(0.0);
    }
    public void setClawClose() {
        _clawServo.setPosition(_glbConfig.maxClawPosition);
    }

    public void setSliderNormalMode() {
        _lsy = _gamepad2.left_stick_y;
        setSliderEncoder(-_lsy);
    }

    public void setSliderEncoder(double upPower) {
        if (_runMode != DcMotor.RunMode.RUN_TO_POSITION)
        {
            _runMode = DcMotor.RunMode.RUN_TO_POSITION;
            _slideLeft.setMode(_runMode);
            _slideRight.setMode(_runMode);
        }

        _currentEncoderVal = upPower > 0 ? _glbConfig.maxSliderEncoderVal : _glbConfig.minSliderEncoderVal;
        _slideLeft.setTargetPosition(_currentEncoderVal);
        _slideRight.setTargetPosition(-_currentEncoderVal);

        _slideLeft.setPower(upPower);
        _slideRight.setPower(upPower);    // don't have to set negative as setTargetPosition() is already inverted
        logSliderEncoderValues();
    }

    // encoderVal value should be in the range of _glbConfig.minSliderEncoderVal to _glbConfig.maxSliderEncoderVal
    // power should just be from 0.0 to 1.0 because the run mode is RUN_TO_POSITION.
    public void setSliderEncoderAuto(int encoderVal, double power) {
        if (_runMode != DcMotor.RunMode.RUN_TO_POSITION)
        {
            _runMode = DcMotor.RunMode.RUN_TO_POSITION;
            _slideLeft.setMode(_runMode);
            _slideRight.setMode(_runMode);
        }

        _currentEncoderVal = encoderVal;
        _slideLeft.setTargetPosition(_currentEncoderVal);
        _slideRight.setTargetPosition(-_currentEncoderVal);

        _slideLeft.setPower(power);
        _slideRight.setPower(power);    // don't have to set negative as setTargetPosition() is already inverted

        // Wait for slider to go to encoder value then shut off power to the motors so they won't overheat.
        while (_slideLeft.isBusy() && _slideRight.isBusy()) {
            // Wait for completion
        }
        _slideLeft.setPower(0.0);
        _slideLeft.setPower(0.0);
        logSliderEncoderValues();
    }

    public void setSliderToZero() {
        if (_runMode != DcMotor.RunMode.RUN_TO_POSITION)
        {
            _runMode = DcMotor.RunMode.RUN_TO_POSITION;
            _slideLeft.setMode(_runMode);
            _slideRight.setMode(_runMode);
        }

        _slideLeft.setTargetPosition(0);
        _slideRight.setTargetPosition(0);
        double power = 1.0;
        while (true)
        {
            _leftEncoderVal = _slideLeft.getCurrentPosition();
            _rightEncoderVal = _slideRight.getCurrentPosition();
            logSliderEncoderValues();
            if (_leftEncoderVal < 20)
                break;

            // Power values greater than 1.0 will be clipped at the lower level APIs
            power = (((double)_leftEncoderVal) / _glbConfig.maxSliderEncoderVal) + 0.33;
            if (power > 1.0) power = 1.0;
            _slideLeft.setPower(power);
            _slideRight.setPower(power);
        }

        _slideLeft.setPower(0.0);
        _slideRight.setPower(0.0);
    }

    public void setSliderManualOverride() {
        _lsy = _gamepad2.left_stick_y;
        setSliderPower(-_lsy);
    }

    public void setSliderPower(double upPower) {
        if (_runMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            _runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            _slideLeft.setMode(_runMode);
            _slideRight.setMode(_runMode);
        }

        _slideLeft.setPower(upPower);
        _slideRight.setPower(-upPower);
        logSliderEncoderValues();
    }

    private void logSliderEncoderValues() {
        _leftEncoderVal = _slideLeft.getCurrentPosition();
        _rightEncoderVal = _slideRight.getCurrentPosition();
        _logger.updateParameter(_paramLeftEncoder, _leftEncoderVal);
        _logger.updateParameter(_paramRightEncoder, _rightEncoderVal);
        _logger.updateAll();
    }
}
