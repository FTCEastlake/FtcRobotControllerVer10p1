package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;

public class ERCDrivetrain {

//    public enum RobotType {
//        Standard,
//        HolyCrab
//    }

    ERCGlobalConfig _glbConfig = ERCGlobalConfig.getInstance();

    // Note: Logitech 310 button mappings can be referenced here
    // https://gm0.org/en/latest/docs/software/tutorials/gamepad.html
    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private ERCParameterLogger _logger;
    private Gamepad _gamepad1;
    private ERCVision _vision = null;
    private ERCImuInterface _imu;

    // Declare motors
    private DcMotorEx _frontLeft = null;
    private DcMotorEx _frontRight = null;
    private DcMotorEx _backLeft = null;
    private DcMotorEx _backRight = null;

    // Constants for encoder calculations
    private static final double COUNTS_PER_MOTOR_REV = 537.7; // For GoBILDA 312 RPM Yellow Jacket
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

//    // Drive speed constants
//    private static final double DRIVE_SPEED = 0.6;
//    private static final double TURN_SPEED = 0.5;
//    private static final double STRAFE_SPEED = 0.6;

    // Number of inches to move when joystick is max (1.0) in any direction.
    private double _inchesPerUnitPower = 24.0;

    private String _paramFLEncVal = "FL Encoder";
    private String _paramFREncVal = "FR Encoder";
    private String _paramBLEncVal = "BL Encoder";
    private String _paramBREncVal = "BR Encoder";

    //shorten controller values for readability, values are "float" data type internally
    private double _lsx = 0;    //lsx = left stick x
    private double _lsy = 0;    //lsy = left stick y
    private double _rsx = 0;    //rsx = right stick x

    private ElapsedTime _runtime = new ElapsedTime();

    public ERCDrivetrain(LinearOpMode opMode, ERCParameterLogger logger, ERCVision vision, boolean shouldResetYaw) throws InterruptedException {

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _gamepad1 = _opMode.gamepad1;
        _logger = logger;
        _vision = vision;

        init(shouldResetYaw);
    }

    private void init(boolean shouldResetYaw) throws InterruptedException {

        // Select the IMU to use.
        _imu = new ERCRevIMU(_opMode, _logger);

        // Make sure your ID's match your configuration
        _frontLeft = (DcMotorEx)_hardwareMap.dcMotor.get("frontLeft");
        _frontRight = (DcMotorEx)_hardwareMap.dcMotor.get("frontRight");
        _backLeft = (DcMotorEx)_hardwareMap.dcMotor.get("backLeft");
        _backRight = (DcMotorEx)_hardwareMap.dcMotor.get("backRight");


        switch (_glbConfig.robotType) {
            case HolyCrab:
            case Frankenstein:
                // Both front wheels have motors that are perpendicular to the wheels.
                // Reverse the current directions of the back wheels to align properly.
                _frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                _backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                _frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                _backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            default:
                if (_glbConfig.hubLogoFacingDir == RevHubOrientationOnRobot.LogoFacingDirection.DOWN) {
                    _frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                    _backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    _frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                    _backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                break;  // nothing to alter
        }


        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramFLEncVal);
        _logger.addParameter(_paramFREncVal);
        _logger.addParameter(_paramBLEncVal);
        _logger.addParameter(_paramBREncVal);

        resetEncoders();
        if (shouldResetYaw)
            resetYaw();

    }

    public void resetYaw() {
        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        _imu.resetYaw();
        _logger.updateStatus("IMU reset yaw");
        _logger.updateAll();
    }

    public void resetEncoders() {
        // Set zero power behavior
        _frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Switch to RUN_TO_POSITION mode
        _frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set target positions
        // Do this so code doesn't throw error when the very next command is not setTargetPosition()
        _frontLeft.setTargetPosition(0);
        _frontRight.setTargetPosition(0);
        _backLeft.setTargetPosition(0);
        _backRight.setTargetPosition(0);
    }

    // Field-Centric code
    // Code is based of https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
    public void move(double forward, double right, double turnCounterClkPower, double drivePower) {

        double power = (right == 0.0) && (forward == 0.0) && (turnCounterClkPower == 0.0) ? 0.0 : drivePower;

        // Don't need to waste processing time if we're not moving.
        if (power == 0.0) {
            setPower(0.0);
            _logger.updateParameter(_paramFLEncVal, _frontLeft.getCurrentPosition());
            _logger.updateParameter(_paramFREncVal, _frontRight.getCurrentPosition());
            _logger.updateParameter(_paramBLEncVal, _backLeft.getCurrentPosition());
            _logger.updateParameter(_paramBREncVal, _backRight.getCurrentPosition());
            _logger.updateAll();
            return;
        }

        double y = forward;
        double x = right;
        double rx = turnCounterClkPower;
        double botHeading = _imu.getBotHeading();

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        //rotX = rotX * _strafeMagnitude;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        // Divide by _inchesPerUnitPower here is more efficient than multiplying with power 4 times.
        double denominator = Math.max(abs(rotY) + abs(rotX) + abs(rx), 1) / _inchesPerUnitPower;
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Calculate new target positions
        int newFLTarget = _frontLeft.getCurrentPosition() + (int)(frontLeftPower * COUNTS_PER_INCH);
        int newFRTarget = _frontRight.getCurrentPosition() + (int)(frontRightPower * COUNTS_PER_INCH);
        int newBLTarget = _backLeft.getCurrentPosition() + (int)(backLeftPower * COUNTS_PER_INCH);
        int newBRTarget = _backRight.getCurrentPosition() + (int)(backRightPower * COUNTS_PER_INCH);

        // Set target positions
        _frontLeft.setTargetPosition(newFLTarget);
        _frontRight.setTargetPosition(newFRTarget);
        _backLeft.setTargetPosition(newBLTarget);
        _backRight.setTargetPosition(newBRTarget);

        // Apply final power
        setPower(power);

        _logger.updateParameter(_paramFLEncVal, newFLTarget);
        _logger.updateParameter(_paramFREncVal, newFRTarget);
        _logger.updateParameter(_paramBLEncVal, newBLTarget);
        _logger.updateParameter(_paramBREncVal, newBRTarget);
        _logger.updateAll();
    }

    private void setPower(double power) {
        _frontLeft.setPower(power);
        _frontRight.setPower(power);
        _backLeft.setPower(power);
        _backRight.setPower(power);
    }

    public void autoMove(double forwardInches, double rightInches, double degrees, double driveSpeed) {

        double forwardSign = forwardInches < 0.0 ? -1.0 : 1.0;
        double forwardPower = abs(forwardInches) / _inchesPerUnitPower;
        while (forwardPower != 0.0)
        {
            double currentPower = forwardPower > 1.0 ? 1.0 : forwardPower;
            move(forwardSign * currentPower, 0, 0, driveSpeed);
            while ((_frontLeft.isBusy() && _frontRight.isBusy() &&
                    _backLeft.isBusy() && _backRight.isBusy())) {
                // Wait for completion
            }
            forwardPower -= currentPower;
        }

        double leftSign = rightInches < 0.0 ? -1.0 : 1.0;
        double leftPower = abs(rightInches) / _inchesPerUnitPower;
        while (leftPower != 0.0)
        {
            double currentPower = leftPower > 1.0 ? 1.0 : leftPower;
            move(0,leftSign * currentPower, 0, driveSpeed);
            while ((_frontLeft.isBusy() && _frontRight.isBusy() &&
                    _backLeft.isBusy() && _backRight.isBusy())) {
                // Wait for completion
            }
            leftPower -= currentPower;
        }
        move(0, 0, 0, driveSpeed);


//        double forwardPower = forwardInches < 0 ? -1.0 : 1.0;
////        double leftPower = 0.0;
////        double turnCounterClkPowerPower = 0.0;
//
//        //forwardPower = forwardInches * COUNTS_PER_INCH / _inchesPerUnitPower;
//
//        move(forwardPower, 0, 0, driveSpeed);
//
//        while ((_frontLeft.isBusy() && _frontRight.isBusy() &&
//                _backLeft.isBusy() && _backRight.isBusy())) {
//            // Wait for completion
//        }
//        move(0, 0, 0, driveSpeed);
    }

}


