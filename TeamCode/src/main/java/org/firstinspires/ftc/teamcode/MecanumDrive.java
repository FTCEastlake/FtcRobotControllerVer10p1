package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.HashMap;

public class MecanumDrive {

    LinearOpMode _opMode;
    HardwareMap _hardwareMap;
    ERCParameterLogger _logger;
    Gamepad _gamepad1;

    // Declare motors
    DcMotor _frontLeft = null;
    DcMotor _frontRight = null;
    DcMotor _backLeft = null;
    DcMotor _backRight = null;

    String _paramLsx = "Left Stick X";
    String _paramLsr = "Left Stick Y";
    String _paramRsx = "Right Stick X";

    //shorten controller values for readability, values are "float" data type internally
    double _lsx = 0;      //lsx = left stick x
    double _lsy = 0;      //lsy = right stick y
    double _rsx = 0;     //rsx = right stick x

    double _powerFactor = 0.5;     // 1.0 = full power, 0.0 = no power
    double _strafeMagnitude = 1.1;
    IMU _imu;

    public MecanumDrive(LinearOpMode opMode, ERCParameterLogger logger,
                        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection,
                        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection){

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _gamepad1 = _opMode.gamepad1;
        _logger = logger;

        init(logoFacingDirection, usbFacingDirection);
    }

    private void init(RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection,
                      RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection)
    {

        // Retrieves the (first) device with the indicated name which is also an instance of the indicated class or interface.
        // First IMU is on the controller hub
        _imu = _hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        _imu.initialize(parameters);


        // Make sure your ID's match your configuration
        _frontLeft = _hardwareMap.dcMotor.get("frontLeft");
        _frontRight = _hardwareMap.dcMotor.get("frontRight");
        _backLeft = _hardwareMap.dcMotor.get("backLeft");
        _backRight = _hardwareMap.dcMotor.get("backRight");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        if (logoFacingDirection == RevHubOrientationOnRobot.LogoFacingDirection.DOWN) {
            _frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            _backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            _frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            _backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramLsx);
        _logger.addParameter(_paramLsr);
        _logger.addParameter(_paramRsx);

        resetYaw();
    }

    public void resetYaw()
    {
        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        _imu.resetYaw();
        _logger.updateStatus("IMU reset yaw");
        _logger.updateAll();
    }

    // Field-Centric code
    // Code is based of https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
    public void manualDrive()
    {
        _lsx = _gamepad1.left_stick_x;
        _lsy = _gamepad1.left_stick_y;
        _rsx = _gamepad1.right_stick_x;

        _logger.updateParameter(_paramLsx, _lsx);
        _logger.updateParameter(_paramLsr, _lsy);
        _logger.updateParameter(_paramRsx,_rsx);
        _logger.updateAll();


        double y = -_lsy;  // Remember, Y stick value is reversed
        double x = _lsx;
        double rx = _rsx;

        double botHeading = _imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * _strafeMagnitude;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        _frontLeft.setPower(frontLeftPower * _powerFactor);
        _backLeft.setPower(backLeftPower * _powerFactor);
        _frontRight.setPower(frontRightPower * _powerFactor);
        _backRight.setPower(backRightPower * _powerFactor);
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void autoDrive(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        _frontLeft.setPower(leftFrontPower);
        _frontRight.setPower(rightFrontPower);
        _backLeft.setPower(leftBackPower);
        _backRight.setPower(rightBackPower);
    }
}
