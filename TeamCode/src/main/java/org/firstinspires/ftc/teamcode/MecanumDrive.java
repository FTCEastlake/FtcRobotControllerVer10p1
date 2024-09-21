package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {

    LinearOpMode _opMode;
    HardwareMap _hardwareMap;
    ERCParameterLogger _logger;

    // Declare motors
    DcMotor _frontLeft = null;
    DcMotor _frontRight = null;
    DcMotor _backLeft = null;
    DcMotor _backRight = null;

    double _strafeMagnitude = 1.1;
    IMU _imu;

    public MecanumDrive(LinearOpMode opMode, ERCParameterLogger logger){

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _logger = logger;

        // Retrieves the (first) device with the indicated name which is also an instance of the indicated class or interface.
        // First IMU is on the controller hub
        _imu = _hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
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
        // See the note about this earlier on this page.
        _frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        _backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Field-Centric code
    // Code is based of https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
    public void drive(double leftStickX, double leftStickY, double rightStickX, boolean resetYaw)
    {
//        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double x = gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x;

        double y = -leftStickY;  // Remember, Y stick value is reversed
        double x = leftStickX;
        double rx = rightStickX;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        //if (gamepad1.options)
        if (resetYaw) {
            _imu.resetYaw();
        }

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

        _frontLeft.setPower(frontLeftPower);
        _backLeft.setPower(backLeftPower);
        _frontRight.setPower(frontRightPower);
        _backRight.setPower(backRightPower);
    }
}
