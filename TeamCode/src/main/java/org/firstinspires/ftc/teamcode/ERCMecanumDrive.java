package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;

public class ERCMecanumDrive {

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
    private DcMotor _frontLeft = null;
    private DcMotor _frontRight = null;
    private DcMotor _backLeft = null;
    private DcMotor _backRight = null;

    private String _paramLsx = "Left Stick X";
    private String _paramLsr = "Left Stick Y";
    private String _paramRsx = "Right Stick X";

    //shorten controller values for readability, values are "float" data type internally
    private double _lsx = 0;    //lsx = left stick x
    private double _lsy = 0;    //lsy = left stick y
    private double _rsx = 0;    //rsx = right stick x

    private double _powerFactor = 1.0;     // 1.0 = full power, 0.0 = no power
    private double _strafeMagnitude = 1.1;

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    private final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    private final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    private final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    private final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    private final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    private final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    public ERCMecanumDrive(LinearOpMode opMode, ERCParameterLogger logger, ERCVision vision, boolean shouldResetYaw) throws InterruptedException {

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _gamepad1 = _opMode.gamepad1;
        _logger = logger;
        _vision = vision;

        init(shouldResetYaw);
    }

    private void init(boolean shouldResetYaw) throws InterruptedException {

        // Select the IMU to use.
        //_imu  = _glbConfig.useNavxImu ? new ERCNavxIMU(_opMode, _logger) : new ERCRevIMU(_opMode, _logger);
        _imu  = new ERCRevIMU(_opMode, _logger);

        // Make sure your ID's match your configuration
        _frontLeft = _hardwareMap.dcMotor.get("frontLeft");
        _frontRight = _hardwareMap.dcMotor.get("frontRight");
        _backLeft = _hardwareMap.dcMotor.get("backLeft");
        _backRight = _hardwareMap.dcMotor.get("backRight");


        switch (_glbConfig.robotType) {
            case HolyCrab:
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
                }
                else {
                    _frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                    _backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                break;  // nothing to alter
        }


        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramLsx);
        _logger.addParameter(_paramLsr);
        _logger.addParameter(_paramRsx);

        if (shouldResetYaw)
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

        double botHeading = _imu.getBotHeading();

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * _strafeMagnitude;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(rotY) + abs(rotX) + abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Apply final power
        _frontLeft.setPower(frontLeftPower * _powerFactor);
        _backLeft.setPower(backLeftPower * _powerFactor);
        _frontRight.setPower(frontRightPower * _powerFactor);
        _backRight.setPower(backRightPower * _powerFactor);
    }

    /**
     * Move robot according to desired axes motions
     * This is robot centric, not field centric, because it's the camera's point of view.
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
        double max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
        max = Math.max(max, abs(leftBackPower));
        max = Math.max(max, abs(rightBackPower));

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

    public void autoDriveAlign(int tagID, double distanceInches, boolean isAutonomousMode) {

        // Align to AprilTag
        String msg = "";
        boolean includeDistance = distanceInches > 0;
        boolean isAligned = false;
        double timeoutLimitSec = _opMode.getRuntime() + 2.0;
        while ((!isAligned) && (isAutonomousMode || _gamepad1.right_bumper))
        {
            AprilTagDetection foundTag = _vision.detectAprilTag(tagID);
            if (foundTag != null)
            {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                // To understand ftcPose range/bearing/yaw parameters see the following link:
                // https://ftc-docs.firstinspires.org/en/latest/apriltag/understanding_apriltag_detection_values/understanding-apriltag-detection-values.html
                double  rangeError      = foundTag.ftcPose.range - (includeDistance ? distanceInches : 0);      // range is how far away the camera is to the AprilTag
                double  headingError    = foundTag.ftcPose.bearing;                         // degrees to the left (+val) or right (-val) of camera
                double  yawError        = foundTag.ftcPose.yaw;                             // rotation of tag on the z-axis

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive  = includeDistance ? Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED) : 0;
                double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                msg = String.format("Auto drive %5.2f, Strafe %5.2f, Turn %5.2f, isAligned = %s \n", drive, strafe, turn, isAligned ? "true" : "false");
                msg += String.format("rangeError %5.2f, headingError %5.2f, yawError %5.2f \n", rangeError, headingError, yawError);
                _logger.updateStatus(msg);
                _logger.updateAll();

                // Apply appropriate power for 40ms then scale back to 33% of power to give vision time to process image for next calculation.
                // You can tweak the sleep time or power factor to optimize cycle time.
                autoDrive(drive, strafe, turn);
                //_opMode.sleep(30);      // 40ms is based on a pipeline of about 25-30ms and overhead of about 3ms
                _opMode.sleep(75);
                //double powerFactor = 0.33;
                double powerFactor = 2.0;
                autoDrive(drive * powerFactor, strafe * powerFactor, turn * powerFactor);

                // Reset timeout for next iteration
                //timeoutLimitSec = _opMode.getRuntime() + 2.0;
                timeoutLimitSec = 0;

                boolean withinLimitStrafe = abs(strafe) <= 0.06;
                boolean withinLimitsTurn = abs(turn) <= 0.02;
                if (withinLimitStrafe && withinLimitsTurn)
                {
                    //isAligned = (includeDistance && (rangeError > 2.0)) ? false : true;
                    isAligned = !includeDistance || (!(rangeError > 2.0));
                    msg = String.format("Auto drive %5.2f, Strafe %5.2f, Turn %5.2f, isAligned = %s \n", drive, strafe, turn, isAligned ? "true" : "false");
                    msg += String.format("rangeError %5.2f, headingError %5.2f, yawError %5.2f \n", rangeError, headingError, yawError);
                    _logger.updateStatus(msg);
                    _logger.updateAll();
                }
            }
            else
            {
                double currentTime = _opMode.getRuntime();
                autoDrive(0, 0, 0);
                _logger.updateStatus("timeout: foundTag == null, currSec = " + currentTime + ", timeoutLimitSec = " + timeoutLimitSec + "\n");
                _logger.updateAll();
                if(currentTime > timeoutLimitSec)
                    break;
            }


        }

        autoDrive(0.0, 0.0, 0.0);
    }
}
