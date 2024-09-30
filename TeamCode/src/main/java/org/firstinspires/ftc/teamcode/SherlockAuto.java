package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "SherlockAuto")
public class SherlockAuto extends LinearOpMode {

    ERCParameterLogger _logger;
    MecanumDrive _mecanumDrive;
    ERCVision _vision;

    String _paramLsx = "Left Stick X";
    String _paramLsr = "Left Stick Y";
    String _paramRsx = "Right Stick X";


    //shorten controller values for readability, values are "float" data type internally
    float _lsx = 0;      //lsx = left stick x
    float _lsy = 0;      //lsy = right stick y
    float _rsx = 0;     //rsx = right stick x
    boolean _startButton = false;
    double _strafeMagnitude = 0; //find stick distance from 0

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    @Override
    public void runOpMode() throws InterruptedException {

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        initRobot();
        _vision.setManualExposure(6, 250);

        // Wait for driver to press start
        _logger.updateStatus("Use 3 dots on upper right corner for camera stream preview on/off");
        _logger.updateAll();

        //******************************
        // Main loop
        //******************************
        waitForStart();
        while (!isStopRequested())
        {
            AprilTagDetection foundTag = _vision.detectAprilTag(11);
            if (foundTag != null && gamepad1.left_bumper)
            {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (foundTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = foundTag.ftcPose.bearing;
                double  yawError        = foundTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                String msg = String.format("Auto drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                _logger.updateStatus(msg);
                _logger.updateAll();
                _mecanumDrive.autoDrive(drive, strafe, turn);
            }
            else {
                //update stick values
                _lsx = gamepad1.left_stick_x / 2.0f;
                _lsy = gamepad1.left_stick_y / 2.0f;
                _rsx = gamepad1.right_stick_x / 3.0f;
                _startButton = gamepad1.start;
                _mecanumDrive.drive(_lsx, _lsy, _rsx, _startButton);
                updateLogAndTelemetry();
            }
        }
    }



    public void updateLogAndTelemetry() {
        double currentTime = getRuntime();
//        String statusMsg = String.format("Current runtime = %.1f\n", currentTime);
//        statusMsg += String.format("Current runtime = %.1f\n", currentTime);
//        statusMsg += String.format("Current runtime = %.1f\n", currentTime);
//        _logger.updateStatus(statusMsg);
        _logger.updateParameter(_paramLsx, _lsx);
        _logger.updateParameter(_paramLsr, _lsy);
        _logger.updateParameter(_paramRsx, _rsx);
        _logger.updateAll();
    }

    private void initRobot() {

        _logger = new ERCParameterLogger(this);
        _logger.init();

        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramLsx);
        _logger.addParameter(_paramLsr);
        _logger.addParameter(_paramRsx);

        _mecanumDrive = new MecanumDrive(this, _logger,
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        _vision = new ERCVision(this, _logger);
    }

}
