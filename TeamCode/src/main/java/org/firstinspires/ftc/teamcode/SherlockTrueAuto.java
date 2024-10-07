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

@TeleOp(name = "SherlockTrueAuto")
public class SherlockTrueAuto extends LinearOpMode {

    ERCParameterLogger _logger;
    MecanumDrive _mecanumDrive;
    ERCVision _vision;

    int _tagID = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();


        // Wait for driver to press start
        _logger.updateStatus("Use 3 dots on upper right corner for camera stream preview on/off");
        _logger.updateAll();

        int exposureMs = 6;
        int gain = 250;
        double distance = 12.0;
        while (!isStarted())
        {
//            boolean needAdjustment = false;
//            if (gamepad1.x) { exposureMs += 1; needAdjustment = true; }
//            else if (gamepad1.a) { exposureMs -= 1; needAdjustment = true; }
//            else if (gamepad1.y) { gain += 1; needAdjustment = true; }
//            else if (gamepad1.b) { gain -= 1; needAdjustment = true; }
//            if (needAdjustment)
//                _vision.setManualExposure(exposureMs, gain);

            if (gamepad1.x) {
                distance += 1.0;
                _logger.updateStatus("distance = " + distance + " inches");
                _logger.updateAll();
            }
            else if (gamepad1.a) {
                distance -= 1.0;
                _logger.updateStatus("sleepMsX = " + distance + " inches");
                _logger.updateAll();
            }
            sleep(200); // debounce
        }

        //******************************
        // Main loop
        //******************************
        waitForStart();

        double forwardPower = 0, leftPower = 0, turnCounterClk = 0;
        long sleepMs = 0;
        boolean enableAuto = true;

        String msg = "";
        while (!isStopRequested())
        {
            if (enableAuto)
            {
                // Simulate pre-loaded specimen
                forwardPower = 0.0; leftPower = 0.5; turnCounterClk = 0.1; _mecanumDrive.autoDrive(forwardPower, leftPower, turnCounterClk);
                sleepMs = 300; sleep(sleepMs);
                _mecanumDrive.autoDrive(0, 0, 0);
                sleepMs = 1000; sleep(sleepMs); // simulate hanging of the specimen

                // Align to AprilTag
                _mecanumDrive.autoDriveAlign(-1, distance);

                // Turn 180 degrees
                forwardPower = 0.0; leftPower = 0.0; turnCounterClk = 0.5; _mecanumDrive.autoDrive(forwardPower, leftPower, turnCounterClk);
                sleepMs = 625; sleep(sleepMs);
                _mecanumDrive.autoDrive(0, 0, 0);
                sleepMs = 1000; sleep(sleepMs);

                // Move to back left a little
                forwardPower = 0.25; leftPower = -0.25; turnCounterClk = 0.0; _mecanumDrive.autoDrive(forwardPower, leftPower, turnCounterClk);
                sleepMs = 500; sleep(sleepMs);
                _mecanumDrive.autoDrive(0, 0, 0);
                sleepMs = 1000; sleep(sleepMs);

                // Turn 180 degrees
                forwardPower = 0.0; leftPower = 0.0; turnCounterClk = 0.5; _mecanumDrive.autoDrive(forwardPower, leftPower, turnCounterClk);
                sleepMs = 700; sleep(sleepMs);
                _mecanumDrive.autoDrive(0, 0, 0);
                sleepMs = 1000; sleep(sleepMs);

                // Align to AprilTag
                _mecanumDrive.autoDriveAlign(-1, distance);


//                // Back out a foot, turn to the right and detect and align to AprilTag 14
//                forwardPower = -0.5; leftPower = 0; turnCounterClk = 0;
//                _mecanumDrive.autoDrive(forwardPower, leftPower, turnCounterClk);
//                sleepMs = 150; sleep(sleepMs);
//                forwardPower = 0.0; leftPower = 0; turnCounterClk = -0.6;
//                _mecanumDrive.autoDrive(forwardPower, leftPower, turnCounterClk);
//                sleepMs = 200; sleep(sleepMs);
//                forwardPower = 0.0; leftPower = 0; turnCounterClk = 0;
//                _mecanumDrive.autoDrive(forwardPower, leftPower, turnCounterClk);
//                sleepMs = 500; sleep(sleepMs);





                //ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ
//                // Align to AprilTag
//                boolean alignToAprilTag = true;
//                while (alignToAprilTag == true)
//                {
//                    AprilTagDetection foundTag = _vision.detectAprilTag(_tagID);
//                    if (foundTag == null)
//                        continue;
//
//
//
//                    foundTag = _vision.detectAprilTag(_tagID);
//                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
//                    // To understand ftcPose range/bearing/yaw parameters see the following link:
//                    // https://ftc-docs.firstinspires.org/en/latest/apriltag/understanding_apriltag_detection_values/understanding-apriltag-detection-values.html
//                    double  rangeError      = (foundTag.ftcPose.range - DESIRED_DISTANCE);      // range is how far away the camera is to the AprilTag
//                    double  headingError    = foundTag.ftcPose.bearing;                         // bearing is how degrees to the left (+val) or right (-val)
//                    double  yawError        = foundTag.ftcPose.yaw;                             // rotation of tag on the z-axis
//
//                    // Use the speed and turn "gains" to calculate how we want the robot to move.
//                    drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//                    turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
//                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//
//                    msg = String.format("Auto drive %5.2f, Strafe %5.2f, Turn %5.2f \n", drive, strafe, turn);
//                    msg += String.format("rangeError %5.2f, headingError %5.2f, yawError %5.2f \n", rangeError, headingError, yawError);
//                    _logger.updateStatus(msg);
//                    _logger.updateAll();
//                    _mecanumDrive.autoDrive(drive, strafe, turn);
//
//                    while (gamepad1.y == false)
//                        sleep(100);
//                }
                //sleepMs = 500; sleep(sleepMs);

                // Turn to the right and detect and align to AprilTag 14
//                forwardPower = 0.0; leftPower = 0; turnCounterClk = -0.6; _mecanumDrive.autoDrive(forwardPower, leftPower, turnCounterClk);
//                sleepMs = 400; sleep(sleepMs);
//                forwardPower = 0.0; leftPower = 0; turnCounterClk = 0; _mecanumDrive.autoDrive(forwardPower, leftPower, turnCounterClk);
//                sleepMs = 500; sleep(sleepMs);

                enableAuto = false;
            }

            boolean y = gamepad1.y;

        }
    }



    private void initRobot() {

        _logger = new ERCParameterLogger(this);
        _vision = new ERCVision(this, _logger);
        _mecanumDrive = new MecanumDrive(this, _logger,
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,
                _vision);
    }

}
