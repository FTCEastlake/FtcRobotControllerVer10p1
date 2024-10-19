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
    ERCSettings _settings;

    int _tagID = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();
        double distance = 12.0;
        //calibrateRobot();


        //******************************
        // Main loop
        //******************************
        waitForStart();

        _vision.ColorDetectionEnable(false);    // turn off color detection

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
                sleepMs = 600; sleep(sleepMs);
                _mecanumDrive.autoDrive(0, 0, 0);
                sleepMs = 1000; sleep(sleepMs);

                // Turn 180 degrees
                forwardPower = 0.0; leftPower = 0.0; turnCounterClk = 0.5; _mecanumDrive.autoDrive(forwardPower, leftPower, turnCounterClk);
                sleepMs = 500; sleep(sleepMs);
                _mecanumDrive.autoDrive(0, 0, 0);
                sleepMs = 1000; sleep(sleepMs);

                // Align to AprilTag
                _mecanumDrive.autoDriveAlign(-1, distance);



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
        _settings = new ERCSettings(this, _logger, _vision);
    }

    private void calibrateRobot()
    {

        // This function still needs tod be worked on.

        // Wait for driver to press start
        _logger.updateStatus("Use 3 dots on upper right corner for camera stream preview on/off");
        _logger.updateAll();

        int exposureMs = 6;
        int gain = 250;

        while (!isStarted())
        {
            _settings.ConfigSettings();
//            boolean needAdjustment = false;
//            if (gamepad1.x) { exposureMs += 1; needAdjustment = true; }
//            else if (gamepad1.a) { exposureMs -= 1; needAdjustment = true; }
//            else if (gamepad1.y) { gain += 1; needAdjustment = true; }
//            else if (gamepad1.b) { gain -= 1; needAdjustment = true; }
//            if (needAdjustment)
//                _vision.setManualExposure(exposureMs, gain);

//            if (gamepad1.x) {
//                distance += 1.0;
//                _logger.updateStatus("distance = " + distance + " inches");
//                _logger.updateAll();
//            }
//            else if (gamepad1.a) {
//                distance -= 1.0;
//                _logger.updateStatus("sleepMsX = " + distance + " inches");
//                _logger.updateAll();
//            }
//            sleep(200); // debounce
//
//            _vision.ColorDetection(true, true, true);
        }
    }

}
