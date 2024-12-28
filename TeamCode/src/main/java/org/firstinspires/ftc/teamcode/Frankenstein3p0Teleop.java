package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Frankenstein3p0Teleop")
public class Frankenstein3p0Teleop extends LinearOpMode {

    //**************************************************************
    // Frankenstein 3.0 hardware configuration:
    // Control Hub:
    //    USB port: "webcam 1"
    //    Motor port0: "frontLeft"  (GoBILDA 5202/3/4 series)
    //    Motor port1: "frontRight" (GoBILDA 5202/3/4 series)
    //    Motor port2: "backLeft"   (GoBILDA 5202/3/4 series)
    //    Motor port3: "backRight"  (GoBILDA 5202/3/4 series)
    //    Servo port0: "blinkin led"
    //    I2C port0: "imu"          (REV internal IMU (BHI260AP))
    //    I2C port1: "odo4bar"      (GoBILDA 4-Bar odometry)




    ERCGlobalConfig _glbConfig = ERCGlobalConfig.getInstance();

    ERCParameterLogger _logger;
    ERCDrivetrain3p0 _drive;
    ERCGobilda4Bar _odometry;
    ERCVision _vision;
//    ERCArm _arm;
//    ERCLed _led;
//    ERCColorSensor _color;
//    ERCTouchSensor _touch;


    private ElapsedTime _debounce = new ElapsedTime();

    // This is where you can configure the specific of your individual robot.
    private void SetConfig() {

        //***********************************************************************************
        // Note: this is where you can OVERWRITE the default configurations for your robot.
        //***********************************************************************************
        _glbConfig.robotType = ERCGlobalConfig.RobotType.Standard;

        // Drivetrain
        _glbConfig.drivePowerFast = 0.75;
        _glbConfig.drivePowerNormal = 0.50;
        _glbConfig.drivePowerSlow = 0.25;

        // Vision parameters
        _glbConfig.visionImageRegionLeft = -0.1;     // 0 = center, -1 = left
        _glbConfig.visionImageRegionRight = 0.1;     // 0 = center, +1 = right
        _glbConfig.visionImageRegionTop = 0.1;       // 0 = center, +1 = top
        _glbConfig.visionImageRegionBottom = -0.1;   // 0 = center, -1 = bottom
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        while (!isStarted())
        {
            // The yaw is reset during the initialization of MecanumDrive class.
            // Do we ever need to reset the yaw during TeleOp mode? Most likely not.
            // Uncomment the code below if we ever need to reset yaw during TeleOp mode.
            if (gamepad1.start)
                _drive.resetYaw();

            double autoDrivePower = 0.35;
//            if (gamepad1.x)
//                _drive.autoMove4Bar(0.0, -10.0, 0, autoDrivePower);
//            else if (gamepad1.b)
//                _drive.autoMove4Bar(0.0, 10.0, 0, autoDrivePower);
//            else if (gamepad1.y)
//                _drive.autoMove4Bar(10.0, 0.0, 0, autoDrivePower);
//            else if (gamepad1.a)
//                _drive.autoMove4Bar(-10.0, 0.0, 0, autoDrivePower);
//            else if (gamepad1.left_bumper)
//                _drive.autoMove4Bar(0.0, 0.0, -90, autoDrivePower);
//            else if (gamepad1.right_bumper)
//                _drive.autoMove4Bar(0.0, 0.0, 90, autoDrivePower);


            if (gamepad1.x)
                _drive.autoMovePIDRight(-10.0, autoDrivePower);
            else if (gamepad1.b)
                _drive.autoMovePIDRight(10.0, autoDrivePower);
            else if (gamepad1.y)
                _drive.autoMovePIDForward(10.0, autoDrivePower);
            else if (gamepad1.a)
                _drive.autoMovePIDForward(-10.0, autoDrivePower);
            else if (gamepad1.left_bumper)
                _drive.autoMovePIDTurn(-90, autoDrivePower);
            else if (gamepad1.right_bumper)
                _drive.autoMovePIDTurn(90, autoDrivePower);

        }

        double drivePower, loopCount = 0.0;

        //******************************
        // Main loop
        //******************************
        waitForStart();
        _logger.resetCycleTimer();
        while (!isStopRequested())
        {
//            if (gamepad1.left_bumper && (_debounce.milliseconds() > 500)) {
//                // switch back and forth with each button press
//                drivePower =  drivePower == _glbConfig.drivePowerFast ?
//                      _glbConfig.drivePowerNormal: _glbConfig.drivePowerFast;
//                _debounce.reset();
//            }
            drivePower = gamepad1.left_bumper ? _glbConfig.drivePowerFast :
                    gamepad1.right_bumper ? _glbConfig.drivePowerSlow : _glbConfig.drivePowerNormal;


//            if (gamepad1.y)
//                _drive.autoMove(10.0, 0.0, 0.0, _glbConfig.drivePowerSlow);
//            else if (gamepad1.a)
//                _drive.autoMove(-10.0, 0.0, 0.0, _glbConfig.drivePowerSlow);
//            else if (gamepad1.x)
//                _drive.autoMove(0.0, -10.0, 0.0, _glbConfig.drivePowerSlow);
//            else if (gamepad1.b)
//                _drive.autoMove(0.0, 10.0, 0.0, _glbConfig.drivePowerSlow);

            if (gamepad1.x)
                _drive.autoMove(0.0, 0.0, -1.0, _glbConfig.drivePowerSlow);
            else if (gamepad1.b)
                _drive.autoMove(0.0, 0.0, 1.0, _glbConfig.drivePowerSlow);
            else if (gamepad1.dpad_up)
                _drive.move(1.0, 0.0, 0.0, _glbConfig.drivePowerSlow);
            else if (gamepad1.dpad_down)
                _drive.move(-1.0, 0.0, 0.0, _glbConfig.drivePowerSlow);
            else if (gamepad1.dpad_left)
                _drive.move(0.0, -1.0, 0.0, _glbConfig.drivePowerSlow);
            else if (gamepad1.dpad_right)
                _drive.move(0.0, 1.0, 0.0, _glbConfig.drivePowerSlow);
            else
                _drive.move(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, drivePower);

            _odometry.getPosition();

            _vision.DetectColor();
            _logger.updateStatus("color detect: red = " + (_vision.isColorDetectedRed() ? "true" : "false") +
                    ", blue = " + (_vision.isColorDetectedBlue() ? "true" : "false") +
                    ", yellow = " + (_vision.isColorDetectedYellow() ? "true" : "false"));
            _logger.updateAll();

            loopCount++;
            _logger.updateCycleTimer(loopCount);
        }
    }




    private void initRobot() throws InterruptedException {

        SetConfig();
        _logger = new ERCParameterLogger(this, true);
        _vision = new ERCVision(this, _logger);


        //****************************************************************************************
        // For 4-bar, we are considering front center of robot frame to be our arbitrary reference point (in millimeters)
        //****************************************************************************************
        double xPodOffsetMM = -166.6875;    // X-pod offset (left/right) of arbitrary point, negative value for right of arbitrary point
        double yPodOffsetMM = -169.8625;    // Y-pod offset (front/behind) of arbitrary point, negative value for behind arbitrary point
        _odometry = new ERCGobilda4Bar(this, _logger, true, xPodOffsetMM, yPodOffsetMM);

        _drive = new ERCDrivetrain3p0(this, _logger, _odometry, _vision, true);
    }



}
