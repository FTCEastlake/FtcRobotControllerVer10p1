package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "HolyCrabTeleop")
public class HolyCrabTeleop extends LinearOpMode {

    //**************************************************************
    // HolyCrab hardware configuration:
    // Control Hub:
    //    USB port: "webcam 1"
    //    Motor port0: "frontLeft"  (GoBILDA 5202/3/4 series)
    //    Motor port1: "frontRight" (GoBILDA 5202/3/4 series)
    //    Motor port2: "backLeft"   (GoBILDA 5202/3/4 series)
    //    Motor port3: "backRight"  (GoBILDA 5202/3/4 series)
    //    Servo port0: "blinkin led"
    //    I2C port0: "imu"          (REV internal IMU (BHI260AP))
    //    I2C port1: "revColorV3"   (REV color sensor V3)
    //    I2C port3: "navx"         (navX2-Micro: 6-axis LSM6DSM IMU and LIS2MDL Magnetometer)
    // Expansion Hub:
    //    Motor port0: "slideLeft"  (GoBILDA 5202/3/4 series)
    //    Motor port1: "slideRight" (GoBILDA 5202/3/4 series)
    //    I2C port0: "imu2"         (navX2-Micro)
    //    Servo port0: "armServo"
    //    Servo port1: "clawServo"



    ERCGlobalConfig _glbConfig = ERCGlobalConfig.getInstance();

    ERCParameterLogger _logger;
    ERCMecanumDrive _mecanumDrive;
    ERCVision _vision;
    ERCArm _arm;
    ERCLed _led;
//    ERCColorSensor _color;
//    ERCTouchSensor _touch;



    // This is where you can configure the specific of your individual robot.
    private void SetConfig() {

        // Note: this is where you change the default configurations for your robot.

        // Vision color detection 12 inches from camera
        _glbConfig.enableVisionColorSensor = true;
        _glbConfig.visionImageRegionLeft = 0.02;    // 0 = center, -1 = left
        _glbConfig.visionImageRegionRight = 0.18;    // 0 = center, +1 = right
        _glbConfig.visionImageRegionTop = -0.02;      // 0 = center, +1 = top
        _glbConfig.visionImageRegionBottom = -0.25; // 0 = center, -1 = bottom

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();


        while (!isStarted())
        {
            // The yaw is reset during the initialization of MecanumDrive class.
            // Do we ever need to reset the yaw during TeleOp mode? Most likely not.
            // Uncomment the code below if we ever need to reset yaw during TeleOp mode.
//            if (gamepad1.start)
//                _mecanumDrive.resetYaw();

            _arm.setArm();
            DetectColor();

        }

        //******************************
        // Main loop
        //******************************
        waitForStart();
        while (!isStopRequested())
        {
            //***********************************
            // Drive
            //***********************************
            if (gamepad1.right_bumper)
                _mecanumDrive.autoDriveAlign(-1, 12, false);
            else
                _mecanumDrive.manualDrive();

            //***********************************
            // Arm
            //***********************************
            _arm.setArm();
            DetectColor();
        }
    }


    ElapsedTime _ledHeartBeat = new ElapsedTime();
    private boolean _ledIdleBlack = true;
    private void DetectColor() {

        _vision.DetectColor();
        if (_vision.isColorDetectedRed()) _led.setLedColorRed();
        else if (_vision.isColorDetectedBlue()) _led.setLedColorBlue();
        else if (_vision.isColorDetectedYellow()) _led.setLedColorGreen();
        else if (_ledHeartBeat.milliseconds() >= 500) {
            if (_ledIdleBlack)
                _led.setLedColorWhite();
            else
                _led.setLedColorBlack();
            _ledIdleBlack = !_ledIdleBlack;
            _ledHeartBeat.reset();
        }
    }



    private void initRobot() throws InterruptedException {

        SetConfig();
        _logger = new ERCParameterLogger(this);
        _vision = new ERCVision(this, _logger);
        _mecanumDrive = new ERCMecanumDrive(this, _logger, _vision, false);
        _arm = new ERCArm(this, _logger);
        _led = new ERCLed(this, _logger);
//        _color = new ERCColorSensor(this, _logger);
//        _touch = new ERCTouchSensor(this, _logger);
    }

//    private void updateLogAndTelemetry() {
//        double currentTime = getRuntime();
//        String statusMsg = String.format("Current runtime = %.1f\n", currentTime);
//        _logger.updateStatus(statusMsg);
//    }

}
