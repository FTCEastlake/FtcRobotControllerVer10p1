package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "HolyCrabAuto")
//@TeleOp(name = "HolyCrabAuto")
public class HolyCrabAuto extends LinearOpMode {

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
    ERCColorSensor _color;
    ERCTouchSensor _touch;



    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        armUp();
        openClaw();
        while (!isStarted())
        {
            //setArmSlider(0.455);    //
            _arm.setArm();
        }


        //******************************
        // Main loop
        //******************************
        boolean isDone = false;
        waitForStart();
        while (!isStopRequested())
        {
            if (isDone)
                continue;


            //Drive(0.4, 0.0, 0.0, 300);
            //setArmSlider(1.1);
            _logger.updateStatus("Setting arm down"); _logger.updateAll();
            armDown();
            sleep(3000);
            _logger.updateStatus("Pick up sample"); _logger.updateAll();
            closeClaw();
            sleep(500);
            armUp();
            sleep(500);
            _logger.updateStatus("Move arm slider to max"); _logger.updateAll();
            setArmSlider(1.1);
            openClaw();

            // We need about 5.5 seconds to
            _logger.updateStatus("Move arm slider to min"); _logger.updateAll();
            ElapsedTime startTime = new ElapsedTime();
            _arm.setSliderToZero();
            double endTime = startTime.seconds();
            _logger.updateStatus("Done setSliderToZero, " + endTime + " seconds"); _logger.updateAll();

            isDone = true;

        }
    }

    private void Drive(double forwardPower, double leftPower, double turnCounterClkPower, long sleepMs) {
        _mecanumDrive.autoDrive(forwardPower, leftPower, turnCounterClkPower);
        sleep(sleepMs);
        _mecanumDrive.autoDrive(0.0, 0.0, 0.0);     // stop
    }

    private void setArmSlider(double height) {
        // Notes:
        // Estimated encoder LSB/sec is 2200/sec or 2.2/ms
        int sleepMs = (int)((_glbConfig.maxSliderEncoderVal * height) / 2.2) ;
        _arm.setSliderEncoder(1.0);
        sleep(sleepMs);
        _arm.setSliderEncoder(0);;     // stop
    }

    private void armUp() {
        _arm.setArmUp();
    }
    private void armDown() {
        _arm.setArmDown();
    }

    private void openClaw() {
        _arm.setClawOpen();
    }
    private void closeClaw() {
        _arm.setClawClose();
    }



//    public void updateLogAndTelemetry() {
//        double currentTime = getRuntime();
//        String statusMsg = String.format("Current runtime = %.1f\n", currentTime);
//        _logger.updateStatus(statusMsg);
//    }

    private void initRobot() throws InterruptedException {

        // Note: this is where you change the default configurations for your robot.
//        _glbConfig.useNavxImu = false;
//        _glbConfig.enableVisionColorSensor = true;

        _logger = new ERCParameterLogger(this);
        _vision = new ERCVision(this, _logger);
        _mecanumDrive = new ERCMecanumDrive(this, _logger, _vision, true);
        _arm = new ERCArm(this, _logger);

//        _led = new ERCLed(this, _logger);
    }

}
