package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "HolyCrabAuto")
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
    //    I2C port3: "navx"         (navX2-Micro: 6-axis LSM6DSM IMU and LIS2MDL Magnetometer)
    // Expansion Hub:
    //    Motor port0: "armLeft"    (GoBILDA 5202/3/4 series)
    //    Motor port1: "armRight"   (GoBILDA 5202/3/4 series)
    //    I2C port0: "imu2"         (navX2-Micro)

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

        List<ERCAutoInstructions> instructions = new ArrayList<>();
        instructions.add(new ERCAutoInstructions(0.0, 0.5, 0.1, 300, false, 12.0));
        instructions.add(new ERCAutoInstructions(0.0, 0.0, 0.0, 1000, false, 12.0));
        instructions.add(new ERCAutoInstructions(0.0, 0.0, 0.0, 1000, true, 12.0));


        //******************************
        // Main loop
        //******************************
        boolean isDone = false;
        waitForStart();
        while (!isStopRequested())
        {
            if (isDone)
                continue;

            for (ERCAutoInstructions step : instructions) {
                if (step.alignToAprilTag)
                    _mecanumDrive.autoDriveAlign(-1, step.alignDistanceInches, true);
                else {
                    _mecanumDrive.autoDrive(step.forwardPower, step.leftPower, step.turnCounterClkPower);
                    sleep(step.sleepMs);
                }
            }

            isDone = true;

        }
    }



    public void updateLogAndTelemetry() {
        double currentTime = getRuntime();
        String statusMsg = String.format("Current runtime = %.1f\n", currentTime);
        _logger.updateStatus(statusMsg);
    }

    private void initRobot() throws InterruptedException {

        // Note: this is where you change the default configurations for your robot.
        _glbConfig.useNavxImu = true;
        _glbConfig.enableVisionColorSensor = true;

        _logger = new ERCParameterLogger(this);
        _vision = new ERCVision(this, _logger);
//        _color = new ERCColorSensor(this, _logger);
//        _touch = new ERCTouchSensor(this, _logger);
        _mecanumDrive = new ERCMecanumDrive(this, _logger, _vision);
        _arm = new ERCArm(this, _logger);
        //_led = new ERCLed(this, _logger);
    }

}
