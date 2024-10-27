package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    // Expansion Hub:
    //    Motor port0: "armLeft"    (GoBILDA 5202/3/4 series)
    //    Motor port1: "armRight"   (GoBILDA 5202/3/4 series)
    //    I2C port0: "imu2"         (REV internal IMU (BNO055))

    ERCParameterLogger _logger;
    MecanumDrive _mecanumDrive;
    ERCArm _arm;
    ERCLed _led;
    ERCColorSensor _color;
    ERCTouchSensor _touch;




    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        while (!isStarted())
        {
            _arm.setArm();
            _led.setLedColor();
        }

        //******************************
        // Main loop
        //******************************
        waitForStart();
        while (!isStopRequested())
        {
            // The yaw is reset during the initialization of MecanumDrive class.
            // Do we ever need to reset the yaw during TeleOp mode? Most likely not.
            // Uncomment the code below if we ever need to reset yaw during TeleOp mode.
//            if (gamepad1.start)
//                _mecanumDrive.resetYaw();

            _mecanumDrive.manualDrive();
            updateLogAndTelemetry();
//            _color.getColor();
        }
    }



    public void updateLogAndTelemetry() {
        double currentTime = getRuntime();
        String statusMsg = String.format("Current runtime = %.1f\n", currentTime);
        _logger.updateStatus(statusMsg);
    }

    private void initRobot() {

        _logger = new ERCParameterLogger(this);
//        _color = new ERCColorSensor(this, _logger);
//        _touch = new ERCTouchSensor(this, _logger);
        _mecanumDrive = new MecanumDrive(this, _logger,
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,
                null);
        _mecanumDrive.setRobotType(MecanumDrive.RobotType.HolyCrab);
        _arm = new ERCArm(this, _logger);
        _led = new ERCLed(this, _logger);
    }

}
