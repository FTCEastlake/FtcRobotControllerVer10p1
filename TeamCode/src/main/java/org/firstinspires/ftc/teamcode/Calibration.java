package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

@TeleOp(name = "Calibration")
//@Disabled
public class Calibration extends LinearOpMode {

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

    // Declare motors
    private DcMotorEx _frontLeft = null;
    private DcMotorEx _frontRight = null;
    private DcMotorEx _backLeft = null;
    private DcMotorEx _backRight = null;


    ERCParameterLogger _logger;
    private Gamepad _gamepad1;
    private Gamepad _gamepad2;

    private String _paramGP1ID = "Gamepad1 ID";
    private String _paramGP2ID = "Gamepad2 ID";
    private String _paramGPDelimiter1 = "//*";
    private String _paramGPDelimiter2 = "//-";

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        while (!isStarted())
        {
            if (gamepad1.x) _frontLeft.setPower(0.15); else _frontLeft.setPower(0.0);
            if (gamepad1.y) _frontRight.setPower(0.15); else _frontRight.setPower(0.0);
            if (gamepad1.a) _backLeft.setPower(0.15); else _backLeft.setPower(0.0);
            if (gamepad1.b) _backRight.setPower(0.15); else _backRight.setPower(0.0);

            _logger.updateParameter(_paramGP1ID, gamepad1.toString());
            _logger.updateParameter(_paramGP2ID, gamepad2.toString());
            _logger.updateAll();
        }

        //******************************
        // Main loop
        //******************************
        waitForStart();
        while (!isStopRequested())
        {
            int x = 1;
        }
    }

    private void initRobot() throws InterruptedException {

        // Make sure your ID's match your configuration
        _frontLeft = (DcMotorEx)hardwareMap.dcMotor.get("frontLeft");
        _frontRight = (DcMotorEx)hardwareMap.dcMotor.get("frontRight");
        _backLeft = (DcMotorEx)hardwareMap.dcMotor.get("backLeft");
        _backRight = (DcMotorEx)hardwareMap.dcMotor.get("backRight");

        _logger = new ERCParameterLogger(this, false);

        _logger.addParameter(_paramGP1ID);
        _logger.addParameter(_paramGP2ID);
        _logger.addParameter(_paramGPDelimiter1);
        _logger.addParameter(_paramGPDelimiter2);

        _logger.updateParameter(_paramGPDelimiter1, "-----------------------");
        _logger.updateParameter(_paramGPDelimiter2, "-----------------------");
        _logger.updateAll();
    }

}
