package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MecanumRobotTeleop")
public class MecanumRobotTeleop extends LinearOpMode {

    ERCParameterLogger _logger;
    MecanumDrive _mecanumDrive;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        //******************************
        // Main loop
        //******************************
        waitForStart();
        while (!isStopRequested())
        {
            _mecanumDrive.manualDrive();
            updateLogAndTelemetry();
        }
    }


    public void updateLogAndTelemetry() {
        double currentTime = getRuntime();
        String statusMsg = String.format("Current runtime = %.1f\n", currentTime);
//        statusMsg += String.format("Current runtime = %.1f\n", currentTime);
//        statusMsg += String.format("Current runtime = %.1f\n", currentTime);
        _logger.updateStatus(statusMsg);
        _logger.updateAll();
    }

    private void initRobot() {

        _logger = new ERCParameterLogger(this);

        _mecanumDrive = new MecanumDrive(this, _logger,
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,
                null);
    }

}
