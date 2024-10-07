package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SherlockTeleop")
public class SherlockTeleop extends LinearOpMode {

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
            // The yaw is reset during the initialization of MecanumDrive class.
            // Do we ever need to reset the yaw during TeleOp mode? Most likely not.
            // Uncomment the code below if we ever need to reset yaw during TeleOp mode.
//            if (gamepad1.start)
//                _mecanumDrive.resetYaw();

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
    }

    private void initRobot() {

        _logger = new ERCParameterLogger(this);

        _mecanumDrive = new MecanumDrive(this, _logger,
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,
                null);
    }

}
