package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

@TeleOp(name = "Calibration")
@Disabled
public class Calibration extends LinearOpMode {

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
