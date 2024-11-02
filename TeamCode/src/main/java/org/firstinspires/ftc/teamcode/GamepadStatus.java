package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

@TeleOp(name = "GamepadStatus")
public class GamepadStatus extends LinearOpMode {

    ERCParameterLogger _logger;
    private Gamepad _gamepad1;
    private Gamepad _gamepad2;

    private String _paramGP1ID = "Gamepad1 ID";
    private String _paramGP2ID = "Gamepad2 ID";
    private String _paramGPDelimiter1 = "//*";
//    private String _paramGP1Lsx = "Gamepad1 Left  Stick X";
//    private String _paramGP1Lsy = "Gamepad1 Left  Stick Y";
//    private String _paramGP1Rsx = "Gamepad1 Right Stick X";
//    private String _paramGP1Rsy = "Gamepad1 Right Stick Y";
//    private String _paramGPDelimiter2 = "//-";
//    private String _paramGP2Lsx = "Gamepad2 Left  Stick X";
//    private String _paramGP2Lsy = "Gamepad2 Left  Stick Y";
//    private String _paramGP2Rsx = "Gamepad2 Right Stick X";
//    private String _paramGP2Rsy = "Gamepad2 Right Stick Y";

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();



        //******************************
        // Main loop
        //******************************
        waitForStart();
        while (!isStopRequested())
        {
            _logger.updateParameter(_paramGP1ID, gamepad1.toString());
            _logger.updateParameter(_paramGP2ID, gamepad2.toString());
//            _logger.updateParameter(_paramGP1Lsx, gamepad1.left_stick_x);
//            _logger.updateParameter(_paramGP1Lsy, gamepad1.left_stick_y);
//            _logger.updateParameter(_paramGP1Rsx, gamepad1.right_stick_x);
//            _logger.updateParameter(_paramGP1Rsy, gamepad1.right_stick_y);
//
//            _logger.updateParameter(_paramGP2Lsx, gamepad2.left_stick_x);
//            _logger.updateParameter(_paramGP2Lsy, gamepad2.left_stick_y);
//            _logger.updateParameter(_paramGP2Rsx, gamepad2.right_stick_x);
//            _logger.updateParameter(_paramGP2Rsy, gamepad2.right_stick_y);
            _logger.updateAll();
        }
    }

    private void initRobot() throws InterruptedException {

        _logger = new ERCParameterLogger(this);

        _logger.addParameter(_paramGP1ID);
        _logger.addParameter(_paramGPDelimiter1);
        _logger.addParameter(_paramGP2ID);
//        _logger.addParameter(_paramGP1Lsx);
//        _logger.addParameter(_paramGP1Lsy);
//        _logger.addParameter(_paramGP1Rsx);
//        _logger.addParameter(_paramGP1Rsy);
//        _logger.addParameter(_paramGPDelimiter2);
//        _logger.addParameter(_paramGP2Lsx);
//        _logger.addParameter(_paramGP2Lsy);
//        _logger.addParameter(_paramGP2Rsx);
//        _logger.addParameter(_paramGP2Rsy);

        _logger.updateParameter(_paramGPDelimiter1, "-----------------------");
//        _logger.updateParameter(_paramGPDelimiter2, "-----------------------");
        _logger.updateAll();
    }

}
