package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp(name = "Frankenstein")
public class Frankenstein extends LinearOpMode {

    ERCParameterLogger _logger = new ERCParameterLogger(this);
    DcMotor m0 = null;
    DcMotor m1 = null;
    DcMotor m2 = null;
    DcMotor m3 = null;

    float lsy = 0;      //lsy = right stick y

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        //******************************
        // Main loop
        //******************************
        waitForStart();

        DcMotor motorX = hardwareMap.get(DcMotorEx.class, "motor3");
        if (motorX == null)
            _logger.updateStatus("motorX is not of type DcMotorEx!");
        else
            _logger.updateStatus("motorX is good");
        _logger.updateAll();


        motorX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorX.setDirection(DcMotorEx.Direction.FORWARD);


        String statusMsg;
        while (!isStopRequested())
        {
            //sleep(100);
            statusMsg = String.format("motorX power = %.2f\n", lsy);
            lsy = gamepad1.left_stick_y;
            _logger.updateStatus(statusMsg);
            _logger.updateAll();
            motorX.setPower(lsy);
        }

        motorX.setPower(0);

    }

    private void initRobot() {

        // Add all of the parameters you want displayed on the driver hub
        //_logger.addStatus();
        _logger.init();

//        // Add all of the parameters you want to see on the driver hub display.
//        _logger.addParameter(_paramLsx, 1);
//        _logger.addParameter(_paramLsr, 2);
//        _logger.addParameter(_paramRsx, 3);
    }
}
