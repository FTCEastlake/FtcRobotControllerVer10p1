package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MecanumRobotTeleop")
public class MecanumRobotTeleop extends LinearOpMode {

    ERCParameterLogger _logger;
    MecanumDrive _mecanumDrive;

    String _paramLsx = "Left Stick X";
    String _paramLsr = "Left Stick Y";
    String _paramRsx = "Right Stick X";

    //variables to track power of each motor
    double _flPwr = 0;
    double _frPwr = 0;
    double _blPower = 0;
    double _brPower = 0;

    //shorten controller values for readability, values are "float" data type internally
    float _lsx = 0;      //lsx = left stick x
    float _lsy = 0;      //lsy = right stick y
    float _rsx = 0;     //rsx = right stick x
    boolean _startButton = false;
    double _strafeMagnitude = 0; //find stick distance from 0

    //initialize gyro
    float _gyroYaw = 0;
    double _gyroCos = 0; //<-- cosine of gyro
    double _gyroSin = 0; //<-- sine of gyro

    //boolean for robot control type selection, true fo field centered, false for robot centered
    boolean fieldCenterControlSelect = true;

    DcMotor _armMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();
        _armMotor = hardwareMap.dcMotor.get("armMotor");
        //telemetry.setAutoClear(true);

        //******************************
        // Main loop
        //******************************
        waitForStart();
        while (!isStopRequested())
        {
            //update stick values
            _lsx = gamepad1.left_stick_x;
            _lsy = gamepad1.left_stick_y;
            _rsx = gamepad1.right_stick_x;
            _startButton = gamepad1.start;
            _armMotor.setPower(_lsx);
            //mecanumDrive.drive(_lsx, _lsy, _rsx, _startButton);
            updateLogAndTelemetry();


        }
    }



    public void updateLogAndTelemetry() {
        double currentTime = getRuntime();
        String statusMsg = String.format("Current runtime = %.1f\n", currentTime);
        statusMsg += String.format("Current runtime = %.1f\n", currentTime);
        statusMsg += String.format("Current runtime = %.1f\n", currentTime);
        _logger.updateStatus(statusMsg);
        _logger.updateParameter(_paramLsx, _lsx);
        _logger.updateParameter(_paramLsr, _lsy);
        _logger.updateParameter(_paramRsx, _rsx);
        _logger.updateAll();
    }

    private void initRobot() {

        _logger = new ERCParameterLogger(this);

        // Add all of the parameters you want displayed on the driver hub
        //_logger.addStatus();
        _logger.init();

        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramLsx, 1);
        _logger.addParameter(_paramLsr, 2);
        _logger.addParameter(_paramRsx, 3);

        _mecanumDrive = new MecanumDrive(this, _logger);
    }

}
