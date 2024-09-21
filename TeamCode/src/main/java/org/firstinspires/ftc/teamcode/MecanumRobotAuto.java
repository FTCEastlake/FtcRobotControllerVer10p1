package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "MecanumRobotAuto")
public class MecanumRobotAuto extends LinearOpMode {

    ERCParameterLogger _logger = new ERCParameterLogger(this);
    //MecanumDrive _mecanumDrive = new MecanumDrive(this);

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
    double _strafeMagnitude = 0; //find stick distance from 0

    //initialize gyro
    float _gyroYaw = 0;
    double _gyroCos = 0; //<-- cosine of gyro
    double _gyroSin = 0; //<-- sine of gyro

    //boolean for robot control type selection, true fo field centered, false for robot centered
    boolean fieldCenterControlSelect = true;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();
        telemetry.setAutoClear(true);

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
            updateLogAndTelemetry();

            //robot control type selection
            if (gamepad1.dpad_up) {
                while (gamepad1.dpad_up) {
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                    updateLogAndTelemetry();
                }
                if (fieldCenterControlSelect) {
                    robotCenteredControl();
                } else {
                    fieldCenteredControl();
                }
            }


        }
    }

    public void robotCenteredControl(){
//        //change wheel speed proportionate to stick values
//        frontLeft.setPower(lsx + lsy + rsx);
//        frontRight.setPower(lsx + lsy - rsx);
//        backLeft.setPower(lsx - lsy + rsx);
//        backRight.setPower(lsx - lsy - rsx);
    }

    public void fieldCenteredControl() {
//        //change wheel speed proportionate to stick values
//
//
//        // roll vs pitch vs yaw
//        // ---> https://upload.wikimedia.org/wikipedia/commons/thumb/c/c1/Yaw_Axis_Corrected.svg/250px-Yaw_Axis_Corrected.svg.png
//
//
//        //mecanum wheel directions mapping
//        // ---> https://gm0.org/en/latest/_images/mecanum-drive-directions.png
//
//
//        //update stick magnitude (for gyroCos and gyroSin)
//        strafeMagnitude = Math.sqrt((lsx * lsx) + (lsy * lsy));
//
//
//        //update gyro values
//        gyroYaw = 0;
//        gyroCos = Math.cos(gyroYaw)/(2 * Math.PI)*strafeMagnitude;
//        gyroSin = Math.sin(gyroYaw)/(2 * Math.PI)*strafeMagnitude;
//
//
//        //math to determine power for each motor
//        // ---> subtract gyroX and gyroY from lsx and lsy respectively to get distance between the two
//        //      ---> leave spin value (rsx) alone, is not affected by gyro
//        FLPower = (lsx - gyroCos) + (lsy - gyroSin) + rsx;
//        FRPower = (lsx - gyroCos) + (lsy - gyroSin) - rsx;
//        BLPower = (lsx - gyroCos) - (lsy - gyroSin) + rsx;
//        BRPower = (lsx - gyroCos) - (lsy - gyroSin) - rsx;
//
//
//        frontLeft.setPower( FLPower );
//        frontRight.setPower( FRPower );
//        backLeft.setPower( BLPower );
//        backRight.setPower( BRPower );
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
        //_mecanumDrive = new MecanumDrive(this);

        // Add all of the parameters you want displayed on the driver hub
        //_logger.addStatus();
        _logger.init();

        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramLsx, 1);
        _logger.addParameter(_paramLsr, 2);
        _logger.addParameter(_paramRsx, 3);
    }

}
