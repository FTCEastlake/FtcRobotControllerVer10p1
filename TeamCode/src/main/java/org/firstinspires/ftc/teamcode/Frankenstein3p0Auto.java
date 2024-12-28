package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Frankenstein3p0Auto")
@Disabled
public class Frankenstein3p0Auto extends LinearOpMode {

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


    ERCGlobalConfig _glbConfig = ERCGlobalConfig.getInstance();

    ERCParameterLogger _logger;
    ERCDrivetrain3p0 _drive;
    ERCGobilda4Bar _odometry;
    ERCVision _vision;



    private ElapsedTime _debounce = new ElapsedTime();

    // This is where you can configure the specific of your individual robot.
    private void SetConfig() {

        //****************************************************************************
        // Note: this is where you change the default configurations for your robot.
        //****************************************************************************
        _glbConfig.robotType = ERCGlobalConfig.RobotType.Standard;

        // Drivetrain
        _glbConfig.drivePowerFast = 0.75;
        _glbConfig.drivePowerNormal = 0.50;
        _glbConfig.drivePowerSlow = 0.25;

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        boolean isDone = false;
        double autoDrivePower = 0.5;

        //******************************
        // Main loop
        //******************************
        waitForStart();
        while (!isStopRequested())
        {
            if (isDone)
                continue;

            // Move forward 10 inches
            CalibratedMove(10.0, 0, 0, autoDrivePower);

            // Move back 5 inches
            CalibratedMove(-5.0, 0, 0, autoDrivePower);

            // Move left 10 inches
            CalibratedMove(0, -10, 0, autoDrivePower);

            // Move right 10 inches
            CalibratedMove(0, 10, 0, autoDrivePower);

            // Turn 90 degrees clockwise
            CalibratedMove(0, 0, 90, autoDrivePower);

            // Turn 90 degrees counter-clockwise
            CalibratedMove(0, 0, -90, autoDrivePower);

            isDone = true;
        }
    }


    // Update these calibration ratios for your specific robots.
    // Example: if you set forwardInches = 10 and it actually moves 11 inches, then set calibratedForwardRatio = 10/11 = 0.9091
    double calibratedForwardRatio = 1.0;
    double calibratedRightRatio = 1.0;
    double calibratedDegreesRatio = 1.0;
    private void CalibratedMove(double forwardInches, double rightInches, double degrees, double driveSpeed)
    {
        _drive.autoMove(
                forwardInches * calibratedForwardRatio,
                rightInches * calibratedRightRatio,
                degrees * calibratedDegreesRatio,
                driveSpeed);
    }

    private void initRobot() throws InterruptedException {

        SetConfig();
        _logger = new ERCParameterLogger(this, true);
        _vision = null;


        //****************************************************************************************
        // For 4-bar, we are considering front center of robot frame to be our arbitrary reference point (in millimeters)
        //****************************************************************************************
        double xPodOffsetMM = -166.6875;    // X-pod offset (left/right) of arbitrary point, negative value for right of arbitrary point
        double yPodOffsetMM = -169.8625;    // Y-pod offset (front/behind) of arbitrary point, negative value for behind arbitrary point
        _odometry = new ERCGobilda4Bar(this, _logger, true, xPodOffsetMM, yPodOffsetMM);


        _drive = new ERCDrivetrain3p0(this, _logger, _odometry, _vision, true);
    }



}
