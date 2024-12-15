package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "FrankensteinAuto")
@Disabled
public class FrankensteinAuto extends LinearOpMode {

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
    //    I2C port1: "revColorV3"   (REV color sensor V3)
    //    I2C port3: "navx"         (navX2-Micro: 6-axis LSM6DSM IMU and LIS2MDL Magnetometer)
    // Expansion Hub:
    //    Motor port0: "slideLeft"  (GoBILDA 5202/3/4 series)
    //    Motor port1: "slideRight" (GoBILDA 5202/3/4 series)
    //    I2C port0: "imu2"         (navX2-Micro)
    //    Servo port0: "armServo"
    //    Servo port1: "clawServo"



    ERCGlobalConfig _glbConfig = ERCGlobalConfig.getInstance();

    ERCParameterLogger _logger;
    ERCDrivetrain _drive;
    ERCGobilda4Bar _odometry;
    ERCVision _vision;
//    ERCArm _arm;
//    ERCLed _led;
//    ERCColorSensor _color;
//    ERCTouchSensor _touch;


    private ElapsedTime _debounce = new ElapsedTime();

    // This is where you can configure the specific of your individual robot.
    private void SetConfig() {

        //****************************************************************************
        // Note: this is where you change the default configurations for your robot.
        //****************************************************************************
        _glbConfig.robotType = ERCGlobalConfig.RobotType.Frankenstein;

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
        //_drive = new ERCMecanumDrive(this, _logger, _vision, true);
        _drive = new ERCDrivetrain(this, _logger, _vision, true);

        // For 4-bar, we are considering front center of robot frame to be our arbitrary reference point (in millimeters)
        //double xPodOffsetMM = -50.00625;    // negative is because the pod is to the right of center
        //double yPodOffsetMM = -23.81250;    // negative is because the pod is to the behind the front
        //_odometry = new ERCGobilda4Bar(this, _logger, true, true, true, xPodOffsetMM, yPodOffsetMM);
    }



}
