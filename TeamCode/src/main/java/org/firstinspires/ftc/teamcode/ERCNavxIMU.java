package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ERCNavxIMU implements ERCImuInterface{

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private ERCParameterLogger _logger;

    private AHRS _navxProc;
    private NavxMicroNavigationSensor _navx;
    private IntegratingGyroscope _gyro;
    private ElapsedTime _timer = new ElapsedTime();


//    private String _paramNavxDx= "NavX dx";
//    private String _paramNavxDy= "NavX dy";
//    private String _paramNavxDz= "NavX dz";
//    private String _paramNavxHeading= "NavX Heading";
//    private String _paramNavxRoll= "NavX Roll";
//    private String _paramNavxPitch= "NavX Pitch";

    public ERCNavxIMU(LinearOpMode opMode, ERCParameterLogger logger) throws InterruptedException {

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _logger = logger;

        init();
    }

    private void init() throws InterruptedException {

        _navx = _hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        _gyro = (IntegratingGyroscope)_navx;
        _navxProc = AHRS.getInstance(_hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData);

        // The gyro automatically starts calibrating. This takes a few seconds.
        _logger.updateStatus("Gyro Calibrating. Do Not Move!");
        _logger.updateAll();

        // Wait until the gyro calibration is complete
        _timer.reset();
        while (_navx.isCalibrating())  {
            _logger.updateStatus("calibrating: seconds = " + _timer.seconds());
            _logger.updateAll();
            Thread.sleep(50);
        }
        _logger.updateStatus("Gyro Calibrated. Press Start.");
        _logger.updateAll();

//        // Add all of the parameters you want to see on the driver hub display.
//        _logger.addParameter(_paramNavxDx);
//        _logger.addParameter(_paramNavxDy);
//        _logger.addParameter(_paramNavxDz);
//        _logger.addParameter(_paramNavxHeading);
//        _logger.addParameter(_paramNavxRoll);
//        _logger.addParameter(_paramNavxPitch);
    }

    public void resetYaw()
    {
        _navxProc.zeroYaw();
    }

    public double getBotHeading() {
        return Math.toRadians(getCurrentHeading());
    }

    private float getCurrentHeading() {
        try {
            // Get the robot's heading using the NavX gyro
            return _gyro.getAngularOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.ZYX,
                    AngleUnit.DEGREES
            ).firstAngle;
        } catch (Exception e) {
            _logger.updateStatus("Failed to get heading: " + e.getMessage());
            _logger.updateAll();
            return 0.0f;
        }
    }

//    public void getStatus () {
//
//        // Read dimensionalized data from the gyro. This gyro can report angular velocities
//        // about all three axes. Additionally, it internally integrates the Z axis to
//        // be able to report an absolute angular Z orientation.
//        AngularVelocity rates = _navx.getAngularVelocity(AngleUnit.DEGREES);
//        Orientation angles = _navx.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        _logger.updateParameter(_paramNavxDx, formatRate(rates.xRotationRate));
//        _logger.updateParameter(_paramNavxDy, formatRate(rates.yRotationRate));
//        _logger.updateParameter(_paramNavxDz, formatRate(rates.zRotationRate));
//
//        _logger.updateParameter(_paramNavxHeading, formatAngle(angles.angleUnit, angles.firstAngle));
//        _logger.updateParameter(_paramNavxRoll, formatAngle(angles.angleUnit, angles.secondAngle));
//        _logger.updateParameter(_paramNavxPitch, formatAngle(angles.angleUnit, angles.thirdAngle));
//        _logger.updateAll();
//    }

    private String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
