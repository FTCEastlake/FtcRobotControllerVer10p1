package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class ERCGobilda4Bar {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private ERCParameterLogger _logger;
    private GoBildaPinpointDriver _odo; // Declare OpMode member for the Odometry Computer

    private double _xOffset;
    private double _yOffset;


    private String _param4BarXPos = "4Bar X (inches)";
    private String _param4BarYPos = "4Bar Y (inches)";
    private String _param4BarRotation = "4Bar Rotation (degrees)";


    public ERCGobilda4Bar(LinearOpMode opMode, ERCParameterLogger logger, boolean resetIMU,
                          boolean xPositiveForward, boolean yPositiveLeft, double xOffset, double yOffset) {

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _logger = logger;
        _xOffset = xOffset;
        _yOffset = yOffset;

        init(resetIMU, xPositiveForward, yPositiveLeft);
    }

    private void init(boolean resetIMU, boolean xPositiveForward, boolean yPositiveLeft) {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        _odo = _hardwareMap.get(GoBildaPinpointDriver.class,"odo4bar");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        _odo.setOffsets(_xOffset, _yOffset); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        _odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        GoBildaPinpointDriver.EncoderDirection xDirection = xPositiveForward ?
                GoBildaPinpointDriver.EncoderDirection.FORWARD : GoBildaPinpointDriver.EncoderDirection.REVERSED;
        GoBildaPinpointDriver.EncoderDirection yDirection = yPositiveLeft ?
                GoBildaPinpointDriver.EncoderDirection.FORWARD : GoBildaPinpointDriver.EncoderDirection.REVERSED;
        _odo.setEncoderDirections(xDirection, yDirection);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        if (resetIMU)
            _odo.resetPosAndIMU();

        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_param4BarXPos);
        _logger.addParameter(_param4BarYPos);
        _logger.addParameter(_param4BarRotation);
        _logger.updateAll();
    }

    double [] getPosition() {
        _odo.update();
        Pose2D pos = _odo.getPosition();
        double[] coordinates = new double[3];
        coordinates[0] = pos.getX(DistanceUnit.INCH);
        coordinates[1] = pos.getY(DistanceUnit.INCH);
        coordinates[2] = pos.getHeading(AngleUnit.DEGREES);

        _logger.updateParameter(_param4BarXPos, coordinates[0]);
        _logger.updateParameter(_param4BarYPos, coordinates[1]);
        _logger.updateParameter(_param4BarRotation, coordinates[2]);
        return coordinates;
    }

    Pose2D getVelocity() {
        _odo.update();
        Pose2D vel  = _odo.getVelocity();

        return vel;
    }


}
