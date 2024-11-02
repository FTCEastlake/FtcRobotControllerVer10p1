package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="MecanumDriveNavX", group="Linear OpMode")
@Disabled
public class MecanumDriveNavX extends LinearOpMode {
    // Declare motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // NavX sensor
    private NavxMicroNavigationSensor navxMicro;
    private IntegratingGyroscope gyro;

    // Constants for motor power calculations
    private final double POWER_SCALE = 0.8;
    private final double ROTATION_SCALE = 0.6;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // Wait for the game to start
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Get gamepad inputs
            double _y = -gamepad1.left_stick_y;
            double _x = gamepad1.left_stick_x;
            double _rx = gamepad1.right_stick_x;

            // Get current robot heading
            float currentHeading = getCurrentHeading();

            // Calculate motor powers using field-centric drive
            double[] powers = calculateMecanumPowers(_y, _x, _rx, currentHeading);

            // Apply powers to motors
            setMotorPowers(powers);

            // Display telemetry
            displayTelemetry(currentHeading, powers);
        }
    }

    private void initializeHardware() {
        try {
            // Initialize motors
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            // Set motor directions
            boolean isHolyCrab = true;
            if (isHolyCrab)
            {
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            else
            {
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
            }


            // Set zero power behavior
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Initialize NavX
            navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
            gyro = (IntegratingGyroscope)navxMicro;

            // Wait for NavX calibration
            while (navxMicro.isCalibrating()) {
                telemetry.addData("Status", "Calibrating NavX...");
                telemetry.update();
                sleep(50);
            }
            telemetry.addData("Status", "NavX Ready");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("Error", "Hardware initialization failed: " + e.getMessage());
            telemetry.update();
        }
    }

    private float getCurrentHeading() {
        try {
            // Get the robot's heading using the NavX gyro
            return gyro.getAngularOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.ZYX,
                    AngleUnit.DEGREES
            ).firstAngle;
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to get heading: " + e.getMessage());
            telemetry.update();
            return 0.0f;
        }
    }

    private double[] calculateMecanumPowers(double _y, double _x, double _rx, float heading) {
        // Convert heading to radians
        double headingRad = -Math.toRadians(heading);

        // Field-centric calculations
        double rotX = _x * Math.cos(headingRad) - _y * Math.sin(headingRad);
        double rotY = _x * Math.sin(headingRad) + _y * Math.cos(headingRad);

        // Calculate motor powers
        double[] powers = new double[4];
        powers[0] = rotY + rotX + _rx; // Front Left
        powers[1] = rotY - rotX - _rx; // Front Right
        powers[2] = rotY - rotX + _rx; // Back Left
        powers[3] = rotY + rotX - _rx; // Back Right

        // Normalize powers
        double max = Math.max(Math.max(Math.abs(powers[0]), Math.abs(powers[1])),
                Math.max(Math.abs(powers[2]), Math.abs(powers[3])));
        if (max > 1.0) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= max;
            }
        }

        // Scale powers
        for (int i = 0; i < powers.length; i++) {
            powers[i] *= POWER_SCALE;
        }

        return powers;
    }

    private void setMotorPowers(double[] powers) {
        try {
            frontLeft.setPower(powers[0]);
            frontRight.setPower(powers[1]);
            backLeft.setPower(powers[2]);
            backRight.setPower(powers[3]);
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to set motor powers: " + e.getMessage());
            telemetry.update();
        }
    }

    private void displayTelemetry(float heading, double[] powers) {
        telemetry.addData("Heading", heading);
        telemetry.addData("Front Left Power", "%.2f", powers[0]);
        telemetry.addData("Front Right Power", "%.2f", powers[1]);
        telemetry.addData("Back Left Power", "%.2f", powers[2]);
        telemetry.addData("Back Right Power", "%.2f", powers[3]);
        telemetry.addData("NavX Status", navxMicro.isCalibrating() ? "Calibrating" : "Ready");
        telemetry.update();
    }
}