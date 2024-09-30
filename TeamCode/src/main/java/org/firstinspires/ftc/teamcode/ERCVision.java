package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class ERCVision {

    LinearOpMode _opMode;
    HardwareMap _hardwareMap;
    ERCParameterLogger _logger;

    private VisionPortal _visionPortal = null;               // Used to manage the video source.
    private AprilTagProcessor _aprilTag = null;              // Used for managing the AprilTag detection process.

    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.

    String _paramId         = "AprilTag ID";
    String _paramRange      = "AprilTag Range";
    String _paramBearing    = "AprilTag Bearing";
    String _paramYaw        = "AprilTag Yaw";
    String _paramDetections = "AprilTag Detections";

    public ERCVision(LinearOpMode opMode, ERCParameterLogger logger) {

        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _logger = logger;

        // Create the AprilTag processor by using a builder.
        _aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        _aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        _visionPortal = new VisionPortal.Builder()
                .setCamera(_hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(_aprilTag)
                .build();

        _logger.addParameter(_paramId);
        _logger.addParameter(_paramRange);
        _logger.addParameter(_paramBearing);
        _logger.addParameter(_paramYaw);
        _logger.addParameter(_paramDetections);
    }

    public AprilTagDetection detectAprilTag(int tagIndex)
    {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        AprilTagDetection _desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = _aprilTag.getDetections();
        _logger.updateParameter(_paramDetections, currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((tagIndex < 0) || (detection.id == tagIndex)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    _desiredTag = detection;
                    break;  // don't look any further.
                }
            }
        }

        // Range, (Distance), from the Camera lens to the center of the Tag, as measured along the X-Y plane (across the ground).
        // A positive Bearing indicates that the robot must employ a positive Yaw (rotate counter clockwise) in order to point towards the target.
        // A yaw value of zero implies that the camera is directly in front of the Tag, as viewed from above.
        if (targetFound) {
            _logger.updateStatus("Found target id = " + _desiredTag.id + " (" + _desiredTag.metadata.name + ")");
            _logger.updateParameter(_paramId, _desiredTag.id);
            _logger.updateParameter(_paramRange, _desiredTag.ftcPose.range);
            _logger.updateParameter(_paramBearing, _desiredTag.ftcPose.bearing);
            // A negative yaw value means the tag is angled to the right of the camera.
            // A positive yaw value means the tag is angled to the left of the camera.
            _logger.updateParameter(_paramYaw, _desiredTag.ftcPose.yaw);
            _logger.updateAll();
        } else {
            _logger.updateStatus("Target AprilTag id " + tagIndex + " not found");
            _logger.updateParameter(_paramId, -999);
            _logger.updateParameter(_paramRange, -999);
            _logger.updateParameter(_paramBearing, -999);
            _logger.updateParameter(_paramYaw, -999);
            _logger.updateAll();
        }

        return _desiredTag;
    }


    public void setManualExposure(int exposureMS, int gain) {

        // Wait for the camera to be open, then use the controls
        if (_visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (_visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            _logger.updateStatus("Camera settings: waiting...");
            _logger.updateAll();
            while (!_opMode.isStopRequested() && (_visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                _opMode.sleep(20);
            }
            _logger.updateStatus("Camera settings: ready");
            _logger.updateAll();
        }

        // Set camera controls unless we are stopping.
        if (!_opMode.isStopRequested())
        {
            ExposureControl exposureControl = _visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                _opMode.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            _opMode.sleep(20);
            GainControl gainControl = _visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            _opMode.sleep(20);
            _logger.updateStatus("Camera settings: exposure = " + exposureMS + "ms, gain = " + gain);
            _logger.updateAll();
        }
    }

}
