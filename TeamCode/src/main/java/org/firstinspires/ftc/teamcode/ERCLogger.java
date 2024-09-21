package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import android.util.Log;


// This class will handle all of the logging
public class ERCLogger {

    Telemetry _telemetry;
    String _debugTag = "Eastlake";

    // We need to use the telemetry belonging to LinearOpMode, otherwise it will throw an exception.
    public ERCLogger(LinearOpMode opMode){
        _telemetry = opMode.telemetry;
    }

    // Note: use [tag: "Eastlake"] to filter on only logs from our code.
    public void writeMsgToLogcat(String msg)
    {
        // Send text to the logcat window
        Log.d(_debugTag, msg);
    }

    public void writeMsgToDriverHub(Object msg)
    {
        // Send text to the driver hub.
        // Caption will default to empty string.
        // msg should contain all of the info.
        _telemetry.addData("", msg);
        _telemetry.update();
    }

    public void writeMsgToAllLogOutputs(Object msg)
    {
        writeMsgToLogcat(msg.toString());
        writeMsgToDriverHub(msg);
    }
}
