package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import android.util.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.List;
import java.util.Objects;
import java.util.Optional;


// This class will handle all of the logging
// https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html?org/firstinspires/ftc/robotcore/external/Telemetry.html
public class ERCParameterLogger {

    private LinearOpMode _opMode;   // handle to LinearOpMode from the main function
    Telemetry _telemetry;

    Map<String, Telemetry.Item> _paramMap;
    Telemetry.Item _statusUpdate;
    String _statusCaption= "Status";

    // We need to use the telemetry belonging to LinearOpMode, otherwise it will throw an exception.
    public ERCParameterLogger(LinearOpMode opMode){
        _opMode = opMode;
        _telemetry = _opMode.telemetry;

        init();
    }

    private void init()
    {
        _paramMap = new HashMap<>();
        _telemetry.clear();             // Removes all items from the receiver whose value is not to be retained.
        _telemetry.setAutoClear(false); // Sets whether clear() is automatically called after each call to update().

        // Status will always be the first (topmost) message to be displayed.
        _statusUpdate = _telemetry.addData(_statusCaption, "Parameters cleared");
        _paramMap.put(_statusCaption, _statusUpdate);
        _telemetry.update();
    }

    //*********************************************************************
    // Adding parameters to telemetry
    //*********************************************************************
    public void addParameter(String paramString) {
        _paramMap.put(paramString, _telemetry.addData(paramString, 0));
    }


    //*********************************************************************
    // Updating status and parameters
    //*********************************************************************
    public void updateStatus(String val) {
        _statusUpdate.setValue(val);
    }
    public void updateParameter(String paramString, int val) {
        Objects.requireNonNull(_paramMap.get(paramString)).setValue(val);
    }
    public void updateParameter(String paramString, double val) {
        Objects.requireNonNull(_paramMap.get(paramString)).setValue(val);
    }
    public void updateParameter(String paramString, String val, Boolean updateNow) {
        Objects.requireNonNull(_paramMap.get(paramString)).setValue(val);
        if (updateNow) _telemetry.update();
    }
    public void updateAll() {
        _telemetry.update();
    }


    // Note: use [tag: "ERCStatus"] to filter on only logs from our code.
    public void writeMsgToLogcat(String msg)
    {
        // Send message to the logcat window
        Log.d(_statusCaption, msg);
    }

    public void writeMsgToDriverHub(Object msg)
    {
        // Send message to the driver hub.
        updateStatus(msg.toString());
    }

    public void writeMsg(Object msg)
    {
        writeMsgToLogcat(msg.toString());
        writeMsgToDriverHub(msg);
    }
}
