package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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


    private ElapsedTime _cycleTime = new ElapsedTime();
    private boolean _enableCycleTime = false;
    private double _cycleTimeMin = 100.0, _cycleTimeMax = 0.0, _cycleTimeAvg = 0.0;
    private double _cycleTimePrevious = 0.0, _cycleTimeCurrent = 0.0, _cycleTimeDelta = 0.0;

    private String _paramCycleTimeMin = "Cycle Time Min (ms)";
    private String _paramCycleTimeMax = "Cycle Time Max (ms";
    private String _paramCycleTimeAvg = "Cycle Time Avg (ms)";

    // We need to use the telemetry belonging to LinearOpMode, otherwise it will throw an exception.
    public ERCParameterLogger(LinearOpMode opMode, boolean enableCycleTimeLogging){
        _opMode = opMode;
        _telemetry = _opMode.telemetry;
        _enableCycleTime = enableCycleTimeLogging;
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

        // Add all of the parameters you want to see on the driver hub display.
        if (_enableCycleTime) {
            addParameter(_paramCycleTimeMin);
            addParameter(_paramCycleTimeMax);
            addParameter(_paramCycleTimeAvg);
        }
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
    public void updateParameter(String paramString, boolean val) {
        Objects.requireNonNull(_paramMap.get(paramString)).setValue(val ? "True" : "False");
    }
    public void updateParameter(String paramString, int val) {
        Objects.requireNonNull(_paramMap.get(paramString)).setValue(val);
    }
    public void updateParameter(String paramString, double val) {
        Objects.requireNonNull(_paramMap.get(paramString)).setValue(val);
    }
    public void updateParameter(String paramString, String val) {
        Objects.requireNonNull(_paramMap.get(paramString)).setValue(val);
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

    public void resetCycleTimer(){
        if (_enableCycleTime)
            _cycleTime.reset();
    }

    public void updateCycleTimer(double loopCount) {
        if (_enableCycleTime) {
            _cycleTimePrevious = _cycleTimeCurrent;
            _cycleTimeCurrent = _cycleTime.milliseconds();
            _cycleTimeDelta = _cycleTimeCurrent - _cycleTimePrevious;

            if (_cycleTimeDelta < _cycleTimeMin) _cycleTimeMin = _cycleTimeDelta;
            if (_cycleTimeDelta > _cycleTimeMax) _cycleTimeMax = _cycleTimeDelta;
            _cycleTimeAvg = _cycleTimeCurrent / loopCount;

            updateParameter(_paramCycleTimeMin, _cycleTimeMin);
            updateParameter(_paramCycleTimeMax, _cycleTimeMax);
            updateParameter(_paramCycleTimeAvg, _cycleTimeAvg);
            updateAll();
        }
    }
}
