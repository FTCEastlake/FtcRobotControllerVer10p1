package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TutorialOpMode")
//public class Tutorial extends OpMode {
public class TutorialOpMode extends OpMode {

    DcMotor motor;

    int _timeSeconds;


    // Code inside this method will run exactly once after you press the INIT button on the driver station.
    // https://gm0.org/en/latest/docs/software/getting-started/linear-opmode-vs-opmode.html
    @Override
    public void init() {
        _timeSeconds = 0;
        //writeMsgToDriverHub("Initialization: ", "is a success");
        writeMsgToLog("Initialization: ", "is a success");
    }


    // Once the code in start() has been run, code inside this method will run continuously
    // until the STOP button is pressed on the driver station.
    // https://gm0.org/en/latest/docs/software/getting-started/linear-opmode-vs-opmode.html
    @Override
    public void loop() {
        sleep(1000);
        _timeSeconds++;
        //writeMsgToDriverHub("TimerCount: ", _timeSeconds + " seconds");
        writeMsgToLog("TimerCount: ", _timeSeconds + " seconds");
    }

    public void writeMsgToLog(String tag, String msg)
    {
        // Send text to the logcat window
        Log.d(tag, msg);
    }

//    public void writeMsgToDriverHub(String caption, Object value)
//    {
//        // Send text to the driver hub
//        telemetry.addData(caption, value);
//        telemetry.update();
//    }
}
