package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "TutorialTelemetryParameters")
//public class Tutorial extends OpMode {
public class TutorialTelemetryParameters extends LinearOpMode {

    //region VARIABLES
    ERCParameterLogger _logger = new ERCParameterLogger(this);
    DcMotor _motor;
    int _timeSeconds = 0;

    // I'm just using "param#" as generic parameters here.
    // You should name the parameters according to the value that it's tracking.
    // ex: String _paramController = "Controller";
    String _param1 = "param1";
    String _param2 = "param2";
    String _param3 = "param3";
    String _param4 = "param4";
    String _param5 = "param5";
    String _param6 = "param6";
    String _param7 = "param7";
    String _param8 = "param8";
    String _param9 = "param9";
    String _param10 = "param10";
    String _param11 = "param11";
    String _param12 = "param12";
    String _param13 = "param13";
    String _param14 = "param14";
    String _param15 = "param15";
    String _param16 = "param16";
    String _param17 = "param17";
    String _param18 = "param18";
    String _param19 = "param19";
    String _param20 = "param20";
    //endregion

    private void initParams()
    {
        // Add all of the parameters you want displayed on the driver hub
        //_logger.addStatus();
        _logger.init();

        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_param1, 1);
        _logger.addParameter(_param2, 2);
        _logger.addParameter(_param3, 3);
        _logger.addParameter(_param4, 4);
        _logger.addParameter(_param5, 5);
        _logger.addParameter(_param6, 6);
        _logger.addParameter(_param7, 7);
        _logger.addParameter(_param8, 8);
        _logger.addParameter(_param9, 9);
        _logger.addParameter(_param10, 10);
        _logger.addParameter(_param11, 11);
        _logger.addParameter(_param12, 12);
        _logger.addParameter(_param13, 13);
        _logger.addParameter(_param14, 14);
        _logger.addParameter(_param15, 15);
        _logger.addParameter(_param16, 16);
        _logger.addParameter(_param17, 17);
        _logger.addParameter(_param18, 17);
        _logger.addParameter(_param19, 19);
        _logger.addParameter(_param20, 20);
    }

    // Code inside this method will run exactly once after you press the INIT button.
    // This is where you should put all code for the OpMode.
    // https://gm0.org/en/latest/docs/software/getting-started/linear-opmode-vs-opmode.html
    @Override
    public void runOpMode() throws InterruptedException {

        initParams();
        String msg = "Init button was pressed. Waiting for start button to be pressed";
        _logger.writeMsg(msg);
        waitForStart();
        msg = "start button was pressed";
        _logger.writeMsg(msg);

        while (!isStopRequested())
        {
            sleep(1000);
            _timeSeconds++;
            msg = "timer: " + _timeSeconds + " seconds";
            _logger.writeMsg(msg);

            // This is just an example of showing how each parameter can be updated in real time.
            if ((_timeSeconds % 2) == 0)
            {
                // Seconds are even values
                _logger.updateParameter(_param1, _timeSeconds);
                _logger.updateParameter(_param2, _timeSeconds * 2);
                _logger.updateParameter(_param3, _timeSeconds * 3);
            }
            else
            {
                // Seconds are odd values
                _logger.updateParameter(_param4, _timeSeconds * 4);
                _logger.updateParameter(_param5, _timeSeconds * 5);
                _logger.updateParameter(_param6, _timeSeconds * 6);
            }

            // Update all parameters at once if they weren't updated immediately.
            _logger.updateAll();
        }

        msg = "stop request was detected";
        _logger.writeMsg(msg);
    }
}
