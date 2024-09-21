package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TutorialLinearOpMode")
//public class Tutorial extends OpMode {
public class TutorialLinearOpMode extends LinearOpMode {

    ERCLogger _logger = new ERCLogger(this);
    DcMotor _motor;
    int _timeSeconds;

    // Code inside this method will run exactly once after you press the INIT button.
    // This is where you should put all code for the OpMode.
    // https://gm0.org/en/latest/docs/software/getting-started/linear-opmode-vs-opmode.html
    @Override
    public void runOpMode() throws InterruptedException {

        String msg = "Init button was pressed. Waiting for start button to be pressed";
        _logger.writeMsgToAllLogOutputs(msg);
        waitForStart();
        msg = "start button was pressed";
        _logger.writeMsgToAllLogOutputs(msg);

        while (!isStopRequested())
        {
            sleep(1000);
            _timeSeconds++;
            msg = "timer: " + _timeSeconds + " seconds";
            _logger.writeMsgToAllLogOutputs(msg);
        }

        msg = "stop request was detected";
        _logger.writeMsgToAllLogOutputs(msg);
    }
}
