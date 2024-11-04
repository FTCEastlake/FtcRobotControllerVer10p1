package org.firstinspires.ftc.teamcode;

public class ERCAutoInstructions {

    public double forwardPower;
    public double leftPower;
    public double turnCounterClkPower;
    public long sleepMs;

    public boolean alignToAprilTag;
    public double alignDistanceInches;

    public ERCAutoInstructions(
            double forwardPower, double leftPower, double turnCounterClkPower, long sleepMs,
            boolean alignToAprilTag, double alignDistanceInches) {
        this.forwardPower = forwardPower;
        this.leftPower = leftPower;
        this.turnCounterClkPower = turnCounterClkPower;
        this.sleepMs = sleepMs;
        this.alignToAprilTag = alignToAprilTag;
        this.alignDistanceInches = alignDistanceInches;
    }
}
