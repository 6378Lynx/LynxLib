package lynxlib.util;

import edu.wpi.first.wpilibj.Timer;

public class SetpointRamp {

    /**
     * Max change over interval of 1s
     * Default to infinity, acts as if there is no ramp
     */
    private double maxChangePerSecond = Double.POSITIVE_INFINITY;

    /**
     * The last value returned
     */
    private double lastValue;

    /**
     * The last time
     */
    private double lastTime;

    public SetpointRamp() {
    }

    /**
     * Ramps a given value
     *
     * @param value the value being ramped
     * @return the ramped value
     */
    public double rampValue(double value) {
        //If your current setpoint is higher than your last, get the minimum between your current setpoint and the last setpoint + maximum changed allowed per interval
        //otherwise get the opposite
        if (value > lastValue) {
            //If your current value is higher than the last value, next value will be the minimum between current value and last value + maxChange allowed in an interval
            lastValue = Math.min(value, lastValue + (Timer.getFPGATimestamp() - lastTime) * maxChangePerSecond);
        } else {
            //If your current value is less than your last value, next value will be the maximum between the current value and last value - maxChange allowed in an interval
            lastValue = Math.max(value, lastValue - (Timer.getFPGATimestamp() - lastTime) * maxChangePerSecond);
        }
        lastTime = Timer.getFPGATimestamp();
        return lastValue;
    }

    /**
     * Sets the max change allowed per second
     *
     * @param maxChangePerSecond the max change allowed per second
     */
    public void setMaxChangePerSecond(double maxChangePerSecond){
        this.maxChangePerSecond = maxChangePerSecond;
    }

}
