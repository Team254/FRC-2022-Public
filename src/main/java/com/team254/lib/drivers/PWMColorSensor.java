package com.team254.lib.drivers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SynchronousInterrupt;

public class PWMColorSensor {
    public static final double kPulsePeriod = 1.0 / 4482.0;
    public static final double kPWMBound = 300.0 / 4095.0;
    public static final double kBlueThreshold = 0.41;
    public static final double kRedThreshold = 0.60;
    
    private final SynchronousInterrupt mInterrupt;
    private double mPrevRatio = 0.5;

    public enum ColorSensorState {
        RED,
        BLUE,
        NONE,
    }

    public PWMColorSensor(int channel) {
        mInterrupt = new SynchronousInterrupt(new DigitalInput(channel));
        mInterrupt.setInterruptEdges(true, true);
    }

    private double adjustRatio(double value) {
        double ret = (value - kPWMBound) / (1.0 - 2 * kPWMBound);
        if (ret < 0.0 || ret > 1.0) {
            return /* mPrevRatio */ 0.5;
        }
        return ret;
    }

    public double getRatio() {
        double rising = mInterrupt.getRisingTimestamp();
        double falling = mInterrupt.getFallingTimestamp();
        if (rising < falling) {
            mPrevRatio = adjustRatio((falling - rising) / kPulsePeriod);
        } else {
            mPrevRatio = adjustRatio(1.0 + (falling - rising) / kPulsePeriod);
        }
        return mPrevRatio;
    }

    public ColorSensorState getColor() {
        double ratio = getRatio();

        if (ratio > kRedThreshold) {
            return ColorSensorState.RED;
        } else if (ratio < kBlueThreshold) {
            return ColorSensorState.BLUE;
        }
        return ColorSensorState.NONE;
    }
}
