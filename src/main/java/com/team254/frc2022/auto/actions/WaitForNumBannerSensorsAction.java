package com.team254.frc2022.auto.actions;

import com.team254.frc2022.subsystems.Serializer;
import edu.wpi.first.wpilibj.Timer;

public class WaitForNumBannerSensorsAction implements Action {

    private double mStartTime = 0;
    private double mTimeout;
    private int mNumSensors;

    public WaitForNumBannerSensorsAction(int numSensors, double timeout) {
        mNumSensors = numSensors;
        mTimeout = timeout;
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();

    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return (Serializer.getInstance().getNumBannersTriggered() >= mNumSensors) || ((Timer.getFPGATimestamp() - mStartTime) >= mTimeout);
    }

    @Override
    public void done() {

    }
}
