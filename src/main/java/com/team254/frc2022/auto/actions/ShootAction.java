package com.team254.frc2022.auto.actions;


import com.team254.frc2022.subsystems.Serializer;
import com.team254.frc2022.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.ShootingParameters;
import com.team254.lib.util.TimeDelayedBoolean;
import edu.wpi.first.wpilibj.Timer;

public class ShootAction implements Action {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private double mTimeout;
    private boolean mResetToIdle;
    private double mStartTime = 0.0;
    private boolean mHasStarted = false;
        private static final double kPostBannerShotClearTime = 0.7;

    private TimeDelayedBoolean mShotsGone = new TimeDelayedBoolean();

    public ShootAction(double timeout, boolean resetToIdle) {
        mTimeout = timeout;
        mResetToIdle = resetToIdle;
    }

    public ShootAction(double timeout) {
        this(timeout, true);
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mSuperstructure.setWantedState(Superstructure.WantedState.SHOOTING);
    }

    @Override
    public void update() {

    }


    @Override
    public boolean isFinished() {
        double elapsed = Timer.getFPGATimestamp() - mStartTime;
        boolean timedOut = elapsed >= mTimeout;
        boolean shotsGone = mShotsGone.update(Serializer.getInstance().getNumBannersTriggered() == 0, kPostBannerShotClearTime);
        return shotsGone || timedOut;
    }

    @Override
    public void done() {
        if (mResetToIdle) {
            mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
        }
    }
}
