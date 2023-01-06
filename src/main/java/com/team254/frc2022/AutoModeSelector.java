package com.team254.frc2022;

import com.team254.frc2022.auto.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING,
        TEST_TRAJECTORY,
        DEFAULT_MODE,
        FAR_RIGHT_5_BALL,
        FAR_RIGHT_5_BALL_PLUS_1,
        FAR_LEFT_2_BALL_PLUS_0,
        FAR_LEFT_2_BALL_PLUS_1,
        FAR_LEFT_2_BALL_PLUS_2
    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private final SendableChooser<DesiredMode> mModeChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test Trajectory", DesiredMode.TEST_TRAJECTORY);
        mModeChooser.addOption("Default Mode", DesiredMode.DEFAULT_MODE);
        mModeChooser.addOption("Far Right 5 Ball", DesiredMode.FAR_RIGHT_5_BALL);
        mModeChooser.addOption("Far Right 5 Ball Plus 1", DesiredMode.FAR_RIGHT_5_BALL_PLUS_1);
        mModeChooser.addOption("Far Left 2 Ball Plus 0", DesiredMode.FAR_LEFT_2_BALL_PLUS_0);
        mModeChooser.addOption("Far Left 2 Ball Plus 1", DesiredMode.FAR_LEFT_2_BALL_PLUS_1);
        mModeChooser.addOption("Far Left 2 Ball Plus 2", DesiredMode.FAR_LEFT_2_BALL_PLUS_2);
        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();

        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }

        if (mCachedDesiredMode != desiredMode) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingAutoMode());
            case TEST_TRAJECTORY:
                return Optional.of(new TestTrajectoryFollowingMode());
            case DEFAULT_MODE:
                return Optional.of(new DefaultMode());
            case FAR_RIGHT_5_BALL:
                return Optional.of(new FarRight5Ball());
            case FAR_RIGHT_5_BALL_PLUS_1:
                return Optional.of(new FarRight5BallPlus1());
            case FAR_LEFT_2_BALL_PLUS_0:
                return Optional.of(new FarLeft2BallPlus0());
            case FAR_LEFT_2_BALL_PLUS_1:
                return Optional.of(new FarLeft2BallPlus1());
            case FAR_LEFT_2_BALL_PLUS_2:
                return Optional.of(new FarLeft2BallPlus2());
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = DesiredMode.DO_NOTHING;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }
}