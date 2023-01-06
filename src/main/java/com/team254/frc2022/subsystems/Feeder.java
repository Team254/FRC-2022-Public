package com.team254.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2022.Constants;
import com.team254.lib.drivers.*;
import com.team254.lib.drivers.PWMColorSensor.ColorSensorState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SynchronousInterrupt;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feeder extends Subsystem {
    public static final double kFullPower = 1.0;
    public static final double kVerySlowPower = 0.05;

    public static final double kFastTicksPer100ms = 21000.0;
    public static final double kNormalTicksPer100ms = 15000.0;
    public static final double kSlowTicksPer100ms = 8000.0;

    public static final double kBallDebounceTime = 0.2;

    private static final int kMinimumColorSensorVotes = 5;
    private static final double kVotingConfidenceThreshold = 0.75;

    private final String mTitle;

    private final TalonFX mMaster;
    private final DigitalInput mBannerSensor;
    private final PWMColorSensor mColorSensor;
    private final SynchronousInterrupt mInterrupt;
    private double mLastFallingEdgeTimestamp = 0.0;

    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private State mState = State.IDLE;

    private int mColorSensorRedVotes = 0;
    private int mColorSensorBlueVotes = 0;

    private static Feeder mLeftInstance;
    private static Feeder mRightInstance;

    public static Feeder getLeftInstance() {
        if (mLeftInstance == null) {
            mLeftInstance = new Feeder(
                    "Left",
                    Constants.kLeftFeederMasterId,
                    Constants.kInvertLeftFeeder,
                    false,
                    Constants.kLeftFeederBannerSensorId,
                    Constants.kLeftFeederColorSensorId
            );
        }
        return mLeftInstance;
    }

    public static Feeder getRightInstance() {
        if (mRightInstance == null) {
            mRightInstance = new Feeder(
                    "Right",
                    Constants.kRightFeederMasterId,
                    Constants.kInvertRightFeeder,
                    true,
                    Constants.kRightFeederBannerSensorId,
                    Constants.kRightFeederColorSensorId
            );
        }
        return mRightInstance;
    }

    public enum State {
        IDLE, FEEDING, FEEDING_VERY_SLOW, SHOOTING, UNJAM, EXHAUSTING_STOP, EXHAUSTING, EXHAUSTING_VERY_SLOW
    }

    public static class PeriodicIO {
        public ColorSensorState color_sensor = ColorSensorState.NONE;
        public boolean banner_sensor;
        public boolean banner_sensor_falling_edge = false;
    }

    private Feeder(String title, CanDeviceId masterId, boolean invert, boolean useLimitSwitch, int bannerSensorId, int colorSensorId) {
        mTitle = title;

        mMaster = TalonFXFactory.createDefaultTalon(masterId);
        mMaster.setInverted(invert);
        mMaster.setNeutralMode(NeutralMode.Brake);
        mMaster.enableVoltageCompensation(true);
        mMaster.configVoltageCompSaturation(12.0);

        mMaster.config_kP(0, Constants.kFeederKp);
        mMaster.config_kI(0, Constants.kFeederKi);
        mMaster.config_kD(0, Constants.kFeederKd);
        mMaster.config_kF(0, Constants.kFeederKf);

        if (useLimitSwitch) {
            TalonUtil.checkError(
                    mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled),
                    "Failed to set forward limit switch");
            TalonUtil.checkError(
                    mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled),
                    "Failed to set reverse limit switch");
            mMaster.overrideLimitSwitchesEnable(false);
        }

        mBannerSensor = new DigitalInput(bannerSensorId);
        mInterrupt = new SynchronousInterrupt(mBannerSensor);
        mInterrupt.setInterruptEdges(false, true);
        mColorSensor = new PWMColorSensor(colorSensorId);
    }

    public synchronized boolean getBannerSensor() {
        return mPeriodicIO.banner_sensor;
    }

    public boolean getForwardLimitSwitch() {
        return mMaster.getSensorCollection().isFwdLimitSwitchClosed() == 1;
    }

    public boolean getReverseLimitSwitch() {
        return mMaster.getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    public void setState(State wantedState) {
        mState = wantedState;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.color_sensor = mColorSensor.getColor();
        mPeriodicIO.banner_sensor = mBannerSensor.get();
        mPeriodicIO.banner_sensor_falling_edge = mInterrupt.getFallingTimestamp() - mLastFallingEdgeTimestamp > kBallDebounceTime;
        mLastFallingEdgeTimestamp = mInterrupt.getFallingTimestamp();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (mState) {
            case IDLE:
                mMaster.set(ControlMode.PercentOutput, 0.0);
                break;
            case FEEDING:
                if (getBannerSensor() || Serializer.getInstance().getBannerSensor()) {
                    mMaster.set(ControlMode.Velocity, kNormalTicksPer100ms);
                } else {
                    mMaster.set(ControlMode.Velocity, kFastTicksPer100ms);
                }
                break;
            case FEEDING_VERY_SLOW:
                mMaster.set(ControlMode.PercentOutput, kVerySlowPower);
                break;
            case SHOOTING:
                mMaster.set(ControlMode.Velocity, kFastTicksPer100ms);
                break;
            case UNJAM:
                if (Serializer.getInstance().getBannerSensor()) {
                    mMaster.set(ControlMode.Velocity, kSlowTicksPer100ms);
                } else {
                    mMaster.set(ControlMode.PercentOutput, 0.0);
                }
                break;
            case EXHAUSTING:
                mMaster.set(ControlMode.PercentOutput, -kFullPower);
                break;
            case EXHAUSTING_VERY_SLOW:
                mMaster.set(ControlMode.PercentOutput, -kVerySlowPower);
                break;
            case EXHAUSTING_STOP:
                if (getBannerSensor()) {
                    mMaster.set(ControlMode.PercentOutput, 0.0);
                } else {
                    mMaster.set(ControlMode.PercentOutput, -kFullPower);
                }
                break;
        }
    }

    public synchronized void updateColorVotes() {
        // Serializer is responsible for calling this at every iteration while this color sensor's
        // readings should be considered 
        if (mPeriodicIO.color_sensor == ColorSensorState.RED) {
            mColorSensorRedVotes++;
        } else if (mPeriodicIO.color_sensor == ColorSensorState.BLUE) {
            mColorSensorBlueVotes++;
        }
    }

    public synchronized void clearColorVotes() {
        // Serializer is responsible for calling this after it has taken the readings it wants from 
        // getCurrentColor()
        mColorSensorRedVotes = 0;
        mColorSensorBlueVotes = 0;
    }

    public synchronized ColorSensorState getCurrentColor() {
        // Only return a color if we are confident in the reading, defined by seeing at least 
        // kMinimumColorSensorVotes measurements that individually are above a color threshold
        // AND seeing that the ratio of red to blue votes (or blue to red votes) exceeds 
        // kVotingConfidenceThreshold. In all other cases, we return ColorSensorState.NONE
        if ((mColorSensorRedVotes + mColorSensorBlueVotes) > kMinimumColorSensorVotes) {
            double voteRatio = mColorSensorRedVotes / (double) (mColorSensorRedVotes + mColorSensorBlueVotes);
            if (voteRatio >= kVotingConfidenceThreshold) {
                return ColorSensorState.RED;
            } else if (voteRatio <= (1.0 - kVotingConfidenceThreshold)) {
                return ColorSensorState.BLUE;
            }
        }
        return ColorSensorState.NONE;
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString(mTitle + " Feeder State", mState.toString());
        SmartDashboard.putBoolean(mTitle + " Banner", mPeriodicIO.banner_sensor);
        SmartDashboard.putNumber(mTitle + " Color Sensor Ratio", mColorSensor.getRatio());
    }
}
