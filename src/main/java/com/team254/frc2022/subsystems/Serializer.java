package com.team254.frc2022.subsystems;

import com.team254.frc2022.Constants;
import com.team254.lib.drivers.PWMColorSensor.ColorSensorState;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.util.TimeDelayedBoolean;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SynchronousInterrupt;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Serializer extends Subsystem {
    private static final double kFinishSerializingTime = 0.4;
    private static final double kEjectBannerLetoff = 0.4;
    private static final double kShootingOffsetTime = 0.05;
    private static final double kShootingCooldown = 0.1; //s

    private final Intake mLeftIntake = Intake.getLeftInstance();
    private final Intake mRightIntake = Intake.getRightInstance();
    private final Feeder mLeftFeeder = Feeder.getLeftInstance();
    private final Feeder mRightFeeder = Feeder.getRightInstance();

    private Feeder mFrontFeeder = mRightFeeder; // side closest to intaking
    private Feeder mBackFeeder = mLeftFeeder; // side furthest from intaking, ball destination
    private Intake mFrontIntake = mRightIntake;
    private Intake mBackIntake = mLeftIntake;

    private final DigitalInput mBannerSensor;
    private final SynchronousInterrupt mInterrupt;
    private double mLastFallingEdgeTimestamp = 0.0;

    private final PeriodicIO mPeriodicIO = new PeriodicIO();

    private final TimeDelayedBoolean mShootingOffset = new TimeDelayedBoolean();

    private ColorSensorState mIncorrectColor = ColorSensorState.BLUE;

    private double mLastEjectTimestamp = 0.0;
    private double mWantIdleTimestamp = 0.0;

    private WantedState mWantedState = WantedState.IDLE;
    private SystemState mSystemState = SystemState.IDLE;
    private ColorRejectionMode mColorRejectionMode = ColorRejectionMode.AUTOMATIC_REJECTION;
    private final SendableChooser<ColorRejectionMode> mModeChooser;

    private boolean mForceNoEject = false;

    private static Serializer mInstance = null;

    public static Serializer getInstance() {
        if (mInstance == null) {
            mInstance = new Serializer();
        }
        return mInstance;
    }

    public enum WantedState {
        IDLE, INTAKING_LEFT, INTAKING_RIGHT, EJECTING_LEFT, EJECTING_RIGHT, EXHAUSTING
    }

    public enum SystemState {
        IDLE, INTAKING, INTAKING_FULL, INTAKING_EJECTING, EJECTING, EJECTING_MANUAL, SERIALIZING, EXHAUSTING
    }

    public enum Direction {
        LEFT, RIGHT
    }

    public enum ColorRejectionMode {
        MANUAL_REJECTION, AUTOMATIC_REJECTION
    }

    public static class PeriodicIO {
        public boolean banner_sensor = false;
        public boolean banner_sensor_falling_edge = false;
    }

    private Serializer() {
        mBannerSensor = new DigitalInput(Constants.kUpperBannerSensorId);

        mInterrupt = new SynchronousInterrupt(mBannerSensor);
        mInterrupt.setInterruptEdges(true, true);
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("ENABLE automatic ball rejection", ColorRejectionMode.AUTOMATIC_REJECTION);
        mModeChooser.addOption("DISABLE automatic ball rejection", ColorRejectionMode.MANUAL_REJECTION);
        SmartDashboard.putData("Serializer Ball Rejection", mModeChooser);
    }

    public boolean getLimitSwitchTripped() {
        if (Constants.kTurretHomingUseReverseLimit) {
            return mRightFeeder.getReverseLimitSwitch();
        } else {
            return mRightFeeder.getForwardLimitSwitch();
        }
    }

    public synchronized boolean getBannerSensor() {
        return mPeriodicIO.banner_sensor;
    }

    public synchronized boolean getBannerSensorFallingEdge() {
        return mPeriodicIO.banner_sensor_falling_edge;
    }

    public int getNumBannersTriggered() {
        return (getBannerSensor() ? 1 : 0) + (mLeftFeeder.getBannerSensor() ? 1 : 0) + (mRightFeeder.getBannerSensor() ? 1 : 0);
    }

    public boolean readyToShoot() {
        return mSystemState == SystemState.INTAKING || getNumBannersTriggered() > 0;
    }

    public boolean isEjecting() {
        return mSystemState == SystemState.EJECTING || mSystemState == SystemState.INTAKING_EJECTING || mSystemState == SystemState.EJECTING_MANUAL;
    }

    /**
     * Check if the next ball to be shot should be rejected. 
     * Incorrect ball colors are only checked when a ball is breaking the middle beam break sensor
     */
    private boolean hasIncorrectBall() {
        return mFrontFeeder.getCurrentColor() == mIncorrectColor ||
                (mBackFeeder.getCurrentColor() == mIncorrectColor && mFrontFeeder.getCurrentColor() == ColorSensorState.NONE);
    }

    public synchronized void setWantedState(WantedState wantedState) {
        if (wantedState == WantedState.IDLE && (mSystemState == SystemState.INTAKING || mSystemState == SystemState.INTAKING_EJECTING)) {
            mWantIdleTimestamp = Timer.getFPGATimestamp();
        }
        mWantedState = wantedState;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.banner_sensor = mBannerSensor.get();
        mPeriodicIO.banner_sensor_falling_edge = mInterrupt.getFallingTimestamp() - mLastFallingEdgeTimestamp > 0.1;
        mLastFallingEdgeTimestamp = mInterrupt.getFallingTimestamp();
    }

    public enum ExhuastMode{
        BOTH,
        LEFT,
        RIGHT
    }
    private ExhuastMode mExhaustMode = ExhuastMode.BOTH;
    public synchronized void setExhaustMode(ExhuastMode mode) {
        mExhaustMode = mode;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Serializer.this) {
                    setWantedState(WantedState.IDLE);
                    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
                        mIncorrectColor = ColorSensorState.RED;
                    } else {
                        mIncorrectColor = ColorSensorState.BLUE;
                    }
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Serializer.this) {

                    // when we have a rising edge or are actively seeing a ball with middle banner,
                    // begin the color voting process.
                    if (getBannerSensor()) {
                        mFrontFeeder.updateColorVotes();
                        mBackFeeder.updateColorVotes();
                    }

                    // when a ball leaves the middle banner area, sample the color voting and then reset the votes
                    if (getBannerSensorFallingEdge()) {
                        if (hasIncorrectBall()) {
                            // by setting this timestamp, we will enter the ejecting system state
                            // within the handleX() methods for wanted state
                            mLastEjectTimestamp = timestamp;
                        }
                        mFrontFeeder.clearColorVotes();
                        mBackFeeder.clearColorVotes();
                    }

                    // Handle logic for changing system state
                    SystemState newState = handleAll(timestamp);
                    if (newState != mSystemState) {
                        System.out.println("Serializer " + mSystemState + " -> " + newState + " " + timestamp);
                        mSystemState = newState;
                    }

                    // Set desired outputs based on system state
                    switch (mSystemState) {
                        case IDLE:
                            if (mFrontFeeder.getBannerSensor() || getBannerSensor()) {
                                mFrontFeeder.setState(Feeder.State.IDLE);
                            } else {
                                mFrontFeeder.setState(Feeder.State.FEEDING_VERY_SLOW);
                            }
                            if (mBackFeeder.getBannerSensor() || getBannerSensor()) {
                                mBackFeeder.setState(Feeder.State.IDLE);
                            } else {
                                mBackFeeder.setState(Feeder.State.EXHAUSTING_VERY_SLOW);
                            }
                            mFrontIntake.setWantedState(Intake.WantedState.IDLE);
                            mBackIntake.setWantedState(Intake.WantedState.IDLE);
                            break;
                        case INTAKING:
                            mFrontFeeder.setState(Feeder.State.FEEDING);
                            mBackFeeder.setState(Feeder.State.EXHAUSTING_STOP);
                            mFrontIntake.setWantedState(Intake.WantedState.INTAKING);
                            mBackIntake.setWantedState(Intake.WantedState.IDLE);
                            break;
                        case INTAKING_FULL:
                            mFrontFeeder.setState(Feeder.State.IDLE);
                            mBackFeeder.setState(Feeder.State.IDLE);
                            mFrontIntake.setWantedState(Intake.WantedState.INTAKING);
                            mBackIntake.setWantedState(Intake.WantedState.IDLE);
                            break;
                        case INTAKING_EJECTING:
                            if (hasIncorrectBall() && Superstructure.getInstance().isReadyToEject()) {
                                mFrontFeeder.setState(Feeder.State.FEEDING);
                                mBackFeeder.setState(Feeder.State.UNJAM);
                            } else { // Set feeders to idle when last ejecting ball is in shooter
                                mFrontFeeder.setState(Feeder.State.IDLE);
                                mBackFeeder.setState(Feeder.State.IDLE);
                            }
                            mFrontIntake.setWantedState(Intake.WantedState.INTAKING);
                            mBackIntake.setWantedState(Intake.WantedState.IDLE);
                            break;
                        case EJECTING:
                            if (hasIncorrectBall() && Superstructure.getInstance().isReadyToEject()) {
                                mFrontFeeder.setState(Feeder.State.FEEDING);
                                mBackFeeder.setState(Feeder.State.UNJAM);
                            } else { // Set feeders to idle when last ejecting ball is in shooter
                                mFrontFeeder.setState(Feeder.State.IDLE);
                                mBackFeeder.setState(Feeder.State.IDLE);
                            }
                            mFrontIntake.setWantedState(Intake.WantedState.IDLE);
                            mBackIntake.setWantedState(Intake.WantedState.IDLE);
                            break;
                        case EJECTING_MANUAL:
                            mFrontFeeder.setState(Feeder.State.FEEDING);
                            mBackFeeder.setState(Feeder.State.UNJAM);
                            mFrontIntake.setWantedState(Intake.WantedState.IDLE);
                            mBackIntake.setWantedState(Intake.WantedState.IDLE);
                            break;
                        case SERIALIZING:
                            mFrontFeeder.setState(Feeder.State.FEEDING);
                            mBackFeeder.setState(Feeder.State.EXHAUSTING_STOP);
                            mFrontIntake.setWantedState(Intake.WantedState.IDLE);
                            mBackIntake.setWantedState(Intake.WantedState.IDLE);
                            break;
                        case EXHAUSTING:
                            if (mExhaustMode == ExhuastMode.LEFT || mExhaustMode == ExhuastMode.RIGHT) {
                                Feeder front = mRightFeeder;
                                Feeder back = mLeftFeeder;
                                Intake frontIntake = mRightIntake;
                                if (mExhaustMode == ExhuastMode.LEFT) {
                                    front = mLeftFeeder;
                                    back = mRightFeeder;
                                }
                                back.setState(Feeder.State.FEEDING);
                                front.setState(Feeder.State.EXHAUSTING);
                                frontIntake.setWantedState(Intake.WantedState.EXHAUSTING);
                            } else{
                                mFrontFeeder.setState(Feeder.State.EXHAUSTING);
                                mBackFeeder.setState(Feeder.State.EXHAUSTING);
                                mFrontIntake.setWantedState(Intake.WantedState.EXHAUSTING);
                                mBackIntake.setWantedState(Intake.WantedState.EXHAUSTING);
                            }
                            break;
                    }

                    // If superstructure is shooting and serializer is not ejecting, set feeders accordingly
                    boolean isShooting = Superstructure.getInstance().isShooting();
                    boolean isShootingAll = mShootingOffset.update(isShooting, kShootingOffsetTime);

                    // this enforces a minimum time between shots
                    if (timestamp - mLastFallingEdgeTimestamp > kShootingCooldown) {
                        if (isShooting && !isEjecting()) {
                            mFrontFeeder.setState(Feeder.State.SHOOTING);
                            if (isShootingAll || getNumBannersTriggered() < 2) {
                                mBackFeeder.setState(Feeder.State.SHOOTING);
                            } else {
                                mBackFeeder.setState(Feeder.State.IDLE);
                            }
                        }
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    private SystemState handleAll(double timestamp) {
        switch (mWantedState) {
            case INTAKING_LEFT:
                setDirection(Direction.LEFT);
                return handleIntaking(timestamp);
            case INTAKING_RIGHT:
                setDirection(Direction.RIGHT);
                return handleIntaking(timestamp);
            case EJECTING_LEFT:
                setDirection(Direction.LEFT);
                return SystemState.EJECTING_MANUAL;
            case EJECTING_RIGHT:
                setDirection(Direction.RIGHT);
                return SystemState.EJECTING_MANUAL;
            case EXHAUSTING:
                mFrontFeeder.clearColorVotes();
                mBackFeeder.clearColorVotes();
                return SystemState.EXHAUSTING;
            case IDLE:
            default:
                return handleIdle(timestamp);
        }
    }

    private void setDirection(Direction direction) {
        mFrontFeeder = direction == Direction.LEFT ? mLeftFeeder : mRightFeeder;
        mBackFeeder = direction == Direction.LEFT ? mRightFeeder : mLeftFeeder;
        mFrontIntake = direction == Direction.LEFT ? mLeftIntake : mRightIntake;
        mBackIntake = direction == Direction.LEFT ? mRightIntake : mLeftIntake;
    }

    private SystemState handleIdle(double timestamp) {
        boolean full = mBackFeeder.getBannerSensor() && getBannerSensor();
        if (shouldRejectBalls() && (hasIncorrectBall() || timestamp - mLastEjectTimestamp < kEjectBannerLetoff)) {
            return SystemState.EJECTING;
        } else if (!full && (mFrontFeeder.getBannerSensor() || getBannerSensor() || timestamp - mWantIdleTimestamp < kFinishSerializingTime)) {
            return SystemState.SERIALIZING;
        }
        return SystemState.IDLE;
    }

    private SystemState handleIntaking(double timestamp) {
        if (shouldRejectBalls() && (hasIncorrectBall() || timestamp - mLastEjectTimestamp < kEjectBannerLetoff)) {
            return SystemState.INTAKING_EJECTING;
        } else if (mBackFeeder.getBannerSensor() && getBannerSensor()) {
            return SystemState.INTAKING_FULL;
        }
        return SystemState.INTAKING;
    }

    private boolean shouldRejectBalls() {
        if (mForceNoEject) {
            return false;
        }
        ColorRejectionMode ballRejectionMode = mModeChooser.getSelected();
        if (ballRejectionMode == null) return mColorRejectionMode == ColorRejectionMode.AUTOMATIC_REJECTION;
        mColorRejectionMode = ballRejectionMode;
        return mColorRejectionMode == ColorRejectionMode.AUTOMATIC_REJECTION;
    }

    // Force a lockout of ball ejection
    public synchronized void setForceNoEject(boolean forceNoEject) {
        mForceNoEject = forceNoEject;
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putString("Serializer Wanted State", mWantedState.toString());
        SmartDashboard.putString("Serializer System State", mSystemState.toString());
        SmartDashboard.putBoolean("Upper Banner Sensor", mPeriodicIO.banner_sensor);

        SmartDashboard.putString("Incorrect Color", mIncorrectColor.toString());

        SmartDashboard.putBoolean("Have One Ball", getNumBannersTriggered() > 0);
        SmartDashboard.putBoolean("Have Two Balls", getNumBannersTriggered() > 1);
        SmartDashboard.putBoolean("HasIncorrect", hasIncorrectBall());
    }
}
