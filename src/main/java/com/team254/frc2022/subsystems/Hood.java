package com.team254.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.team254.frc2022.Constants;
import com.team254.lib.drivers.ServoMotorSubsystem;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood extends ServoMotorSubsystem {
    private static Hood mInstance;
    private LatchedBoolean mWasHoming = new LatchedBoolean();
    private double mHomingStartTime = Double.NaN;
    private double mHomingLimitStartTime = Double.NaN;
    private boolean mHoming = false;

    private static final double kMaxHomingTime = 10.0;
    private static final double kHomingHardstopTime = 1.0;

    public synchronized static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood(Constants.kHoodConstants);
        }

        return mInstance;
    }

    private Hood(final ServoMotorSubsystemConstants constants) {
        super(constants);

        // Tell the talon where the limit switch plugs in
        TalonUtil.checkError(
                mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen),
                mConstants.kName + ": Could not set reverse limit switch: ");
        // Disable limit switch so we can drive through the error overlap of the sensor
        mMaster.overrideLimitSwitchesEnable(false);
        mMaster.overrideSoftLimitsEnable(true);
        mMaster.setNeutralMode(NeutralMode.Coast);
    }

    public synchronized void setHoming() {
        mHoming = true;
        mHomingStartTime = Timer.getFPGATimestamp();
        mHomingLimitStartTime = Double.NaN;
        mHasBeenZeroed = false;

        mMaster.overrideLimitSwitchesEnable(false);
        mMaster.overrideSoftLimitsEnable(false);
        TalonUtil.checkError(mMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
                        true,
                        5,
                        10,
                        mConstants.kStatorPeakCurrentDuration)),
                mConstants.kName + ": Could not set stator current limit.");
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mWasHoming.update(!hasBeenZeroed())) {
            setHoming();
            System.out.println("Hood needs homing! Entering homing mode.");
        }

        if (mHoming) {
            if ((Timer.getFPGATimestamp() - mHomingStartTime) > kMaxHomingTime) {
                System.out.println("Timeout on hood homing! Zeroing!");
                zeroSensors();
            }

            mMaster.set(ControlMode.PercentOutput, -0.05,
                    DemandType.ArbitraryFeedForward, 0.0);
            if (atHomingLocation() && Double.isNaN(mHomingLimitStartTime)) {
                mHomingLimitStartTime = Timer.getFPGATimestamp();
            } else if (!atHomingLocation()) {
                mHomingLimitStartTime = Double.NaN;
            }
            if (!Double.isNaN(mHomingLimitStartTime) &&
                    (Timer.getFPGATimestamp() - mHomingLimitStartTime) > kHomingHardstopTime) {
                resetIfAtHome();
            }

            if (hasBeenZeroed()) {
                System.out.println("Hood homed successfully!");
                // Explicity turn off limit switch
                mMaster.overrideLimitSwitchesEnable(false);
                mMaster.overrideSoftLimitsEnable(true);
                TalonUtil.checkError(mMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
                                mConstants.kEnableStatorCurrentLimit,
                                mConstants.kStatorContinuousCurrentLimit,
                                mConstants.kStatorPeakCurrentLimit,
                                mConstants.kStatorPeakCurrentDuration)),
                        mConstants.kName + ": Could not set stator current limit.");
                mMaster.set(ControlMode.PercentOutput, 0.0,
                        DemandType.ArbitraryFeedForward, 0.0);

                mHoming = false;
            }
        } else {
            super.writePeriodicOutputs();
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mMaster.setNeutralMode(NeutralMode.Brake);
            }

            @Override
            public void onLoop(double timestamp) {

            }

            @Override
            public void onStop(double timestamp) {
                mMaster.setNeutralMode(NeutralMode.Coast);
            }
        });
        super.registerEnabledLoops(mEnabledLooper);
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized boolean isAtSetpoint() {
        return Util.epsilonEquals(mPeriodicIO.position_ticks, mPeriodicIO.demand, mConstants.kPositionDeadband);
    }

    @Override
    public boolean atHomingLocation() {
        return mMaster.getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Hood Angle", getPosition());
        SmartDashboard.putBoolean("Hood at Homing Sensor", atHomingLocation());
        SmartDashboard.putNumber("Hood Error", this.ticksToUnits(mPeriodicIO.error_ticks));
    }
}