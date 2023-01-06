package com.team254.lib.drivers;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.motion.IMotionProfileGoal;
import com.team254.lib.motion.MotionProfileConstraints;
import com.team254.lib.motion.MotionState;
import com.team254.lib.motion.SetpointGenerator;
import com.team254.lib.motion.SetpointGenerator.Setpoint;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstract base class for a subsystem with a single sensored servo-mechanism.
 */
public abstract class ServoMotorSubsystem extends Subsystem {
    private static final int kMotionProfileSlot = 0;
    private static final int kPositionPIDSlot = 1;

    // Recommend initializing in a static block!
    public static class TalonFXConstants {
        public CanDeviceId id = new CanDeviceId(-1);
        public boolean invert_motor = false;
        public boolean invert_sensor_phase = false;
        public int encoder_ppr = 2048;
    }

    // Recommend initializing in a static block!
    public static class ServoMotorSubsystemConstants {
        public String kName = "ERROR_ASSIGN_A_NAME";

        public double kLooperDt = 0.01;
        public int kCANTimeoutMs = 10; // use for important on the fly updates
        public int kLongCANTimeoutMs = 100; // use for constructors

        public TalonFXConstants kMasterConstants = new TalonFXConstants();
        public TalonFXConstants[] kSlaveConstants = new TalonFXConstants[0];

        public double kHomePosition = 0.0; // Units
        public double kTicksPerUnitDistance = 1.0;
        public double kKp = 0;  // Raw output / raw error
        public double kKi = 0;  // Raw output / sum of raw error
        public double kKd = 0;  // Raw output / (err - prevErr)
        public double kKf = 0;  // Raw output / velocity in ticks/100ms
        public double kKa = 0;  // Raw output / accel in (ticks/100ms) / s
        public double kMaxIntegralAccumulator = 0;
        public int kIZone = 0; // Ticks
        public int kDeadband = 0; // Ticks

        public double kPositionKp = 0;
        public double kPositionKi = 0;
        public double kPositionKd = 0;
        public double kPositionKf = 0;
        public double kPositionMaxIntegralAccumulator = 0;
        public int kPositionIZone = 0; // Ticks
        public int kPositionDeadband = 0; // Ticks

        public int kCruiseVelocity = 0; // Ticks / 100ms
        public int kAcceleration = 0; // Ticks / 100ms / s
        public double kRampRate = 0.0; // s
        public double kMaxVoltage = 12.0;

        public int kSupplyContinuousCurrentLimit = 20; // amps
        public int kSupplyPeakCurrentLimit = 60; // amps
        public double kSupplyPeakCurrentDuration = 0.2; // seconds
        public boolean kEnableSupplyCurrentLimit = false;

        public int kStatorContinuousCurrentLimit = 20; // amps
        public int kStatorPeakCurrentLimit = 60; // amps
        public double kStatorPeakCurrentDuration = 0.2; // seconds
        public boolean kEnableStatorCurrentLimit = false;

        public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
        public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;

        public int kStatusFrame8UpdateRate = 1000;
        public boolean kRecoverPositionOnReset = false;
    }

    protected final ServoMotorSubsystemConstants mConstants;
    protected final TalonFX mMaster;
    protected final TalonFX[] mSlaves;

    protected MotionState mMotionStateSetpoint = null;

    protected final int mForwardSoftLimitTicks;
    protected final int mReverseSoftLimitTicks;

    protected ServoMotorSubsystem(final ServoMotorSubsystemConstants constants) {
        mConstants = constants;
        mMaster = TalonFXFactory.createDefaultTalon(mConstants.kMasterConstants.id);
        mSlaves = new TalonFX[mConstants.kSlaveConstants.length];

        TalonUtil.checkError(mMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
                mConstants.kLongCANTimeoutMs), mConstants.kName + ": Could not detect encoder: ");

        mForwardSoftLimitTicks = (int) ((mConstants.kMaxUnitsLimit - mConstants.kHomePosition) * mConstants.kTicksPerUnitDistance);
        TalonUtil.checkError(
                mMaster.configForwardSoftLimitThreshold(mForwardSoftLimitTicks, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set forward soft limit: ");

        TalonUtil.checkError(mMaster.configForwardSoftLimitEnable(true, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not enable forward soft limit: ");

        mReverseSoftLimitTicks = (int) ((mConstants.kMinUnitsLimit - mConstants.kHomePosition) * mConstants.kTicksPerUnitDistance);
        TalonUtil.checkError(
                mMaster.configReverseSoftLimitThreshold(mReverseSoftLimitTicks, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set reverse soft limit: ");

        TalonUtil.checkError(mMaster.configReverseSoftLimitEnable(true, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not enable reverse soft limit: ");

        TalonUtil.checkError(mMaster.configVoltageCompSaturation(12.0, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set voltage compensation saturation: ");

        TalonUtil.checkError(mMaster.config_kP(kMotionProfileSlot, mConstants.kKp, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": could not set kP: ");

        TalonUtil.checkError(mMaster.config_kI(kMotionProfileSlot, mConstants.kKi, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": could not set kI: ");

        TalonUtil.checkError(mMaster.config_kD(kMotionProfileSlot, mConstants.kKd, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": could not set kD: ");

        TalonUtil.checkError(mMaster.config_kF(kMotionProfileSlot, mConstants.kKf, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set kF: ");

        TalonUtil.checkError(mMaster.configMaxIntegralAccumulator(kMotionProfileSlot, mConstants.kMaxIntegralAccumulator,
                mConstants.kLongCANTimeoutMs), mConstants.kName + ": Could not set max integral: ");

        TalonUtil.checkError(mMaster.config_IntegralZone(kMotionProfileSlot, mConstants.kIZone, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set i zone: ");

        TalonUtil.checkError(
                mMaster.configAllowableClosedloopError(kMotionProfileSlot, mConstants.kDeadband, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set deadband: ");

        TalonUtil.checkError(mMaster.config_kP(kPositionPIDSlot, mConstants.kPositionKp, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": could not set kP: ");

        TalonUtil.checkError(mMaster.config_kI(kPositionPIDSlot, mConstants.kPositionKi, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": could not set kI: ");

        TalonUtil.checkError(mMaster.config_kD(kPositionPIDSlot, mConstants.kPositionKd, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": could not set kD: ");

        TalonUtil.checkError(mMaster.config_kF(kPositionPIDSlot, mConstants.kPositionKf, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set kF: ");

        TalonUtil.checkError(mMaster.configMaxIntegralAccumulator(kPositionPIDSlot, mConstants.kPositionMaxIntegralAccumulator,
                mConstants.kLongCANTimeoutMs), mConstants.kName + ": Could not set max integral: ");

        TalonUtil.checkError(mMaster.config_IntegralZone(kPositionPIDSlot, mConstants.kPositionIZone, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set i zone: ");

        TalonUtil.checkError(
                mMaster.configAllowableClosedloopError(kPositionPIDSlot, mConstants.kPositionDeadband, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set deadband: ");

        TalonUtil.checkError(
                mMaster.configMotionCruiseVelocity(mConstants.kCruiseVelocity, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set cruise velocity: ");

        TalonUtil.checkError(mMaster.configMotionAcceleration(mConstants.kAcceleration, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set acceleration: ");

        TalonUtil.checkError(mMaster.configOpenloopRamp(mConstants.kRampRate, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set voltage ramp rate: ");

        TalonUtil.checkError(mMaster.configClosedloopRamp(mConstants.kRampRate, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set closed loop ramp rate: ");

        TalonUtil.checkError(mMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                        mConstants.kEnableSupplyCurrentLimit,
                        mConstants.kSupplyContinuousCurrentLimit,
                        mConstants.kSupplyPeakCurrentLimit,
                        mConstants.kSupplyPeakCurrentDuration)),
                mConstants.kName + ": Could not set supply current limit.");

        TalonUtil.checkError(mMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
                        mConstants.kEnableStatorCurrentLimit,
                        mConstants.kStatorContinuousCurrentLimit,
                        mConstants.kStatorPeakCurrentLimit,
                        mConstants.kStatorPeakCurrentDuration)),
                mConstants.kName + ": Could not set stator current limit.");

        mMaster.configVoltageMeasurementFilter(8);


        TalonUtil.checkError(
                mMaster.configVoltageCompSaturation(mConstants.kMaxVoltage, mConstants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set voltage comp saturation.");
        mMaster.enableVoltageCompensation(true);

        mMaster.setInverted(mConstants.kMasterConstants.invert_motor);
        mMaster.setSensorPhase(mConstants.kMasterConstants.invert_sensor_phase);
        mMaster.setNeutralMode(NeutralMode.Brake);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 20);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 1000, 20);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, mConstants.kStatusFrame8UpdateRate, 20);

        // Start with kMotionProfileSlot.
        mMaster.selectProfileSlot(kMotionProfileSlot, 0);

        for (int i = 0; i < mSlaves.length; ++i) {
            mSlaves[i] = TalonFXFactory.createPermanentSlaveTalon(mConstants.kSlaveConstants[i].id, mConstants.kMasterConstants.id);
            mSlaves[i].setInverted(mConstants.kSlaveConstants[i].invert_motor);
            mSlaves[i].setNeutralMode(NeutralMode.Brake);
            mSlaves[i].follow(mMaster);
        }

        // The accel term can re-use the velocity unit conversion because both input and output units are per second.
        mMotionProfileConstraints = new MotionProfileConstraints(ticksPer100msToUnitsPerSecond(mConstants.kCruiseVelocity),
            ticksPer100msToUnitsPerSecond(-mConstants.kCruiseVelocity),
            ticksPer100msToUnitsPerSecond(mConstants.kAcceleration));

        // Send a neutral command.
        stop();
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double position_ticks;
        public double position_units;
        public double velocity_ticks_per_100ms;
        public double active_trajectory_position; // ticks
        public double active_trajectory_velocity; // ticks/100ms
        public double active_trajectory_acceleration; // ticks/100ms/s
        public double output_percent;
        public double output_voltage;
        public double master_current;
        public double error_ticks;
        public int encoder_wraps;
        public int absolute_pulse_offset = 0;
        // public int absolute_pulse_position;
        public int absolute_pulse_position_modded;
        public boolean reset_occured;

        // OUTPUTS
        public double demand;
        public double feedforward;
    }

    protected enum ControlState {
        OPEN_LOOP, MOTION_MAGIC, POSITION_PID, MOTION_PROFILING
    }

    protected PeriodicIO mPeriodicIO = new PeriodicIO();
    protected ControlState mControlState = ControlState.OPEN_LOOP;
    protected ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    protected boolean mHasBeenZeroed = false;
    protected StickyFaults mFaults = new StickyFaults();
    protected SetpointGenerator mSetpointGenerator = new SetpointGenerator();
    protected MotionProfileConstraints mMotionProfileConstraints;

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        if (mMaster.hasResetOccurred()) {
            DriverStation.reportError(mConstants.kName + ": Talon Reset! ", false);
            mPeriodicIO.reset_occured = true;
            return;
        } else {
            mPeriodicIO.reset_occured = false;
        }

        mMaster.getStickyFaults(mFaults);
        if (mFaults.hasAnyFault()) {
            DriverStation.reportError(mConstants.kName + ": Talon Fault! " + mFaults.toString(), false);
            mMaster.clearStickyFaults(0);
        }
        if (mMaster.getControlMode() == ControlMode.MotionMagic) {
            mPeriodicIO.active_trajectory_position = mMaster.getActiveTrajectoryPosition();

            if (mPeriodicIO.active_trajectory_position < mReverseSoftLimitTicks) {
                DriverStation.reportError(mConstants.kName + ": Active trajectory past reverse soft limit!", false);
            } else if (mPeriodicIO.active_trajectory_position > mForwardSoftLimitTicks) {
                DriverStation.reportError(mConstants.kName + ": Active trajectory past forward soft limit!", false);
            }
            final double newVel = mMaster.getActiveTrajectoryVelocity();
            if (Util.epsilonEquals(newVel, mConstants.kCruiseVelocity, Math.max(1, mConstants.kDeadband)) || Util
                    .epsilonEquals(newVel, mPeriodicIO.active_trajectory_velocity, Math.max(1, mConstants.kDeadband))) {
                // Mechanism is ~constant velocity.
                mPeriodicIO.active_trajectory_acceleration = 0.0;
            } else {
                // Mechanism is accelerating.
                mPeriodicIO.active_trajectory_acceleration = Math
                        .signum(newVel - mPeriodicIO.active_trajectory_velocity) * mConstants.kAcceleration;
            }
            mPeriodicIO.active_trajectory_velocity = newVel;
        } else {
            mPeriodicIO.active_trajectory_position = Integer.MIN_VALUE;
            mPeriodicIO.active_trajectory_velocity = 0;
            mPeriodicIO.active_trajectory_acceleration = 0.0;
        }
        if (mMaster.getControlMode() == ControlMode.Position) {
            mPeriodicIO.error_ticks = mMaster.getClosedLoopError(0);
        } else {
            mPeriodicIO.error_ticks = 0;
        }
        mPeriodicIO.master_current = mMaster.getStatorCurrent();
        mPeriodicIO.output_voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.output_percent = mMaster.getMotorOutputPercent();
        mPeriodicIO.position_ticks = mMaster.getSelectedSensorPosition(0);
        mPeriodicIO.position_units = ticksToHomedUnits(mPeriodicIO.position_ticks);
        mPeriodicIO.velocity_ticks_per_100ms = mMaster.getSelectedSensorVelocity(0);

        if (mConstants.kRecoverPositionOnReset) {
            mPeriodicIO.absolute_pulse_position_modded = (int) mMaster.getSensorCollection().getIntegratedSensorAbsolutePosition();
            if (mPeriodicIO.absolute_pulse_position_modded < 0) {
                mPeriodicIO.absolute_pulse_position_modded += mConstants.kMasterConstants.encoder_ppr;
            }

            double estimated_pulse_pos = ((mConstants.kMasterConstants.invert_sensor_phase ? -1 : 1) * mPeriodicIO.position_ticks) + mPeriodicIO.absolute_pulse_offset;
            int new_wraps = (int) Math.floor(estimated_pulse_pos / ((double) mConstants.kMasterConstants.encoder_ppr));
            // Only set this when we are really sure its a valid change
            if (Math.abs(mPeriodicIO.encoder_wraps - new_wraps) <= 1) {
                mPeriodicIO.encoder_wraps = new_wraps;
            }
        }

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    /**
     * @return absolute encoders raw ticks bounded to one rotation
     */
    protected int getAbsoluteEncoderRawPosition() {
        int abs_raw_with_rollover = (int) mMaster.getSensorCollection().getIntegratedSensorAbsolutePosition();
        return abs_raw_with_rollover + (abs_raw_with_rollover < 0 ? abs_raw_with_rollover + mConstants.kMasterConstants.encoder_ppr : 0);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mControlState == ControlState.MOTION_MAGIC) {
            mMaster.set(ControlMode.MotionMagic, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.feedforward);
        } else if (mControlState == ControlState.POSITION_PID || mControlState == ControlState.MOTION_PROFILING) {
            mMaster.set(ControlMode.Position, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.feedforward);
        } else {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.feedforward);
        }
    }

    public synchronized void handleMasterReset(boolean reset) {}

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // if (mCSVWriter == null) {
                //     mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/"
                //             + mConstants.kName.replaceAll("[^A-Za-z0-9]+", "").toUpperCase() + "-LOGS.csv",
                //             PeriodicIO.class);
                // }
            }

            @Override
            public void onLoop(double timestamp) {
                if (mPeriodicIO.reset_occured) {
                    System.out.println(mConstants.kName + ": Master Talon reset occurred; resetting frame rates.");
                    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 20);
                    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
                    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, mConstants.kStatusFrame8UpdateRate, 20);

                    // Reset encoder position to estimated position from absolute encoder
                    if (mConstants.kRecoverPositionOnReset) {
                        mMaster.setSelectedSensorPosition(estimateSensorPositionFromAbsolute(), 0, mConstants.kCANTimeoutMs);
                    }
                }
                handleMasterReset(mPeriodicIO.reset_occured);
                for (TalonFX slave : mSlaves) {
                    if (slave.hasResetOccurred()) {
                        System.out.println(mConstants.kName + ": Slave Talon reset occurred");
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                if (mCSVWriter != null) {
                    mCSVWriter.flush();
                    mCSVWriter = null;
                }

                stop();
            }
        });
    }

    public synchronized double getPositionTicks() {
        return mPeriodicIO.position_ticks;
    }

    // In "Units"
    public synchronized double getPosition() {
        return ticksToHomedUnits(mPeriodicIO.position_ticks);
    }

    // In "Units per second"
    public synchronized double getVelocity() {
        return ticksToUnits(mPeriodicIO.velocity_ticks_per_100ms) * 10.0;
    }

    public synchronized boolean hasFinishedTrajectory() {
        return Util.epsilonEquals(mPeriodicIO.active_trajectory_position, ticksToUnits(getSetpoint()),
                Math.max(1, mConstants.kDeadband));
    }

    public synchronized double getSetpoint() {
        return (mControlState == ControlState.MOTION_MAGIC ||
                mControlState == ControlState.POSITION_PID ||
                mControlState == ControlState.MOTION_PROFILING) ?
                ticksToUnits(mPeriodicIO.demand) : Double.NaN;
    }

    public synchronized double getSetpointHomed() {
        return (mControlState == ControlState.MOTION_MAGIC ||
                mControlState == ControlState.POSITION_PID ||
                mControlState == ControlState.MOTION_PROFILING) ?
                ticksToHomedUnits(mPeriodicIO.demand) : Double.NaN;
    }

    public synchronized void setSetpointMotionMagic(double units, double feedforward_v) {
        mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
        mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kKf + mConstants.kKd / 100.0) / 1023.0;
        if (mControlState != ControlState.MOTION_MAGIC) {
            mMaster.selectProfileSlot(kMotionProfileSlot, 0);
            mControlState = ControlState.MOTION_MAGIC;
        }
    }

    public synchronized void setSetpointMotionMagic(double units) {
        setSetpointMotionMagic(units, 0.0);
    }

    public synchronized void setSetpointPositionPID(double units, double feedforward_v) {
        mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
        double feedforward_ticks_per_100ms = unitsPerSecondToTicksPer100ms(feedforward_v);
        mPeriodicIO.feedforward = feedforward_ticks_per_100ms * (mConstants.kKf + mConstants.kKd / 100.0) / 1023.0;
        if (mControlState != ControlState.POSITION_PID) {
            mMaster.selectProfileSlot(kPositionPIDSlot, 0);
            mControlState = ControlState.POSITION_PID;
        }
    }

    public synchronized void setSetpointPositionPID(double units) {
        setSetpointPositionPID(units, 0.0);
    }

    public synchronized void setSetpointMotionProfiling(IMotionProfileGoal goal, double feedforward_v) {
        if (mControlState != ControlState.MOTION_PROFILING) {
            mMaster.selectProfileSlot(kPositionPIDSlot, 0);
            mControlState = ControlState.MOTION_PROFILING;
            mMotionStateSetpoint = new MotionState(mPeriodicIO.timestamp, mPeriodicIO.position_units, ticksPer100msToUnitsPerSecond(mPeriodicIO.velocity_ticks_per_100ms), 0.0);
            mSetpointGenerator.reset();
        }
        Setpoint setpoint = mSetpointGenerator.getSetpoint(mMotionProfileConstraints, goal, mMotionStateSetpoint, mPeriodicIO.timestamp + mConstants.kLooperDt);
        mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(setpoint.motion_state.pos()));
        mPeriodicIO.feedforward = (unitsPerSecondToTicksPer100ms(feedforward_v + setpoint.motion_state.vel()) * mConstants.kKf +
            unitsPerSecondToTicksPer100ms(setpoint.motion_state.acc()) * mConstants.kKa) / 1023.0;
        mMotionStateSetpoint = setpoint.motion_state;
    }

    protected double ticksToUnits(double ticks) {
        return ticks / mConstants.kTicksPerUnitDistance;
    }

    protected double ticksToHomedUnits(double ticks) {
        double val = ticksToUnits(ticks);
        return val + mConstants.kHomePosition;
    }

    protected double unitsToTicks(double units) {
        return units * mConstants.kTicksPerUnitDistance;
    }

    protected double homeAwareUnitsToTicks(double units) {
        return unitsToTicks(units - mConstants.kHomePosition);
    }

    protected double constrainTicks(double ticks) {
        return Util.limit(ticks, mReverseSoftLimitTicks, mForwardSoftLimitTicks);
    }

    protected double ticksPer100msToUnitsPerSecond(double ticks_per_100ms) {
        return ticksToUnits(ticks_per_100ms) * 10.0;
    }

    protected double unitsPerSecondToTicksPer100ms(double units_per_second) {
        return unitsToTicks(units_per_second) / 10.0;
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }
    }

    public synchronized double getActiveTrajectoryUnits() {
        return ticksToHomedUnits(mPeriodicIO.active_trajectory_position);
    }

    public synchronized double getActiveTrajectoryVelocityUnitsPerSec() {
        return ticksPer100msToUnitsPerSecond(mPeriodicIO.active_trajectory_velocity);
    }

    public synchronized double getPredictedPositionUnits(double lookahead_secs) {
        if (mMaster.getControlMode() != ControlMode.MotionMagic) {
            return getPosition();
        }

        double predicted_units = ticksToHomedUnits(mPeriodicIO.active_trajectory_position +
                lookahead_secs * mPeriodicIO.active_trajectory_velocity +
                0.5 * mPeriodicIO.active_trajectory_acceleration * lookahead_secs * lookahead_secs);
        if (mPeriodicIO.demand >= mPeriodicIO.active_trajectory_position) {
            return Math.min(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
        } else {
            return Math.max(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
        }
    }

    public boolean atHomingLocation() {
        return false;
    }

    public synchronized void resetIfAtHome() {
        if (atHomingLocation()) {
            zeroSensors();
        }
    }

    @Override
    public synchronized void zeroSensors() {
        mMaster.setSelectedSensorPosition(0, 0, mConstants.kCANTimeoutMs);
        mPeriodicIO.absolute_pulse_offset = getAbsoluteEncoderRawPosition();
        mHasBeenZeroed = true;
    }

    public synchronized void forceZero() {
        mMaster.setSelectedSensorPosition(0, 0, mConstants.kCANTimeoutMs);
        mPeriodicIO.absolute_pulse_offset = getAbsoluteEncoderRawPosition();
    }

    public synchronized boolean hasBeenZeroed() {
        return mHasBeenZeroed;
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
        mMaster.set(ControlMode.PercentOutput, 0.0);
    }

    public int estimateSensorPositionFromAbsolute() {
        int estimated_pulse_pos = (mPeriodicIO.encoder_wraps * mConstants.kMasterConstants.encoder_ppr) + mPeriodicIO.absolute_pulse_position_modded;
        int estimate_position_ticks = (mConstants.kMasterConstants.invert_sensor_phase ? -1 : 1) * (estimated_pulse_pos - mPeriodicIO.absolute_pulse_offset);
        return estimate_position_ticks;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber(mConstants.kName + ": Position (units)", mPeriodicIO.position_units);
        SmartDashboard.putBoolean(mConstants.kName + ": Homing Location", atHomingLocation());
        // synchronized (this) {
        //     if (mCSVWriter != null) {
        //         mCSVWriter.write();
        //     }
        // }
    }
}