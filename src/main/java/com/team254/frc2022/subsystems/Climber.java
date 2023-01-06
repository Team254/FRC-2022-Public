package com.team254.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2022.Constants;
import com.team254.frc2022.subsystems.Serializer.SystemState;
import com.team254.lib.drivers.LazySolenoid;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.util.TimeDelayedBoolean;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class Climber extends Subsystem {
    private Drive mDrive = null;

    private static final double kElevatorAbsoluteMaxExtension = 37.45675 / Constants.kClimberTicksToInches; // (inches -> ticks) allegedly 38.625"
    private static final double kElevatorAbsoluteMinExtension = 0; // (inches -> ticks)

    private static final double kElevatorMaxExtension = kElevatorAbsoluteMaxExtension * 0.99;           // (ticks)

    private static final double kSacHooksStowPositionInches = 4.9254;   // TODO THIS IS ONLY FOR SAC HOOKS (v4, v5)!!
    private static final double kPostSacHooksStowPositionInches = 5.291;    // TODO THIS IS ONLY FOR POST SAC HOOKS (v6)!!
    private static final double kPostPostSacHooksStowPositionInches = kSacHooksStowPositionInches + 0.7658;    // TODO THIS IS ONLY FOR POST POST SAC HOOKS (v7)!!

    private static final double kElevatorStartPosition = kPostPostSacHooksStowPositionInches / Constants.kClimberTicksToInches;      // (inches -> ticks)  TODO replace with homing

    private static final double kElevatorMinUnlatchPosition = 7.600 / Constants.kClimberTicksToInches;  // (inches -> ticks)

    private static final double kPreclimbPosition = 18.146 / Constants.kClimberTicksToInches;           // (inches -> ticks)  allegedly 19.5"
    private static final double kTailLatchedOnThreshold = 6.0 / Constants.kClimberTicksToInches;
    private static final double kL4MinPosition = 0;
    private static final double kL2MinPosition = 4.0 / Constants.kClimberTicksToInches;               // (inches -> ticks)

    private static Optional<Double> mUnlatchStartTime = Optional.empty();
    private static final double kMinUnlatchTime = 1;    // (s)

    private static final double kAllowableExtensionError = 1.315 / Constants.kClimberTicksToInches;     // (inches -> ticks)
    private static final double kAllowableRetractionError = 0.625 / Constants.kClimberTicksToInches;    // (inches -> ticks) roughly tuned 4/1

    private static final double kMinimumAngleDeployStinger = -2.5;  // deg
    private static final double kMinimumHoldAngleDeployStingerTime = 0.5;  // s

    private static final boolean kExtendStingerValue = true;

    private static Climber mInstance = null;

    public static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public enum SystemState {
        IDLE(Double.NaN, !kExtendStingerValue, true),
        PRE_CLIMB(kPreclimbPosition, !kExtendStingerValue, false),
        EXTENDING(kElevatorMaxExtension, !kExtendStingerValue, false),
        EXTENDED(kElevatorMaxExtension, !kExtendStingerValue, false),
        RETRACTING_L2(kL2MinPosition, !kExtendStingerValue, false),
        RETRACTING_L4(kL4MinPosition, kExtendStingerValue, false),
        MANUAL(Double.NaN, !kExtendStingerValue, false),
        SWITCHING_BAR(kElevatorMaxExtension * 0.8, kExtendStingerValue, true);

        public final double climber_setpoint;
        public final boolean stinger_extend_demand;
        public final boolean coast;

        SystemState(double climber_setpoint, boolean stinger_extend_demand, boolean coast) {
            this.climber_setpoint = climber_setpoint;
            this.stinger_extend_demand = stinger_extend_demand;
            this.coast = coast;
        }
    }

    public enum WantedState {
        IDLE, PRE_CLIMB, EXTEND, CLIMB_L2, CLIMB_L4, MANUAL
    }

    private WantedState mWantedState = WantedState.IDLE;
    private SystemState mSystemState = SystemState.IDLE;

    private final TalonFX mClimberMaster;
    private final Solenoid mStingerExtend;

    private final int kDefaultPositionPIDSlot = 0;
    private final int kSwitchBarPositionPIDSlot = 1;

    private Climber() {
        mDrive = Drive.getInstance();
        mClimberMaster = TalonFXFactory.createDefaultTalon(Constants.kClimberMasterId);

        mClimberMaster.config_kP(kDefaultPositionPIDSlot, Constants.kClimberKp);
        mClimberMaster.config_kI(kDefaultPositionPIDSlot, Constants.kClimberKi);
        mClimberMaster.config_kD(kDefaultPositionPIDSlot, Constants.kClimberKd);

        mClimberMaster.config_kP(kSwitchBarPositionPIDSlot, Constants.kClimberTraversalTransferKp);
        mClimberMaster.config_kI(kSwitchBarPositionPIDSlot, Constants.kClimberTraversalTransferKi);
        mClimberMaster.config_kD(kSwitchBarPositionPIDSlot, Constants.kClimberTraversalTransferKd);

        mClimberMaster.selectProfileSlot(kDefaultPositionPIDSlot, 0);

        mClimberMaster.setSensorPhase(true);
        mClimberMaster.setInverted(true);
        mClimberMaster.configForwardSoftLimitThreshold(kElevatorMaxExtension);
        mClimberMaster.configForwardSoftLimitEnable(true);

        mClimberMaster.setSelectedSensorPosition(kElevatorStartPosition);

        mClimberMaster.setNeutralMode(NeutralMode.Coast);
        mClimberMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        mClimberMaster.overrideSoftLimitsEnable(true);
        mClimberMaster.overrideLimitSwitchesEnable(true);

        mStingerExtend = new LazySolenoid(PneumaticsModuleType.REVPH, Constants.kTailExtendId);

        mClimberMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 80, 120, 3));
    }

    public static class PeriodicIO {
        public double climber_setpoint;
        public boolean stinger_extend_demand;
        public double open_loop_demand;

        public double climber_position;

        public double state_start_time;

        public boolean setCoast;   // only exists if want to change neutral mode
    }

    private final PeriodicIO mPeriodicIO = new PeriodicIO();

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.climber_position = mClimberMaster.getSelectedSensorPosition();
    }

    private boolean forceBreak = false;
    public synchronized void setForceBreak(boolean force) {
        forceBreak = force;
        prevSetCoast = !mPeriodicIO.setCoast;
    }

    private Boolean prevSetCoast = null;
    private boolean wantForceDeployStinger = false;
    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.setCoast != prevSetCoast) {
            if (!forceBreak) {
                mClimberMaster.setNeutralMode(mPeriodicIO.setCoast ? NeutralMode.Coast : NeutralMode.Brake);
            } else {
                System.out.println("Forcing break!");
                mClimberMaster.setNeutralMode(NeutralMode.Brake);
            }
            prevSetCoast = mPeriodicIO.setCoast;
        }

        if (mSystemState == SystemState.IDLE) {
            mClimberMaster.set(ControlMode.PercentOutput, 0);
            mStingerExtend.set(mPeriodicIO.stinger_extend_demand);
        } else {
            if (wantForceDeployStinger) {
                mStingerExtend.set(kExtendStingerValue);
            } else {
                mStingerExtend.set(mPeriodicIO.stinger_extend_demand);
            }

            if (mSystemState == SystemState.SWITCHING_BAR) {
                if (mPeriodicIO.climber_position < kTailLatchedOnThreshold) {
                    mClimberMaster.set(ControlMode.PercentOutput, 0.30);
                } else if (mPeriodicIO.climber_position < mSystemState.climber_setpoint) {
                    mClimberMaster.set(ControlMode.PercentOutput, 0.65);
                } else {
                    mClimberMaster.set(ControlMode.PercentOutput, 0);
                }
            } else if (mSystemState == SystemState.MANUAL) {
                if (!isUnlatched() && mPeriodicIO.open_loop_demand < 0) {
                    mClimberMaster.set(ControlMode.PercentOutput, 0);
                } else {
                    mClimberMaster.set(ControlMode.PercentOutput, mPeriodicIO.open_loop_demand);
                }
            } else {
                mClimberMaster.set(ControlMode.Position, mPeriodicIO.climber_setpoint);
            }
        }
    }

    public void setManualStingerDeploy() {
        wantForceDeployStinger = true;
    }

    public boolean isUnlatched() {
        if (mUnlatchStartTime.isPresent()) {
            return Timer.getFPGATimestamp() - mUnlatchStartTime.get() >= kMinUnlatchTime;
        }
        return false;
    }

    public void setElevatorAtStartPos() {
        mClimberMaster.setSelectedSensorPosition(kElevatorStartPosition);
    }

    public void updateManualInput(double inp) {    // should be called on loop
        mPeriodicIO.open_loop_demand = inp * 0.8;
        if (Math.abs(inp) > 0.3) {
            setWantedState(WantedState.MANUAL);
        }
    }

    public boolean withinSwitchBarPos() {
        return Math.abs(mPeriodicIO.climber_position - kL4MinPosition) <= kAllowableRetractionError;
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mPeriodicIO.setCoast = mSystemState.coast;
                prevSetCoast = !mPeriodicIO.setCoast;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Climber.this) {
                    if (!isUnlatched()) {
                        if (mPeriodicIO.climber_position < kElevatorMinUnlatchPosition) {
                            mUnlatchStartTime = Optional.empty();
                        } else if (mUnlatchStartTime.isEmpty()) {
                            mUnlatchStartTime = Optional.of(Timer.getFPGATimestamp());
                        }
                    }

                    // get new system state
                    SystemState newState = handleStateTransition();
                    if (newState != mSystemState) {
                        System.out.println("Transition: " + mSystemState + " to " + newState + "\n\tTime in state: " + -(mPeriodicIO.state_start_time - Timer.getFPGATimestamp()));
                        mPeriodicIO.state_start_time = Timer.getFPGATimestamp();

                        if (newState == SystemState.SWITCHING_BAR) {
                            mClimberMaster.selectProfileSlot(kSwitchBarPositionPIDSlot, 0);
                        }
                        mPeriodicIO.setCoast = newState.coast;
                        mSystemState = newState;
                    }

                    mPeriodicIO.climber_setpoint = newState.climber_setpoint;
                    mPeriodicIO.stinger_extend_demand = newState.stinger_extend_demand;
                }
            }

            @Override
            public void onStop(double timestamp) {
                if (mSystemState == SystemState.RETRACTING_L4) {
                    mClimberMaster.setNeutralMode(NeutralMode.Coast);
                }
                stop();
                mWantedState = WantedState.IDLE;
                mSystemState = SystemState.IDLE;
            }
        });
    }

    private SystemState handleStateTransition() {
        double timeInState = Timer.getFPGATimestamp() - mPeriodicIO.state_start_time;
        switch (mWantedState) {
            case MANUAL:
                return SystemState.MANUAL;
            case IDLE:
                return SystemState.IDLE;
            case PRE_CLIMB:
                return handleWantPreclimb();
            case EXTEND:
                return handleWantExtend(timeInState);
            case CLIMB_L2:
                return handleWantClimbL2(timeInState);
            case CLIMB_L4:
                return handleWantClimbL4(timeInState);
            default:
                System.out.println("Unexpected Climber State: " + mWantedState);
                return SystemState.IDLE;
        }
    }

    public void wantPreclimb() {
        setWantedState(WantedState.PRE_CLIMB);
    }

    public void wantExtend() {
        setWantedState(WantedState.EXTEND);
    }

    public void wantClimbL4() {
        setWantedState(WantedState.CLIMB_L4);
    }

    public void wantClimbL2() {
        setWantedState(WantedState.CLIMB_L2);
    }

    public void setWantedState(WantedState newState) {
        // latch into abort
        if (mWantedState != newState) {
            System.out.println("Setting wanted state..." + newState);
            mWantedState = newState;
        }
    }

    public boolean isInExtended() {
        return Math.abs(mPeriodicIO.climber_setpoint - kElevatorMaxExtension) <= kAllowableExtensionError;
    }

    TimeDelayedBoolean posDetector = new TimeDelayedBoolean();
    public SystemState handleWantClimbL4(double timeInState) {
        if (!isUnlatched()) {
            return SystemState.EXTENDING;
        }
        switch (mSystemState) {
            case EXTENDED:
            case RETRACTING_L2:
                return SystemState.RETRACTING_L4;
            case RETRACTING_L4:
                if (posDetector.update(withinSwitchBarPos(), kMinimumHoldAngleDeployStingerTime)
                        /*&& pitchDetector.update(
                                mDrive.getRoll().getDegrees() >= kMinimumAngleDeployStinger,
                                        kMinimumHoldAngleDeployStingerTime)*/) {
                    return SystemState.SWITCHING_BAR;
                }
                return SystemState.RETRACTING_L4;
            case SWITCHING_BAR:
                return SystemState.SWITCHING_BAR;
            case IDLE:
            case PRE_CLIMB:
            case EXTENDING:
                setWantedState(WantedState.EXTEND);
                return SystemState.EXTENDING;
            case MANUAL:
                if (mPeriodicIO.climber_position >= kElevatorMaxExtension) { // todo incorporate gyro pitch?
                    return SystemState.RETRACTING_L4;
                }
                setWantedState(WantedState.EXTEND);
                return SystemState.EXTENDING;
            default:
                System.out.println("Unexpected Climber State: " + mWantedState);
                return SystemState.IDLE;
        }
    }

    public SystemState handleWantClimbL2(double timeInState) {
        if (!isUnlatched()) {
            return SystemState.EXTENDING;
        }
        switch (mSystemState) {
            case EXTENDING:
                setWantedState(WantedState.EXTEND);
            case IDLE:
            case PRE_CLIMB:
                return SystemState.EXTENDING;
            case EXTENDED:
            case RETRACTING_L2:
            case RETRACTING_L4:
                return SystemState.RETRACTING_L2;
            case SWITCHING_BAR:
                return SystemState.SWITCHING_BAR;
            case MANUAL:
                if (mPeriodicIO.climber_position >= kElevatorMaxExtension) { // todo incorporate gyro pitch?
                    return SystemState.RETRACTING_L2;
                }
                setWantedState(WantedState.EXTEND);
                return SystemState.EXTENDING;
            default:
                System.out.println("Unexpected Climber State: " + mWantedState);
                return SystemState.IDLE;
        }
    }

    public SystemState handleWantExtend(double timeInState) {
        switch (mSystemState) {
            case IDLE:
            case PRE_CLIMB:
            case MANUAL:
                return SystemState.EXTENDING;
            case EXTENDED:
                return SystemState.EXTENDED;
            case RETRACTING_L2:
            case RETRACTING_L4:
                return SystemState.RETRACTING_L2;
            case EXTENDING:
                if (isInExtended()) {
                    return SystemState.EXTENDED;
                }
                mDrive.resetRoll();
                return SystemState.EXTENDING;
            case SWITCHING_BAR:
                return SystemState.SWITCHING_BAR;
            default:
                System.out.println("Unexpected Climber State: " + mWantedState);
                return SystemState.IDLE;
        }
    }

    public SystemState handleWantPreclimb() {
        switch (mSystemState) {
            case IDLE:
            case MANUAL:
            case PRE_CLIMB:
                return SystemState.PRE_CLIMB;
            case EXTENDING:
            case EXTENDED:
            case RETRACTING_L2:
            case SWITCHING_BAR:
                return mSystemState;
            case RETRACTING_L4:
                return SystemState.RETRACTING_L2;
            default:
                System.out.println("Unexpected Climber State: " + mWantedState);
                return SystemState.IDLE;
        }
    }

    @Override
    public void stop() {
        mSystemState = SystemState.IDLE;
    }

    public void abort() {
        setWantedState(WantedState.IDLE);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }


    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Climber Setpoint Ticks", mPeriodicIO.climber_setpoint);
        SmartDashboard.putNumber("Climber Setpoint Inches", mPeriodicIO.climber_setpoint * Constants.kClimberTicksToInches);
        SmartDashboard.putNumber("Climber Position Ticks", mPeriodicIO.climber_position);
        SmartDashboard.putNumber("Climber Position Inches", mPeriodicIO.climber_position * Constants.kClimberTicksToInches);

        SmartDashboard.putBoolean("Climber Unlatched", isUnlatched());
        SmartDashboard.putBoolean("Extend Stinger State", mPeriodicIO.stinger_extend_demand == kExtendStingerValue);
        SmartDashboard.putNumber("Climber Open Loop Demand", mPeriodicIO.open_loop_demand);

        SmartDashboard.putBoolean("Elevator Reverse Limit Switch", mClimberMaster.getSensorCollection().isRevLimitSwitchClosed() == 1);

        SmartDashboard.putString("Climber current state", mSystemState.toString());
        SmartDashboard.putString("Climber wanted state", mWantedState.toString());
    }
}