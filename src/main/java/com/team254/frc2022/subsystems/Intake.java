package com.team254.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2022.Constants;
import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.drivers.LazySolenoid;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem {
    private static final double kIntakePower = 1.0;
    private static final double kExhaustPower = -0.3;

    private final String mTitle;

    private final TalonFX mRoller;
    private final Solenoid mDeploySolenoid;

    private final PeriodicIO mPeriodicIO = new PeriodicIO();

    private WantedState mWantedState = WantedState.IDLE;
    private SystemState mSystemState = SystemState.IDLE;

    private boolean mIsDeployed = false;


    private static Intake mLeftInstance = null;
    private static Intake mRightInstance = null;

    public static Intake getLeftInstance() {
        if (mLeftInstance == null) {
            mLeftInstance = new Intake(
                    "Left",
                    Constants.kLeftIntakeSolenoidId,
                    Constants.kLeftIntakeMotorId,
                    Constants.kLeftIntakeInvertRoller);
        }
        return mLeftInstance;
    }

    public static Intake getRightInstance() {
        if (mRightInstance == null) {
            mRightInstance = new Intake(
                    "Right",
                    Constants.kRightIntakeSolenoidId,
                    Constants.kRightIntakeMotorId,
                    Constants.kRightIntakeInvertRoller);
        }
        return mRightInstance;
    }

    public enum WantedState {
        IDLE, INTAKING, EXHAUSTING
    }

    public enum SystemState {
        IDLE, INTAKING, EXHAUSTING
    }

    public static class PeriodicIO {
        public double demand = 0.0;
    }

    private Intake(String title, int deployId, CanDeviceId rollerId, boolean invertRoller) {
        mTitle = title;
        mDeploySolenoid = new LazySolenoid(PneumaticsModuleType.REVPH, deployId);
        mRoller = TalonFXFactory.createDefaultTalon(rollerId);
        mRoller.setInverted(invertRoller);

        mRoller.configOpenloopRamp(0.25);
        mRoller.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 50, 0.2));

        mRoller.setNeutralMode(NeutralMode.Coast);
    }

    public synchronized void deploy() {
        if (!mIsDeployed) {
            mDeploySolenoid.set(true);
            mIsDeployed = true;
        }
    }

    public synchronized void stow() {
        if (mIsDeployed) {
            mDeploySolenoid.set(false);
            mIsDeployed = false;
        }
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mRoller.set(ControlMode.PercentOutput, mPeriodicIO.demand);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Intake.this) {
                    mSystemState = SystemState.IDLE;
                    mRoller.setNeutralMode(NeutralMode.Coast);
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Intake.this) {
                    SystemState newState = handleAll(timestamp);
                    if (newState != mSystemState) {
                        mSystemState = newState;
                    }

                    switch (mSystemState) {
                        case IDLE:
                            mPeriodicIO.demand = 0.0;
                            break;
                        case INTAKING:
                            mPeriodicIO.demand = kIntakePower;
                            break;
                        case EXHAUSTING:
                            mPeriodicIO.demand = kExhaustPower;
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                synchronized (Intake.this) {
                    mSystemState = SystemState.IDLE;
                    mRoller.setNeutralMode(NeutralMode.Brake);
                }
            }
        });
    }

    private SystemState handleAll(double timestamp) {
        switch (mWantedState) {
            case EXHAUSTING:
                return SystemState.EXHAUSTING;
            case INTAKING:
                return SystemState.INTAKING;
            case IDLE:
            default:
                return SystemState.IDLE;
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString(mTitle + " Intake System State", mSystemState.toString());
    }
}
