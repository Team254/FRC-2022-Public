package com.team254.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2022.Constants;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    private static Shooter mInstance;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }

        return mInstance;
    }

    private final TalonFX mRightMaster, mLeftMaster;

    public static class PeriodicIO {
        // inputs
        public double timestamp;
        public double left_output_voltage;
        public double left_supply_current;
        public double left_stator_current;
        public double right_output_voltage;
        public double right_supply_current;
        public double right_stator_current;
        public double right_velocity_ticks_per_100_ms = 0.0;
        public double left_velocity_ticks_per_100_ms = 0.0;
        public double right_velocity_rpm = 0.0;
        public double left_velocity_rpm = 0.0;

        // outputs
        public double demand = 0.0;
    }

    public enum ShooterControlState {
        OPEN_LOOP, VELOCITY
    }

    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private ShooterControlState mControlState = ShooterControlState.OPEN_LOOP;

    private Shooter() {
        mRightMaster = TalonFXFactory.createDefaultTalon(Constants.kShooterRightId);
        mLeftMaster = TalonFXFactory.createDefaultTalon(Constants.kShooterLeftId);
        resetSettings();

        // initialize encoders and set status frames
        TalonUtil.checkError(mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
                Constants.kLongCANTimeoutMs), "Shooter Right Master: Could not detect encoder: ");
        TalonUtil.checkError(mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
                Constants.kLongCANTimeoutMs), "Shooter Left Master: Could not detect encoder: ");

        // TODO velocity measurement windows/periods?

        // set right master gains
        TalonUtil.checkError(mRightMaster.config_kP(0, Constants.kShooterKp, Constants.kLongCANTimeoutMs),
                "Shooter Right Master: could not set kP: ");
        TalonUtil.checkError(mRightMaster.config_kI(0, Constants.kShooterKi, Constants.kLongCANTimeoutMs),
                "Shooter Right Master: could not set kI: ");
        TalonUtil.checkError(mRightMaster.config_kD(0, Constants.kShooterKd, Constants.kLongCANTimeoutMs),
                "Shooter Right Master: could not set kD: ");
        TalonUtil.checkError(mRightMaster.config_kF(0, Constants.kShooterKf, Constants.kLongCANTimeoutMs),
                "Shooter Right Master: Could not set kF: ");
        TalonUtil.checkError(mRightMaster.config_IntegralZone(0, rpmToNativeUnits(Constants.kShooterIZoneRPM), Constants.kLongCANTimeoutMs),
                "Shooter Right Master: could not set izone: ");

        // set left master gains
        TalonUtil.checkError(mLeftMaster.config_kP(0, Constants.kShooterKp, Constants.kLongCANTimeoutMs),
                "Shooter Left Master: could not set kP: ");
        TalonUtil.checkError(mLeftMaster.config_kI(0, Constants.kShooterKi, Constants.kLongCANTimeoutMs),
                "Shooter Left Master: could not set kI: ");
        TalonUtil.checkError(mLeftMaster.config_kD(0, Constants.kShooterKd, Constants.kLongCANTimeoutMs),
                "Shooter Left Master: could not set kD: ");
        TalonUtil.checkError(mLeftMaster.config_kF(0, Constants.kShooterKf, Constants.kLongCANTimeoutMs),
                "Shooter Left Master: Could not set kF: ");
        TalonUtil.checkError(mLeftMaster.config_IntegralZone(0, rpmToNativeUnits(Constants.kShooterIZoneRPM), Constants.kLongCANTimeoutMs),
                "Shooter Left Master: could not set izone: ");
    }

    public void resetSettings() {
        mRightMaster.setInverted(false);

        // this makes us less sensitive to battery voltage, hopefully
        mRightMaster.configPeakOutputForward(0.8);
        mLeftMaster.configPeakOutputForward(0.8);
        mRightMaster.configPeakOutputReverse(0.8);
        mLeftMaster.configPeakOutputReverse(0.8);

        mRightMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mRightMaster.enableVoltageCompensation(true);
        mLeftMaster.setInverted(true);
        mLeftMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mLeftMaster.enableVoltageCompensation(true);
        mRightMaster.setNeutralMode(NeutralMode.Brake);
        mLeftMaster.setNeutralMode(NeutralMode.Brake);
        mRightMaster.configPeakOutputReverse(-1);
        mLeftMaster.configPeakOutputReverse(-1);
        mRightMaster.configPeakOutputForward(1);
        mLeftMaster.configPeakOutputForward(1);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // NOTE: Uncomment this to debug shooter
                // startLogging();
            }

            @Override
            public void onLoop(double timestamp) {}

            @Override
            public void onStop(double timestamp) {
                // NOTE: Uncomment this to debug shooter
                // stopLogging();
                stop();
            }
        });
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.left_output_voltage = mLeftMaster.getMotorOutputVoltage();
        mPeriodicIO.left_supply_current = mLeftMaster.getSupplyCurrent();
        mPeriodicIO.left_stator_current = mLeftMaster.getStatorCurrent();

        mPeriodicIO.right_output_voltage = mRightMaster.getMotorOutputVoltage();
        mPeriodicIO.right_supply_current = mRightMaster.getSupplyCurrent();
        mPeriodicIO.right_stator_current = mRightMaster.getStatorCurrent();

        mPeriodicIO.right_velocity_ticks_per_100_ms = mRightMaster.getSelectedSensorVelocity();
        mPeriodicIO.left_velocity_ticks_per_100_ms = mLeftMaster.getSelectedSensorVelocity();

        mPeriodicIO.right_velocity_rpm = nativeUnitsToRPM(mPeriodicIO.right_velocity_ticks_per_100_ms);
        mPeriodicIO.left_velocity_rpm = nativeUnitsToRPM(mPeriodicIO.left_velocity_ticks_per_100_ms);

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (mControlState == ShooterControlState.OPEN_LOOP) {
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
        } else if (mControlState == ShooterControlState.VELOCITY) {
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.demand);
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.demand);
        }
    }

    @Override
    public void stop() {
        mRightMaster.set(ControlMode.PercentOutput, 0.0);
        mLeftMaster.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public synchronized void setOpenLoop(double power) {
        mControlState = ShooterControlState.OPEN_LOOP;
        mPeriodicIO.demand = power;
    }

    public synchronized void setRPM(double rpm) {
        mControlState = ShooterControlState.VELOCITY;
        mPeriodicIO.demand = rpmToNativeUnits(rpm);
    }

    public synchronized boolean isAtSetpoint() {
        return getLeftRPM() / nativeUnitsToRPM(mPeriodicIO.demand) > Constants.kShooterAllowablePercentError &&
                getRightRPM() / nativeUnitsToRPM(mPeriodicIO.demand) > Constants.kShooterAllowablePercentError;
    }

    public synchronized double getRightRPM() {
        return nativeUnitsToRPM(getRightVelocityNativeUnits());
    }

    public synchronized double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100_ms;
    }

    public synchronized double getLeftRPM() {
        return nativeUnitsToRPM(getLeftVelocityNativeUnits());
    }

    public synchronized double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100_ms;
    }

    public synchronized double getAverageRPM() {
        return (getRightRPM() + getLeftRPM()) / 2.0;
    }

    public synchronized double getDemandRPM() {
        return nativeUnitsToRPM(mPeriodicIO.demand);
    }

    public synchronized double getAverageOutputVoltage() {
        return (mPeriodicIO.right_output_voltage + mPeriodicIO.left_output_voltage) / 2.0;
    }

    public synchronized double getAverageSupplyCurrent() {
        return (mPeriodicIO.right_supply_current + mPeriodicIO.left_supply_current) / 2.0;
    }

    public synchronized double getAverageStatorCurrent() {
        return (mPeriodicIO.right_stator_current + mPeriodicIO.left_stator_current) / 2.0;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/SHOOTER-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    /**
     * @param ticks_per_100_ms
     * @return rpm
     */
    public double nativeUnitsToRPM(double ticks_per_100_ms) {
        return ticks_per_100_ms * 10.0 * 60.0 / Constants.kShooterTicksPerRevolution;
    }

    /**
     * @param rpm
     * @return ticks per 100 ms
     */
    public double rpmToNativeUnits(double rpm) {
        return rpm / 60.0 / 10.0 * Constants.kShooterTicksPerRevolution;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Shooter Right Master RPM", getRightRPM());
        SmartDashboard.putNumber("Shooter Left Master RPM", getLeftRPM());
        SmartDashboard.putNumber("Shooter Left Master RPM Error", getLeftRPM() - nativeUnitsToRPM(mPeriodicIO.demand));
        SmartDashboard.putNumber("Shooter Demand", mControlState == ShooterControlState.OPEN_LOOP ? mPeriodicIO.demand
                : (mControlState == ShooterControlState.VELOCITY ? nativeUnitsToRPM(mPeriodicIO.demand) : 0.0));

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }
}