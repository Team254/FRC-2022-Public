package com.team254.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import java.util.ArrayList;

public class TalonFXChecker extends MotorChecker<BaseTalon> {
    private static class StoredTalonFXConfiguration {
        public ControlMode mMode;
        public double mSetValue;
    }

    protected ArrayList<TalonFXChecker.StoredTalonFXConfiguration> mStoredConfigurations = new ArrayList<>();

    public static boolean checkMotors(Subsystem subsystem,
                                      ArrayList<MotorChecker.MotorConfig<BaseTalon>> motorsToCheck,
                                      MotorChecker.CheckerConfig checkerConfig) {
        TalonFXChecker checker = new TalonFXChecker();
        return checker.checkMotorsImpl(subsystem, motorsToCheck, checkerConfig);
    }

    @Override
    protected void storeConfiguration() {
        // record previous configuration for all talons
        for (MotorChecker.MotorConfig<BaseTalon> config : mMotorsToCheck) {
            LazyTalonFX talon = (LazyTalonFX) config.mMotor;

            TalonFXChecker.StoredTalonFXConfiguration configuration = new TalonFXChecker.StoredTalonFXConfiguration();
            configuration.mMode = talon.getControlMode();
            configuration.mSetValue = talon.getLastSet();

            mStoredConfigurations.add(configuration);
        }
    }

    @Override
    protected void restoreConfiguration() {
        for (int i = 0; i < mMotorsToCheck.size(); ++i) {
            mMotorsToCheck.get(i).mMotor.set(mStoredConfigurations.get(i).mMode,
                    mStoredConfigurations.get(i).mSetValue);
        }
    }

    @Override
    protected void setMotorOutput(BaseTalon motor, double output) {
        motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    protected double getMotorCurrent(BaseTalon motor) {
        return motor.getSupplyCurrent();
    }
}
