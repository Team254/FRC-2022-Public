package com.team254.frc2022.subsystems;

import com.team254.frc2022.Constants;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Infrastructure extends Subsystem {
    private final PneumaticHub mPneumaticHub;
    private final PowerDistribution mPowerDistributionHub;

    private static Infrastructure mInstance = null;

    public static Infrastructure getInstance() {
        if (mInstance == null) {
            mInstance = new Infrastructure();
        }
        return mInstance;
    }

    private Infrastructure() {
        mPneumaticHub = new PneumaticHub();
        mPowerDistributionHub = new PowerDistribution();
    }

    public void enableCompressorAnalog() {
        mPneumaticHub.enableCompressorAnalog(90.0, 110.0);
    }

    public void disableCompressor() {
        mPneumaticHub.disableCompressor();
    }

    public double getVoltage() {
        return mPowerDistributionHub.getVoltage();
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                enableCompressorAnalog();
            }

            @Override
            public void onLoop(double timestamp) {}

            @Override
            public void onStop(double timestamp) {
                disableCompressor();
                stop();
            }
        });
    }

    @Override
    public void stop() {
//        mPneumaticHub.disableCompressor();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Pressure", mPneumaticHub.getPressure(Constants.kAnalogSensorChannel));
        SmartDashboard.putBoolean("Compressor On", mPneumaticHub.getCompressor());
        SmartDashboard.putNumber("Battery Voltage", mPowerDistributionHub.getVoltage());
    }
}
