package com.team254.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.team254.frc2022.Constants;
import com.team254.lib.drivers.ServoMotorSubsystem;
import com.team254.lib.drivers.TalonUtil;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends ServoMotorSubsystem {
    private final Serializer mSerializer = Serializer.getInstance();

    private static Turret mInstance;

    public synchronized static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret(Constants.kTurretConstants);
        }

        return mInstance;
    }

    private Turret(final ServoMotorSubsystemConstants constants) {
        super(constants);
        TalonUtil.checkError(
                mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen),
                mConstants.kName + ": Could not set forward limit switch: ");

        TalonUtil.checkError(
                mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen),
                mConstants.kName + ": Could not set reverse limit switch: ");

        mMaster.overrideLimitSwitchesEnable(true);
        mMaster.overrideSoftLimitsEnable(true);
        zeroSensors(); //TODO Better Zeroing
    }

    @Override
    public boolean atHomingLocation() {
        return mSerializer.getLimitSwitchTripped();
    }


    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Turret Reverse Limit Switch", mMaster.getSensorCollection().isRevLimitSwitchClosed() == 1);
        SmartDashboard.putBoolean("Turret Forward Limit Switch", mMaster.getSensorCollection().isFwdLimitSwitchClosed() == 1);
        SmartDashboard.putBoolean("Turret Homed", atHomingLocation());
        SmartDashboard.putNumber("Turret Angle", getAngle());
        SmartDashboard.putNumber("Turret Error", getSetpoint() - getAngle());
        super.outputTelemetry();
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

}