package com.team254.lib.trajectory.timing;

import com.team254.lib.geometry.ICurvature;
import com.team254.lib.geometry.IPose2d;
import com.team254.lib.physics.SwerveDrive;
import com.team254.frc2022.Constants;

public class SwerveDriveDynamicsConstraint<S extends IPose2d<S> & ICurvature<S>> implements TimingConstraint<S> {

    protected final SwerveDrive drive_;
    protected final double abs_voltage_limit_;

    public SwerveDriveDynamicsConstraint(final SwerveDrive drive, double abs_voltage_limit) {
        drive_ = drive;
        abs_voltage_limit_ = abs_voltage_limit;
    }

    @Override
    public double getMaxVelocity(S state) {
        return Constants.kMaxVelocityMetersPerSecond / (1 + Math.abs(4.0*state.getCurvature()));// from 1323 TODO verify or fix
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(S state,
                                                    double velocity) {
        return new MinMaxAcceleration(-Constants.kMaxDriveAcceleration, Constants.kMaxDriveAcceleration);
    }
}