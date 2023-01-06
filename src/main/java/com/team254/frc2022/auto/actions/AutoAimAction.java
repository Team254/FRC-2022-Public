package com.team254.frc2022.auto.actions;

import com.team254.frc2022.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

import java.util.Optional;

public class AutoAimAction implements Action {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private Optional<Rotation2d> mHint = Optional.empty();

    public AutoAimAction() {}

    public AutoAimAction(Rotation2d hint) {
        mHint = Optional.of(hint);
    }

    @Override
    public void start() {
        if (mHint.isPresent()) {
            mSuperstructure.setFieldRelativeTurretHint(mHint.get());
        } else {
            mSuperstructure.resetFieldRelativeTurretHint();
        }
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }
}

