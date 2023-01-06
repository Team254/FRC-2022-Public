package com.team254.frc2022.auto.actions;

import com.team254.frc2022.subsystems.Intake;
import com.team254.frc2022.subsystems.Serializer;

public class StowIntakeAction implements Action {
    private final Intake mIntake;
    private final Serializer mSerializer;
    private final boolean mResetIdle;
    private boolean mWantStow = true;

    public StowIntakeAction(boolean want_left) {
        this(want_left, true);
    }

    public StowIntakeAction(boolean want_left, boolean reset_idle) {
        mIntake = want_left ? Intake.getLeftInstance() : Intake.getRightInstance();
        mSerializer = Serializer.getInstance();
        mResetIdle = reset_idle;
        mWantStow = true;
    }

    public StowIntakeAction(boolean want_left, boolean reset_idle, boolean want_stow) {
        mIntake = want_left ? Intake.getLeftInstance() : Intake.getRightInstance();
        mSerializer = Serializer.getInstance();
        mResetIdle = reset_idle;
        mWantStow = want_stow;
    }

    @Override
    public void start() {
        if (mWantStow) {
            mIntake.stow();
        }
        if (mResetIdle) {
            mSerializer.setWantedState(Serializer.WantedState.IDLE);
        }
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }
}
