package com.team254.frc2022.auto.actions;

import com.team254.frc2022.subsystems.Superstructure;

public class SuperstructureIdleAction implements Action {
    @Override
    public void start() {
        Superstructure.getInstance().setWantedState(Superstructure.WantedState.IDLE);
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
