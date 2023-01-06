package com.team254.frc2022.auto.actions;

import com.team254.frc2022.subsystems.Intake;

public class DeployIntakeAction implements Action {
    private final Intake mIntake;

    public DeployIntakeAction(boolean want_left) {
        mIntake = want_left ? Intake.getLeftInstance() : Intake.getRightInstance();
    }

    @Override
    public void start() {
        mIntake.deploy();
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
