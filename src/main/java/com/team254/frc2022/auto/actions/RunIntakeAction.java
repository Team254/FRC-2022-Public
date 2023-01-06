package com.team254.frc2022.auto.actions;

import com.team254.frc2022.subsystems.Intake;
import com.team254.frc2022.subsystems.Serializer;

public class RunIntakeAction implements Action {
    private Serializer mSerializer = Serializer.getInstance();
    private Intake mIntake;
    private final boolean want_left;

    public RunIntakeAction(boolean want_left) {
        this.want_left = want_left;
        mIntake = want_left ? Intake.getLeftInstance() : Intake.getRightInstance();
    }

    @Override
    public void start() {
        mIntake.deploy();
        mSerializer.setWantedState(want_left ? Serializer.WantedState.INTAKING_LEFT : Serializer.WantedState.INTAKING_RIGHT);
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
