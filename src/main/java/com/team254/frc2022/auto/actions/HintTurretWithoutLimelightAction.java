package com.team254.frc2022.auto.actions;

import com.team254.frc2022.subsystems.Limelight;
import com.team254.frc2022.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

public class HintTurretWithoutLimelightAction implements Action {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private Optional<Rotation2d> mHint = Optional.empty();
    private boolean disable_ll_while_rot;

    public HintTurretWithoutLimelightAction() {}

    public HintTurretWithoutLimelightAction(Rotation2d hint) {
        mHint = Optional.of(hint);
        disable_ll_while_rot = false;
    }

    public HintTurretWithoutLimelightAction(Rotation2d hint, boolean disable_ll) {
        mHint = Optional.of(hint);
        disable_ll_while_rot = disable_ll;
    }

    @Override
    public void start() {
        if (mHint.isPresent()) {
            mSuperstructure.setFieldRelativeTurretHint(mHint.get());
        }
        if (disable_ll_while_rot) {
            Limelight.getInstance().setLed(Limelight.LedMode.OFF);
        }
    }


    private Rotation2d kAcceptableHintError = Rotation2d.fromDegrees(10);
    @Override
    public void update() {
        if (mSuperstructure.withinHint(Timer.getFPGATimestamp(), kAcceptableHintError)) {
            Limelight.getInstance().setLed(Limelight.LedMode.ON);
        }
    }

    @Override
    public boolean isFinished() {
        return mSuperstructure.hasTarget();
    }

    @Override
    public void done() {

    }
}

