package com.team254.frc2022.controlboard;

import com.team254.frc2022.Constants;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.DelayedBoolean;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private static GamepadButtonControlBoard mInstance = null;

    private final double kDPadDelay = 0.02;
    private final DelayedBoolean mDPadValid = new DelayedBoolean(Timer.getFPGATimestamp(), kDPadDelay);
    private CardinalDirection mLastCardinal;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadButtonControlBoard() {
        mController = new XboxController(Constants.kOperatorControllerPort);
    }

    @Override
    public boolean getLeftEject() {
        return mController.getButton(XboxController.Button.LB);
    }

    @Override
    public boolean getRightEject() {
        return mController.getButton(XboxController.Button.RB);
    }

    @Override
    public boolean getLeftIntake() {
        return mController.getTrigger(XboxController.Side.LEFT);
    }

    @Override
    public boolean getRightIntake() {
        return mController.getTrigger(XboxController.Side.RIGHT);
    }

    @Override
    public boolean getClearGoalTracker() {
        return mController.getButton(XboxController.Button.B);
    }

    @Override
    public boolean getExhaust() {
        return mController.getButton(XboxController.Button.Y);
    }

    @Override
    public boolean getShoot() {
        return mController.getButton(XboxController.Button.X);
    }

    @Override
    public CardinalDirection getTurretHint() {
        int dPad = mController.getDPad();
        CardinalDirection newCardinal = dPad == -1 ? CardinalDirection.NONE : CardinalDirection.findClosest(Rotation2d.fromDegrees(dPad));
        if (newCardinal != CardinalDirection.NONE && CardinalDirection.isDiagonal(newCardinal)) {
            // Latch previous direction on diagonal presses, because the D-pad sucks at diagonals.
            newCardinal = mLastCardinal;
        }
        boolean valid = mDPadValid.update(Timer.getFPGATimestamp(), newCardinal != CardinalDirection.NONE && (mLastCardinal == CardinalDirection.NONE || newCardinal == mLastCardinal));
        if (valid) {
            if (mLastCardinal == CardinalDirection.NONE) {
                mLastCardinal = newCardinal;
            }
            return mLastCardinal;
        } else {
            mLastCardinal = newCardinal;
        }
        return CardinalDirection.NONE;
    }

    @Override 
    public boolean getLogShooterValues() {
        return mController.getButton(XboxController.Button.BACK);
    }

    @Override
    public boolean getToggleClimbMode() {
        return mController.getButton(XboxController.Button.START) && mController.getButton(XboxController.Button.BACK);
    }

    @Override
    public boolean getClimbExtend() {
        return mController.getButton(XboxController.Button.Y);
    }

    @Override
    public boolean getClimbPullL4() {
        return mController.getButton(XboxController.Button.A);
    }

    @Override
    public boolean abortClimb() {
        return mController.getButton(XboxController.Button.LB) && mController.getButton(XboxController.Button.RB);
    }

    @Override
    public boolean getClimbPullL2() {
        return mController.getButton(XboxController.Button.B);
    }

    @Override
    public double getClimbManual() {
        return Util.handleDeadband(mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y), Constants.kJoystickThreshold);
    }

    @Override
    public boolean getManualStinger() {
        return mController.getTrigger(XboxController.Side.RIGHT) && mController.getTrigger(XboxController.Side.LEFT);
    }

    @Override
    public void setRumble(GenericHID.RumbleType type, double value) {
        mController.setRumble(type, value);
    }
}
