package com.team254.frc2022.controlboard;

import com.team254.frc2022.Constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

public class ControlBoard implements IDriveControlBoard, IButtonControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final IDriveControlBoard mDriveControlBoard;
    private final IButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        boolean useDriveGamepad = Constants.kForceDriveGamepad ||
                DriverStation.getJoystickIsXbox(Constants.kDriveGamepadPort);
        mDriveControlBoard = useDriveGamepad ? GamepadDriveControlBoard.getInstance()
                : MainDriveControlBoard.getInstance();
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getStrafe() {
        return mDriveControlBoard.getStrafe();
    }

    @Override
    public double getRotation() {
        return mDriveControlBoard.getRotation();
    }

    @Override
    public boolean resetGyro() {
        return mDriveControlBoard.resetGyro();
    }

    public boolean getLeftEject() {
        return mButtonControlBoard.getLeftEject();
    }

    @Override
    public boolean getRightEject() {
        return mButtonControlBoard.getRightEject();
    }

    @Override
    public boolean getLeftIntake() {
        return mButtonControlBoard.getLeftIntake();
    }

    @Override
    public boolean getRightIntake() {
        return mButtonControlBoard.getRightIntake();
    }

    @Override
    public boolean getExhaust() {
        return mButtonControlBoard.getExhaust();
    }

    @Override
    public boolean getClearGoalTracker() {
        return mButtonControlBoard.getClearGoalTracker();
    }

    @Override
    public boolean getShoot() {
        return mButtonControlBoard.getShoot() || mDriveControlBoard.getShoot();
    }

    @Override
    public CardinalDirection getTurretHint() {
        return mButtonControlBoard.getTurretHint();
    }

    @Override
    public double getDPad() { return mDriveControlBoard.getDPad(); }

    @Override
    public boolean getSnapToOrigin() { return mDriveControlBoard.getSnapToOrigin(); }

    @Override
    public boolean togglePolarDrive() { return mDriveControlBoard.togglePolarDrive(); }

    @Override
    public boolean getLogShooterValues() {
        return mButtonControlBoard.getLogShooterValues();
    }

    @Override
    public boolean getToggleClimbMode() {
        return mButtonControlBoard.getToggleClimbMode();
    }

    @Override
    public boolean getClimbExtend() {
        return mButtonControlBoard.getClimbExtend();
    }

    @Override
    public boolean getClimbPullL4() {
        return mButtonControlBoard.getClimbPullL4();
    }

    @Override
    public boolean getClimbPullL2() {
        return mButtonControlBoard.getClimbPullL2();
    }

    @Override
    public double getClimbManual() {
        return mButtonControlBoard.getClimbManual();
    }

    @Override
    public boolean abortClimb() {
        return mButtonControlBoard.abortClimb();
    }

    @Override
    public boolean getTestButton() {
        return mDriveControlBoard.getTestButton();
    }

    @Override
    public boolean getXWheels(){
        return mDriveControlBoard.getXWheels();
    }

    @Override
    public boolean getManualStinger() {
        return mButtonControlBoard.getManualStinger();
    }

    @Override
    public boolean getSmoothMode() {
        return mDriveControlBoard.getSmoothMode();
    }

    @Override
    public void setRumble(GenericHID.RumbleType type, double value) {
        mButtonControlBoard.setRumble(type, value);
    }
}