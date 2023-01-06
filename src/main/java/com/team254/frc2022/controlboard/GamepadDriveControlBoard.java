package com.team254.frc2022.controlboard;

import com.team254.frc2022.Constants;
import com.team254.frc2022.controlboard.XboxController.Button;
import com.team254.lib.geometry.Rotation2d;

public class GamepadDriveControlBoard implements IDriveControlBoard {
    private static GamepadDriveControlBoard mInstance = null;

    public static GamepadDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadDriveControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadDriveControlBoard() {
        mController = new XboxController(Constants.kDriveGamepadPort);
    }

    @Override
    public double getThrottle() {
        return mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
    }

    @Override
    public double getStrafe() {
        return -mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.X);
    }

    @Override
    public double getRotation() {
        return -mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
    }

    @Override
    public boolean resetGyro() {
        return mController.getButton(XboxController.Button.BACK);
    }

    @Override
    public double getDPad() {
        if (mController.getDPad() == -1) {
            return -1;
        }

        double degs = CardinalDirection.findClosest(Rotation2d.fromDegrees(-mController.getDPad() + 90)).rotation.getDegrees();
        if (degs < 0) {
            degs += 360;
        }
        return degs;
    }

    @Override
    public boolean getSnapToOrigin() { return mController.getButton(XboxController.Button.Y); }

    @Override
    public boolean togglePolarDrive() { return mController.getButton(XboxController.Button.X); }

    public boolean getShoot() {
        return false;
    }

    @Override
    public boolean getTestButton() { return mController.getButton(XboxController.Button.A); }

    @Override
    public boolean getXWheels() { return false; }

    @Override
    public boolean getSmoothMode() { return mController.getButton(XboxController.Button.LB); }
}