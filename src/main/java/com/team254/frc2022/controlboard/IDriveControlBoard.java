package com.team254.frc2022.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getStrafe();

    double getRotation();

    boolean resetGyro();

    double getDPad();

    boolean getSnapToOrigin();

    boolean togglePolarDrive();

    boolean getShoot();

    boolean getXWheels();

    boolean getSmoothMode();

    boolean getTestButton();
}