package com.team254.frc2022.controlboard;


import edu.wpi.first.wpilibj.GenericHID;

public interface IButtonControlBoard {
    boolean getLeftEject();

    boolean getRightEject();

    boolean getLeftIntake();

    boolean getRightIntake();

    boolean getExhaust();

    boolean getShoot();

    boolean getClearGoalTracker();

    CardinalDirection getTurretHint();

    boolean getLogShooterValues();

    boolean getToggleClimbMode();

    boolean getClimbExtend();

    boolean getClimbPullL4();

    boolean getClimbPullL2();

    double getClimbManual();

    boolean abortClimb();

    boolean getManualStinger();

    void setRumble(GenericHID.RumbleType type, double on);
}
