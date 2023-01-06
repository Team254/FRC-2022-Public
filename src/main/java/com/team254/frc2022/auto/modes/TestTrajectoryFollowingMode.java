package com.team254.frc2022.auto.modes;

import com.team254.frc2022.auto.AutoModeEndedException;
import com.team254.frc2022.auto.actions.DriveTrajectoryAction;
import com.team254.frc2022.auto.actions.OrientModulesAction;
import com.team254.frc2022.auto.actions.RunIntakeAction;
import com.team254.frc2022.auto.actions.WaitAction;
import com.team254.frc2022.paths.TrajectoryGenerator;
import com.team254.frc2022.subsystems.Drive;
import com.team254.lib.geometry.Rotation2d;

public class TestTrajectoryFollowingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new OrientModulesAction(1,Rotation2d.fromDegrees(0)));
//        runAction(new WaitAction(3));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().testTrajectory, true));
//        runAction(new RunIntakeAction(true));
    }
}