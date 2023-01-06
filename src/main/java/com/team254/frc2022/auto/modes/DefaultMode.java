package com.team254.frc2022.auto.modes;

import com.team254.frc2022.auto.AutoModeEndedException;
import com.team254.frc2022.auto.actions.*;
import com.team254.frc2022.paths.TrajectoryGenerator;
import com.team254.lib.trajectory.Trajectory;

public class DefaultMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        // outdated (diagonal ball approach angle isn't very good)
//        runAction(
//                new ParallelAction(
//                        new DeployIntakeAction(false, true),
//                        new DeployIntakeAction(true, true),
//                        new RunIntakeAction(false),
//                        new RunIntakeAction(true),
//                new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().startToRightBall));
//                ));
//        runAction(
//                new ParallelAction(
//                        new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().rightBallToLeftBall),
//                        new DeployIntakeAction(false, false)));
//        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().leftBallToHP));
//        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().HPToCloseShoot));

        runAction(new RunIntakeAction(false));
        runAction(new WaitAction(1));
        runAction(new StowIntakeAction(false));

        runAction(new WaitAction(1));

        runAction(new RunIntakeAction(true));
        runAction(new WaitAction(1));
        runAction(new StowIntakeAction(true));

        runAction(new ShootAction(1));
    }
}
