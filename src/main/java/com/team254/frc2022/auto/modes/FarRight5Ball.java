package com.team254.frc2022.auto.modes;

import com.team254.frc2022.auto.AutoModeEndedException;
import com.team254.frc2022.auto.actions.*;
import com.team254.frc2022.paths.TrajectoryGenerator;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import java.util.Optional;

public class FarRight5Ball extends AutoModeBase {
    public FarRight5Ball() {
        mStartPose = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new HintTurretAction(Optional.of(Rotation2d.fromDegrees(90))));

        runAction(new OrientModulesAction(0.75, Rotation2d.fromDegrees(0)));

        // Drive to first floor ball (Ball 2)
        runAction(
                new ParallelAction(
                        new SeriesAction(
                                new DeployIntakeAction(false),
                                new WaitAction(0.75),
                                new StowIntakeAction(false)
                        ),
                        new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().farRightStartToFarRightBallHalf, true)
                )
        );

        // Gather and shoot ball 1+2
        runAction(
                new ParallelAction(
                        new RunIntakeAction(true),
                        new ShootAction(1.5)
                )
        );

        runAction(new HintTurretAction(Optional.empty()));

        // Drive small distance to second floor ball (ball 3)
        runAction(
                new ParallelAction(
                        new RunIntakeAction(true),
                        new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().farRightBallHalfToCloseRightBall)
                )
        );

        // Shoot ball 3
        runAction(
                new ParallelAction(
                        new StowIntakeAction(true),
                        new ShootAction(0.85)
                )
        );

        runAction(
                new ParallelAction(
                        new SuperstructureIdleAction(),
                        new RunIntakeAction(true),
                        new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().closeRightBallToHP)
                )
        );
        runAction(
                new ParallelAction(
                        new RunIntakeAction(true),
                        new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().HPToFeed)
                )
        );

        // WaitForNumBannerSensorsAction for 2 balls
        runAction(new WaitForNumBannerSensorsAction(2,0.75));
        runAction(
                new ParallelAction(
                        new SeriesAction(
                            new StowIntakeAction(true, false),
                            new WaitAction(1.0),
                            new RunIntakeAction(false)
                        ),
                        new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().feedToFarRebound),
                        new HintTurretAction(Optional.of(Rotation2d.fromDegrees(15)))
                )
        );
        runAction(new ShootAction(2.0, false));
    }
}
