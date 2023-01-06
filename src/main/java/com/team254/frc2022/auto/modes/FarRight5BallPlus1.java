package com.team254.frc2022.auto.modes;

import com.team254.frc2022.auto.AutoModeEndedException;
import com.team254.frc2022.auto.actions.*;
import com.team254.frc2022.paths.TrajectoryGenerator;
import com.team254.frc2022.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import java.util.Optional;

public class FarRight5BallPlus1 extends AutoModeBase {
    public FarRight5BallPlus1() {
        mStartPose = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new HintTurretAction(Optional.of(Rotation2d.fromDegrees(90))));

        runAction(new OrientModulesAction(0.75, Rotation2d.fromDegrees(0)));

        Superstructure.getInstance().setAimingGoal(Rotation2d.fromDegrees(160.0));
        Superstructure.getInstance().setEjectConstraints(6, 5.5);
        runAction(
                new ParallelAction(
                        new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().farRightStartToFarRightBallEject, true),
                        new SeriesAction(
                                new WaitAction(1.0),
                                new RunIntakeAction(false),
                                new WaitAction(0.75),
                                new StowIntakeAction(false),
                                new WaitAction(1.0)
                        )
                )
        );

        runAction(new HintTurretAction(Optional.empty()));

        runAction(
                new ParallelAction(
                        new SeriesAction(
                            new RunIntakeAction(true),
                            new WaitAction(1.2),
                            new StowIntakeAction(true, true, false)
                        ),
                        new SeriesAction(
                                new WaitAction(0.05),
                                new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().farRightBallEjectToCloseRightBall)
                        ),
                        new SeriesAction(
                                new WaitAction(2.5),
                                new ParallelAction(
                                    new ShootAction(1.75),
                                    new SeriesAction(
                                            new WaitAction(0.4),
                                            new RunIntakeAction(true),
                                            new WaitAction(0.4),
                                            new StowIntakeAction(true, false, true),
                                            new WaitAction(0.5)
                                    )
                                )
                        )
                )
        );

        runAction(
                new ParallelAction(
                        new SuperstructureIdleAction(),
                        new RunIntakeAction(true),
                        new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().closeRightBallToHPRebound)
                )
        );
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().HPToFeedRebound));

        // WaitForNumBannerSensorsAction for 2 balls
        runAction(new WaitForNumBannerSensorsAction(2,0.8));

        // drive to rebound point and leave intake out
        runAction(
                new ParallelAction(
                        new SeriesAction(
                                new StowIntakeAction(true, false), // dislodge ball if it gets stuck in intake
                                new WaitAction(1.0),
                                new RunIntakeAction(false)
                        ),
                        new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().feedToCloseRebound),
                        new HintTurretAction(Optional.of(Rotation2d.fromDegrees(20)))   // overshoot a bit to make sure turret hits goal
                )
        );

        runAction(new ShootAction(5.0, false));
    }

}
