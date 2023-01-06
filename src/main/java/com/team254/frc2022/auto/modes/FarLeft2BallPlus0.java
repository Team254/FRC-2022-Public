package com.team254.frc2022.auto.modes;

import com.team254.frc2022.auto.AutoModeEndedException;
import com.team254.frc2022.auto.actions.*;
import com.team254.frc2022.paths.TrajectoryGenerator;
import com.team254.frc2022.subsystems.Intake;
import com.team254.frc2022.subsystems.Serializer;
import com.team254.frc2022.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import java.util.Optional;

public class FarLeft2BallPlus0 extends AutoModeBase {
    public FarLeft2BallPlus0() {
        mStartPose = new Pose2d(0, 0, Rotation2d.fromDegrees(-40));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        Serializer.getInstance().setForceNoEject(true);
        Serializer.getInstance().setExhaustMode(Serializer.ExhuastMode.RIGHT);
        runAction(new OrientModulesAction(0.75, Rotation2d.fromDegrees(0)));

        // Drive backwards off the line
        runAction(
                new ParallelAction(
                        new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().farLeftStartToFarLeftStaging, true),
                        new HintTurretAction(Optional.of(Rotation2d.fromDegrees(-40)))
                )
        );

        // Grab first good ball and shoot it and preload
        runAction(
                new ParallelAction(
                        new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().farLeftStagingToFarLeftBall1),
                        new RunIntakeAction(false)
                )
        );
        runAction(new ShootAction(3.0));

        // Drive to eject spot (but don't eject anything)
        runAction(
                new ParallelAction(
                        new StowIntakeAction(false),
                        new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().farLeftBall1ToFarLeftEject)
                )
        );

        runAction(
                new ParallelAction(
                        new RunIntakeAction(true),
                        new ShootAction(5.0, false),
                        new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().farLeftEjectToFarLeftRebound)
                )
        );
    }
}
