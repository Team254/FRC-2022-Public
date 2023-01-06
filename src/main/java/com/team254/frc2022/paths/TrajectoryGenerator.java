package com.team254.frc2022.paths;

import com.team254.frc2022.Constants;
import com.team254.frc2022.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryGenerator {
    // TODO tune
    private static final double kMaxVel = Units.metersToInches(Constants.kMaxVelocityMetersPerSecond * 0.9);
    private static final double kMaxAccel = 100.0;
    private static final double kMaxVoltage = 9.0;

    private static final TrajectoryGenerator mInstance = new TrajectoryGenerator();

    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, headings, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, headings, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    public class TrajectorySet {
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> testTrajectory;

        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> farRightStartToFarRightBallHalf;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> farRightBallHalfToCloseRightBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> closeRightBallToHP;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> HPToFeed;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> feedToFarRebound;

        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> farRightStartToFarRightBallEject;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> farRightBallEjectToCloseRightBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> closeRightBallToHPRebound;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> HPToFeedRebound;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> feedToCloseRebound;

        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> farLeftStartToFarLeftStaging;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> farLeftStagingToFarLeftBall1;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> farLeftBall1ToFarLeftBall2;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> farLeftBall2ToFarRightBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> farRightBallToFarLeftEject;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> farLeftEjectToFarLeftRebound;

        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> farLeftBall1ToFarLeftEject;
        public final Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> farLeftBall2ToFarLeftEject;

        private TrajectorySet() {
            testTrajectory = getTestTrajectory();

            farRightStartToFarRightBallHalf = getFarRightStartToFarRightBallHalf();
            farRightBallHalfToCloseRightBall = getFarRightBallHalfToCloseRightBall();
            closeRightBallToHP = getCloseRightBallToHP();
            HPToFeed = getHPToFeed();
            feedToFarRebound = getFeedToFarRebound();

            farRightStartToFarRightBallEject = getFarRightStartToFarRightBallEject();
            farRightBallEjectToCloseRightBall = getFarRightBallEjectToCloseRightBall();
            closeRightBallToHPRebound = getCloseRightBallToHPRebound();
            HPToFeedRebound = getHPToFeedRebound();
            feedToCloseRebound = getFeedToCloseRebound();

            farLeftStartToFarLeftStaging = getFarLeftStartToFarLeftStaging();
            farLeftStagingToFarLeftBall1 = getFarLeftStagingToFarLeftBall1();
            farLeftBall1ToFarLeftBall2 = getFarLeftBall1ToFarLeftBall2();
            farLeftBall2ToFarRightBall = getFarLeftBall2ToFarRightBall();
            farRightBallToFarLeftEject = getFarRightBallToFarLeftEject();
            farLeftEjectToFarLeftRebound = getFarLeftEjectToFarLeftRebound();

            farLeftBall1ToFarLeftEject = getFarLeftBall1ToFarLeftEject();
            farLeftBall2ToFarLeftEject = getFarLeftBall2ToFarLeftEject();
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getTestTrajectory() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(0));
            waypoints.add(new Pose2d(170, 0, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(90));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFarRightStartToFarRightBallHalf() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0, 0, Rotation2d.fromDegrees(270)));
            headings.add(Rotation2d.fromDegrees(90));
            waypoints.add(new Pose2d(0,   -50, Rotation2d.fromDegrees(270)));
            headings.add(Rotation2d.fromDegrees(90));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFarRightStartToFarRightBallEject() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0, 0, Rotation2d.fromDegrees(270)));
            headings.add(Rotation2d.fromDegrees(90));
            waypoints.add(new Pose2d(2,   -50, Rotation2d.fromDegrees(270)));
            headings.add(Rotation2d.fromDegrees(90));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFarRightBallHalfToCloseRightBall() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0, -50, Rotation2d.fromDegrees(100)));
            headings.add(Rotation2d.fromDegrees(90));
            waypoints.add(new Pose2d(-118,   -9, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(90));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFarRightBallEjectToCloseRightBall() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(2, -50, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(90));
            waypoints.add(new Pose2d(-15, -50, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(90));
            waypoints.add(new Pose2d(-123,   3, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(90));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getCloseRightBallToHP() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(-118,   -9, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(90));
            waypoints.add(new Pose2d(-257.5, -19.5, Rotation2d.fromDegrees(225)));
            headings.add(Rotation2d.fromDegrees(135));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getCloseRightBallToHPRebound() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(-123,   3, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(90));
            waypoints.add(new Pose2d(-260.5, -13.5, Rotation2d.fromDegrees(225)));
            headings.add(Rotation2d.fromDegrees(135));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getHPToFeed() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(-257.5, -19.5, Rotation2d.fromDegrees(45)));
            headings.add(Rotation2d.fromDegrees(135));
            waypoints.add(new Pose2d(-248.5, -10.5, Rotation2d.fromDegrees(45)));
            headings.add(Rotation2d.fromDegrees(135));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getHPToFeedRebound() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(-260.5, -13.5, Rotation2d.fromDegrees(45)));
            headings.add(Rotation2d.fromDegrees(135));
            waypoints.add(new Pose2d(-253.5, -4.5, Rotation2d.fromDegrees(45)));
            headings.add(Rotation2d.fromDegrees(135));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel/2.0, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFeedToFarRebound() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(-248.5, -10.5, Rotation2d.fromDegrees(-10)));
            headings.add(Rotation2d.fromDegrees(135));
            waypoints.add(new Pose2d(-45,   -38, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(90));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel*1.05, kMaxAccel*1.05, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFeedToCloseRebound() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(-253.5, -4.5, Rotation2d.fromDegrees(10)));
            headings.add(Rotation2d.fromDegrees(135));
            waypoints.add(new Pose2d(-130,   -40, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(90));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel*1.05, kMaxAccel*1.05, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFarLeftStartToFarLeftStaging() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0, 0, Rotation2d.fromDegrees(140)));
            headings.add(Rotation2d.fromDegrees(-40));
            waypoints.add(new Pose2d(-44,   42, Rotation2d.fromDegrees(140)));
            headings.add(Rotation2d.fromDegrees(-40));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFarLeftStagingToFarLeftBall1() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(-44, 42, Rotation2d.fromDegrees(-130)));
            headings.add(Rotation2d.fromDegrees(-40));
            waypoints.add(new Pose2d(-51,   35, Rotation2d.fromDegrees(-130)));
            headings.add(Rotation2d.fromDegrees(-40));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFarLeftBall1ToFarLeftBall2() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(-51,   35, Rotation2d.fromDegrees(50)));
            headings.add(Rotation2d.fromDegrees(-40));
            waypoints.add(new Pose2d(-36,   50, Rotation2d.fromDegrees(50)));
            headings.add(Rotation2d.fromDegrees(-40));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFarLeftBall2ToFarRightBall() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(-36,   50, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(-40));
            waypoints.add(new Pose2d(-28,   50, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(-30));
            waypoints.add(new Pose2d(-55,   -90, Rotation2d.fromDegrees(-90)));
            headings.add(Rotation2d.fromDegrees(0));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFarRightBallToFarLeftEject() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(-55,   -90, Rotation2d.fromDegrees(90)));
            headings.add(Rotation2d.fromDegrees(0));
            waypoints.add(new Pose2d(-55,   36, Rotation2d.fromDegrees(90)));
            headings.add(Rotation2d.fromDegrees(-90));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFarLeftEjectToFarLeftRebound() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(-55,   36, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(-90));
            waypoints.add(new Pose2d(-30,   36, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(-90));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFarLeftBall1ToFarLeftEject() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(-36,   50, Rotation2d.fromDegrees(140)));
            headings.add(Rotation2d.fromDegrees(-40));
            waypoints.add(new Pose2d(-55,   36, Rotation2d.fromDegrees(90)));
            headings.add(Rotation2d.fromDegrees(-90));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> getFarLeftBall2ToFarLeftEject() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(-51,   35, Rotation2d.fromDegrees(140)));
            headings.add(Rotation2d.fromDegrees(-40));
            waypoints.add(new Pose2d(-55,   36, Rotation2d.fromDegrees(90)));
            headings.add(Rotation2d.fromDegrees(-90));
            return generateTrajectory(false, waypoints, headings, List.of(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }
    }
}