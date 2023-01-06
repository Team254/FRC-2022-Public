package com.team254.frc2022.subsystems;

import java.util.Optional;

import com.team254.frc2022.Constants;
import com.team254.frc2022.RobotState;
import com.team254.frc2022.shooting.ShootingUtil;
import com.team254.frc2022.states.LEDState;
import com.team254.frc2022.states.TimedLEDState;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.motion.LinearTimeVaryingMotionProfileGoal;
import com.team254.lib.motion.MotionProfileGoal;
import com.team254.lib.util.*;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {
    private static final boolean kAllowSuperEject = true;

    private static final double kIdlePower = 0.05;
    private static final double kEjectingRpm = 254.0;
    private static final double kEjectingHoodAngle = 10.0;
    private static final double kNearGoalThreshold = 2.25;
    private static final double kNearGoalTurretOffset = 65.0;
    private static final double kShootingTimeout = 0.25; // s
    private static final double kSuperEjectTargetPadding = 1.2; // meters from center of goal to aim at
    private static final double kSuperEjectMinOffset = 18; // min offset for super eject
    private static final double kSuperEjectHoodAngle = 33.5;

    private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kSuperEjectOffsetShotRpmMap = new InterpolatingTreeMap<>();
    static {
        kSuperEjectOffsetShotRpmMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(700.)); // make this the same as n+1
        kSuperEjectOffsetShotRpmMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(700.));
        kSuperEjectOffsetShotRpmMap.put(new InterpolatingDouble(4.5), new InterpolatingDouble(750.));
        kSuperEjectOffsetShotRpmMap.put(new InterpolatingDouble(6.0), new InterpolatingDouble(850.));
        kSuperEjectOffsetShotRpmMap.put(new InterpolatingDouble(7.5), new InterpolatingDouble(900.));
        kSuperEjectOffsetShotRpmMap.put(new InterpolatingDouble(20.), new InterpolatingDouble(900.)); // make this the same as n-1
    }


    private final Serializer mSerializer = Serializer.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Turret mTurret = Turret.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();
    private final LED mLED = LED.getInstance();

    private boolean mHasTarget = false;

    private final ShootingParameters mShootingParameters = Constants.kShootingParams;
    private Optional<AimingParameters> mLatestAimingParameters = Optional.empty();
    private boolean mShooterSetpointValidForShooting = false;

    private Optional<Rotation2d> mFieldRelativeTurretHint = Optional.empty();

    private WantedState mWantedState = WantedState.IDLE;
    private SystemState mSystemState = SystemState.IDLE;
    private double mCurrentStateStart = 0.0;
    private double mWasShootingAt = 0.0;
    private double mLastWantShootTime = 0.0;
    private final LatchedBoolean mWantShootRisingEdge = new LatchedBoolean();
    private final StickyBoolean mStartedEjecting = new StickyBoolean();
    private CloseSideSuperEjectMode mCloseSideEjectMode = CloseSideSuperEjectMode.TRUSS_EJECT;
    private final SendableChooser<CloseSideSuperEjectMode> mCloseSideEjectModeChooser;

    private boolean mHoodWasReady = false;
    private boolean mShooterWasReady = false;
    private boolean mTurretWasReady = false;

    private final TuningLogger hoodLogger = new TuningLogger("/home/lvuser/SHOOTER-HOOD.txt");
    private final TuningLogger rpmLogger = new TuningLogger("/home/lvuser/SHOOTER-RPM.txt");

    private final PeriodicIO mPeriodicIO = new PeriodicIO();

    private static Superstructure mInstance = null;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    public enum WantedState {
        IDLE, SHOOTING, CLIMBING
    }

    public enum SystemState {
        IDLE, EJECTING, SHOOTING, SPINNING_UP, CLIMBING
    }

    public enum CloseSideSuperEjectMode {
        TRUSS_EJECT, CENTER_EJECT, NO_CLOSE_SUPER_EJECT
    }

    public static class PeriodicIO {
        public double shooterRPM = Constants.kDefaultShooterRPM;
        public double hoodAngle = 0.0;
        public double turretAngle = 0.0;
        public double turretFeedforwardV = 0.0;
        public double range = 0.0;
        public double rangeOffset = Constants.kRangeOffset;
    }

    private Superstructure() {
        // Put these tuning values to Smart Dashboard one time so we can read from them later
        SmartDashboard.putNumber("Shooter RPM to Set", Constants.kDefaultShooterRPM);
        SmartDashboard.putNumber("Hood Angle to Set", 0.0);

        // On-the-fly shooter adjustment
        SmartDashboard.putNumber("Shooter range offset", Constants.kRangeOffset);
        mCloseSideEjectModeChooser = new SendableChooser<>();
        mCloseSideEjectModeChooser.setDefaultOption("Eject Toward Truss", CloseSideSuperEjectMode.TRUSS_EJECT);
        mCloseSideEjectModeChooser.addOption("Eject Toward Middle Driver Station", CloseSideSuperEjectMode.CENTER_EJECT);
        mCloseSideEjectModeChooser.addOption("No Close Side Super Eject", CloseSideSuperEjectMode.NO_CLOSE_SUPER_EJECT);
        SmartDashboard.putData("Close Side Eject Mode", mCloseSideEjectModeChooser);
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
        if (mWantShootRisingEdge.update(mWantedState == WantedState.SHOOTING)) {
            mLastWantShootTime = Timer.getFPGATimestamp();
        }
    }

    public synchronized boolean isShooting() {
        return mSystemState == SystemState.SHOOTING;
    }

    private double mHoodEjectTolerance = 3;
    private double mTurretEjectTolerance = 1.0;
    public synchronized void setEjectConstraints(double hood, double turret) {
        mHoodEjectTolerance = hood;
        mTurretEjectTolerance = turret;
    }

    public synchronized void resetEjectConstraints() {
        mHoodEjectTolerance = 3;
        mTurretEjectTolerance = 1.0;
    }

    public synchronized boolean isReadyToEject() {
        if (mSystemState != SystemState.EJECTING) {
            return false;
        } else {
            boolean timedOut = Timer.getFPGATimestamp() - mCurrentStateStart > 1.5;
            boolean hoodAtTarget = Util.epsilonEquals(mHood.getAngle(), mPeriodicIO.hoodAngle, mHoodEjectTolerance);
            boolean turretAtTarget = Util.epsilonEquals(mTurret.getPosition(), mPeriodicIO.turretAngle, mTurretEjectTolerance);
            return mStartedEjecting.update(timedOut || (hoodAtTarget && turretAtTarget));
        }
    }

    public synchronized boolean withinHint(double timestamp, Rotation2d acceptableHintError) {
        if (mFieldRelativeTurretHint.isPresent()) {
            Rotation2d turret_error = RobotState.getInstance().getFieldToVehicle(timestamp).inverse().getRotation()
                    .rotateBy(mFieldRelativeTurretHint.get().getRotation());
            return Util.inRange(turret_error.getDegrees(), acceptableHintError.getDegrees());
        }
        return true;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        int track_id = -1;
        if (mLatestAimingParameters.isPresent()) {
            track_id = mLatestAimingParameters.get().getTrackId();
        }
        mLatestAimingParameters = mRobotState.getAimingParameters(track_id, Constants.kGoalTrackerConstants.kMaxGoalTrackAge);
        if (mLatestAimingParameters.isPresent()) {
            mPeriodicIO.range = mLatestAimingParameters.get().getRange();
        }

        mPeriodicIO.rangeOffset = SmartDashboard.getNumber("Shooter range offset", Constants.kRangeOffset);

        double timestamp = Timer.getFPGATimestamp();
        setShooterSetpoints(timestamp);
        setEjectSetpoints(timestamp);

        if (Constants.kIsShooterTuning) {
            mPeriodicIO.shooterRPM = SmartDashboard.getNumber("Shooter RPM to Set", Constants.kDefaultShooterRPM);
            mPeriodicIO.hoodAngle = SmartDashboard.getNumber("Hood Angle to Set", Constants.kHoodConstants.kHomePosition);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    boolean shootingFallingEdge = timestamp - mWasShootingAt < kShootingTimeout && mSerializer.getBannerSensorFallingEdge();
                    if (mWantedState == WantedState.SHOOTING || shootingFallingEdge) {
                        mWasShootingAt = timestamp;
                    }

                    SystemState newState = handleAll(timestamp);
                    if (newState != mSystemState) {
                        System.out.println("Superstructure " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        mCurrentStateStart = timestamp;

                        if (newState == SystemState.EJECTING) {
                            setEjectSetpoints(timestamp);
                        }
                    }

                    // Always set turret and hood
                    if (mSystemState != SystemState.CLIMBING && mSystemState != SystemState.EJECTING) {
                        mTurret.setSetpointMotionProfiling(
                                new LinearTimeVaryingMotionProfileGoal(
                                        timestamp,
                                        mPeriodicIO.turretAngle,
                                        mPeriodicIO.turretFeedforwardV,
                                        MotionProfileGoal.CompletionBehavior.VIOLATE_MAX_ACCEL),
                                0.0);
                        mHood.setSetpointPositionPID(mPeriodicIO.hoodAngle);
                    }

                    // Reset ejecting states
                    if (mSystemState != SystemState.EJECTING) {
                        mStartedEjecting.reset();
                    }

                    switch (mSystemState) {
                        case CLIMBING:
                            mShooter.setOpenLoop(0);
                            mTurret.setSetpointMotionProfiling(new MotionProfileGoal(0.0), 0.0);
                            mHood.setSetpointPositionPID(0.0);
                            mInfrastructure.enableCompressorAnalog();
                            break;
                        case EJECTING:
                            mTurret.setSetpointPositionPID(mPeriodicIO.turretAngle);
                            mShooter.setRPM(mPeriodicIO.shooterRPM);
                            mHood.setSetpointPositionPID(mPeriodicIO.hoodAngle);
                            mLED.setSuperstructureLEDState(TimedLEDState.BlinkingLEDState.kBlinkingEject);
                            mInfrastructure.enableCompressorAnalog();
                            break;
                        case SPINNING_UP:
                            if (mWantedState == WantedState.SHOOTING || mSerializer.getNumBannersTriggered() == 0) {
                                TimedLEDState state = new TimedLEDState.StaticLEDState(new LEDState(
                                        (mShooterWasReady) ? 255 : 0,
                                        (mTurretWasReady) ? 255 : 0,
                                        (mHoodWasReady) ? 255 : 0
                                ));
                                mLED.setSuperstructureLEDState(state);
                            } else if (mSerializer.getNumBannersTriggered() == 1) {
                                mLED.setSuperstructureLEDState(TimedLEDState.StaticLEDState.kStaticOneBallInRobot);
                            } else if (mSerializer.getNumBannersTriggered() > 1) {
                                mLED.setSuperstructureLEDState(TimedLEDState.BlinkingLEDState.kBlinkingTwoBalls);
                            }
                            mShooter.setRPM(mPeriodicIO.shooterRPM);
                            mInfrastructure.enableCompressorAnalog();
                            break;
                        case SHOOTING:
                            mLED.setSuperstructureLEDState(TimedLEDState.BlinkingLEDState.kBlinkingShooting);
                            mShooter.setRPM(mPeriodicIO.shooterRPM);
                            mInfrastructure.disableCompressor();
                            break;
                        case IDLE:
                            if (mSerializer.getNumBannersTriggered() == 0) {
                                mLED.setSuperstructureLEDState(TimedLEDState.StaticLEDState.kStaticOff);
                            } else if (mSerializer.getNumBannersTriggered() == 1) {
                                mLED.setSuperstructureLEDState(TimedLEDState.StaticLEDState.kStaticOneBallInRobot);
                            } else {
                                mLED.setSuperstructureLEDState(TimedLEDState.BlinkingLEDState.kBlinkingTwoBalls);
                            }
                            mShooter.setOpenLoop(kIdlePower);
                            mInfrastructure.enableCompressorAnalog();
                            mShooterWasReady = false;
                            mHoodWasReady = false;
                            mTurretWasReady = false;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    private SystemState handleAll(double timestamp) {
        if (mSerializer.isEjecting()) {
            return SystemState.EJECTING;
        }

        switch (mWantedState) {
            case CLIMBING:
                return SystemState.CLIMBING;
            case SHOOTING:
                return handleShooting(timestamp);
            case IDLE:
            default:
                // when we swap back to idle from shooting, keep it shooting for a short amount of time
                if (timestamp - mWasShootingAt < kShootingTimeout) {
                    return handleShooting(timestamp);
                } else if (mLatestAimingParameters.isPresent() && mSerializer.readyToShoot()) {
                    return SystemState.SPINNING_UP;
                }
                return SystemState.IDLE;
        }
    }

    private SystemState handleShooting(double timestamp) {
        switch (mSystemState) {
            case SPINNING_UP:
                boolean hoodReady = mShootingParameters.isHoodAtSetpoint(mHood.getAngle(), mPeriodicIO.hoodAngle);
                boolean shooterReady = mShootingParameters.isShooterAtSetpoint(mShooter.getLeftRPM(), mPeriodicIO.shooterRPM);
                boolean turretReady = mShootingParameters.isTurretAtSetpoint(mTurret.getPosition(), mPeriodicIO.turretAngle, mPeriodicIO.range);
                boolean ready = hoodReady && shooterReady && turretReady
                        && mShooterSetpointValidForShooting;
                if (ready) {
                    System.out.println("Time in: " + (timestamp - mLastWantShootTime)
                            + " Making shot: range: " + mPeriodicIO.range
                            + " hood: " + mPeriodicIO.hoodAngle
                            + " RPM: " + mPeriodicIO.shooterRPM
                            + " hoodWasReady: " + mHoodWasReady
                            + " turretWasReady: " + mTurretWasReady
                            + " shooterWasReady: " + mShooterWasReady);
                    return SystemState.SHOOTING;
                }
                mHoodWasReady = hoodReady;
                mShooterWasReady = shooterReady;
                mTurretWasReady = turretReady;
                return SystemState.SPINNING_UP;
            case SHOOTING:
                return SystemState.SHOOTING;
            case CLIMBING:
            default:
                return SystemState.SPINNING_UP;
        }
    }

    protected class HoodAndRpm {
        public double hoodAngle;
        public double rpm;
    }

    private HoodAndRpm getHoodAndRpm(double range) {
        HoodAndRpm params = new HoodAndRpm();
        var horizontalRpm = Constants.kHorizontalRpmOffset + Constants.kHorizontalRpmPerMeter * range;
        var verticalRpm = Constants.kVerticalRpmOffset + Constants.kVerticalRpmPerMeter * range;
        params.hoodAngle = Math.toDegrees(Math.atan(horizontalRpm / verticalRpm)) - Constants.kHoodAngleOffset;
        params.rpm = Math.hypot(horizontalRpm, verticalRpm);
        return params;
    }

    private Rotation2d mEjectGoal = Rotation2d.fromDegrees(180.0);

    public synchronized void setAimingGoal(Rotation2d rotation) {
        mEjectGoal = rotation;
    }

    private void setEjectSetpoints(double timestamp) {
        if (mSystemState != SystemState.EJECTING) {
            return;
        }

        final double yOffsetForZones = 1.75;
        final double minSouthShotX = -2.5;

        boolean leftSide = mGoalToRobot.y() > 0; // robot is on the left side of the field
        boolean inMiddleOfY = mGoalToRobot.y() > -yOffsetForZones && mGoalToRobot.y() < yOffsetForZones;

        // Do south shot when to the left/right of the hub and when not super close to the close field wall
        boolean doSuperEjectSouthShot = mGoalToRobot.x() > 0 && !inMiddleOfY;

        // Do offset turret from vision target shot when behind the hub on the far side
        boolean doSuperEjectOffsetShot = mGoalToRobot.x() > 0 && inMiddleOfY;

        // When near the hub, don't shoot 1 pt goals
        boolean doSafeNearHubEject = mGoalToRobot.norm() < kNearGoalThreshold;

        // When on close side, super eject toward truss
        boolean doSuperEjectTrussShot = mGoalToRobot.x() < 0;

        // Default to regular eject
        mPeriodicIO.hoodAngle = kEjectingHoodAngle;
        mPeriodicIO.shooterRPM = kEjectingRpm;
        mPeriodicIO.turretFeedforwardV = 0.0;
        Optional<AimingParameters> aimingParameters = mLatestAimingParameters;
        mCloseSideEjectMode = mCloseSideEjectModeChooser.getSelected();
        if(mCloseSideEjectMode == CloseSideSuperEjectMode.NO_CLOSE_SUPER_EJECT) {
            if (doSafeNearHubEject) {
                // Move turret so ball doesn't shoot into 1 pt goal
                double newTurretAngle = mPeriodicIO.turretAngle + (leftSide ? -kNearGoalTurretOffset : kNearGoalTurretOffset);
                if (newTurretAngle <= Constants.kTurretConstants.kMinUnitsLimit || newTurretAngle >= Constants.kTurretConstants.kMaxUnitsLimit) {
                    newTurretAngle = mPeriodicIO.turretAngle + (leftSide ? kNearGoalTurretOffset : -kNearGoalTurretOffset);
                }
                mPeriodicIO.turretAngle = newTurretAngle;
                // hood angle and shooter rpm already set
            } else if (doSuperEjectOffsetShot && kAllowSuperEject) {
                // Find a new turret angle that is slightly offset from goal
                double angleOffsetFromTarget = Math.max(kSuperEjectMinOffset, Math.abs(Math.toDegrees(Math.atan2(kSuperEjectTargetPadding, mPeriodicIO.range))));
                double newTurretAngle = mPeriodicIO.turretAngle + (leftSide ? -angleOffsetFromTarget : angleOffsetFromTarget);
                boolean turretCanReachEjectSetpoint = Util.inRange(newTurretAngle, Constants.kTurretConstants.kMinUnitsLimit, Constants.kTurretConstants.kMaxUnitsLimit);
                if (!turretCanReachEjectSetpoint) {
                    mPeriodicIO.turretAngle = mTurret.getAngle();
                    return;
                }
                mPeriodicIO.turretAngle = newTurretAngle;
                mPeriodicIO.shooterRPM = kSuperEjectOffsetShotRpmMap.getInterpolated(new InterpolatingDouble(mPeriodicIO.range)).value;
                mPeriodicIO.hoodAngle = kSuperEjectHoodAngle;
            } else if (doSuperEjectSouthShot && aimingParameters.isPresent() && kAllowSuperEject) {
                double aimingGoal = leftSide ? mEjectGoal.getDegrees() + 15 : mEjectGoal.getDegrees() - 15; // slightly right or left of straight back based on side
                double newTurretAngle = Rotation2d.fromDegrees(aimingGoal).rotateBy(aimingParameters.get().getFieldToVehicle().getRotation().inverse()).getDegrees();
                boolean turretCanReachEjectSetpoint = Util.inRange(newTurretAngle, Constants.kTurretConstants.kMinUnitsLimit, Constants.kTurretConstants.kMaxUnitsLimit);
                if (!turretCanReachEjectSetpoint) {
                    mPeriodicIO.turretAngle = mTurret.getAngle();
                    return;
                }
                mPeriodicIO.shooterRPM = 1000.0;
                mPeriodicIO.turretAngle = newTurretAngle;
                mPeriodicIO.hoodAngle = kSuperEjectHoodAngle;
            }
        } else if (aimingParameters.isPresent()){
            Optional<Pose2d> targetPose = Optional.empty();
            final Pose2d vehicle_to_goal = aimingParameters.get().getVehicleToGoal();
            final Pose2d field_to_vehicle = aimingParameters.get().getFieldToVehicle();

            if (doSafeNearHubEject) {
                // Move turret so ball doesn't shoot into 1 pt goal
                Pose2d field_to_eject = field_to_vehicle.transformBy(Pose2d.fromTranslation(vehicle_to_goal.getTranslation().rotateBy(
                        (leftSide) ? Rotation2d.fromDegrees(-kNearGoalTurretOffset) : Rotation2d.fromDegrees(kNearGoalTurretOffset))));
                // hood angle and shooter rpm already set
                targetPose = Optional.of(field_to_eject);
            } else if (doSuperEjectOffsetShot && kAllowSuperEject) {
                // Find a new turret angle that is slightly offset from goal
                double angleOffsetFromTarget = Math.max(kSuperEjectMinOffset, Math.abs(Math.toDegrees(Math.atan2(kSuperEjectTargetPadding, mPeriodicIO.range))));
                Pose2d field_to_eject = field_to_vehicle.transformBy(Pose2d.fromTranslation(vehicle_to_goal.getTranslation().rotateBy(
                        (leftSide) ? Rotation2d.fromDegrees(-angleOffsetFromTarget) : Rotation2d.fromDegrees(angleOffsetFromTarget))));
                targetPose = Optional.of(field_to_eject);
                mPeriodicIO.shooterRPM = kSuperEjectOffsetShotRpmMap.getInterpolated(new InterpolatingDouble(mPeriodicIO.range)).value;
                mPeriodicIO.hoodAngle = kSuperEjectHoodAngle;
            } else if (doSuperEjectSouthShot && kAllowSuperEject) {
                double aimingGoal = leftSide ? mEjectGoal.getDegrees() + 10 : mEjectGoal.getDegrees() - 15; // slightly right or left of straight back based on side
                Pose2d field_to_eject =
                        new Pose2d(field_to_vehicle.getTranslation(), Rotation2d.fromDegrees(aimingGoal)).transformBy(
                                Pose2d.fromTranslation(new Translation2d(mGoalToRobot.x() + 7.0, 0.0)));
                if (mGoalToRobot.y() < 0) {
                    mPeriodicIO.shooterRPM = 1000.0;
                } else {
                    mPeriodicIO.shooterRPM = 850.0 + 75.0 * mGoalToRobot.x();
                }
                targetPose = Optional.of(field_to_eject);
                mPeriodicIO.hoodAngle = kSuperEjectHoodAngle;
            } else if (doSuperEjectTrussShot && kAllowSuperEject) {
                Translation2d targetPoint;
                if (mCloseSideEjectModeChooser.getSelected() == CloseSideSuperEjectMode.TRUSS_EJECT) {
                    targetPoint = Constants.goalToTruss;
                } else {
                    targetPoint = Constants.goalToMiddleDriverStation;
                }
                Pose2d fieldToTruss = Pose2d.fromTranslation(aimingParameters.get().getFieldToGoal().getTranslation().translateBy(targetPoint));
                targetPose = Optional.of(fieldToTruss);
                mPeriodicIO.shooterRPM = 100.0 + 60.0 * aimingParameters.get().getFieldToVehicle().inverse().transformBy(fieldToTruss).getTranslation().norm(); // Intentionally large number
                mPeriodicIO.hoodAngle = kSuperEjectHoodAngle;
            }

            boolean fallback = true;
            if (targetPose.isPresent()) {
                fallback = false;
                var shooting_params = ShootingUtil.getShootingParameters(mTurret.getAngle(),
                        RobotState.getInstance().getVehicleToTurret(timestamp).inverse()
                                .transformBy(RobotState.getInstance().getFieldToVehicle(timestamp).inverse()),
                        new AimingParameters(field_to_vehicle, targetPose.get(), -1, false),
                        1.05, mRobotState.getMeasuredVelocity());
                mPeriodicIO.turretFeedforwardV = shooting_params.effective_yaw_feedforward_velocity_deg_s;
                if (Math.abs(shooting_params.effective_yaw_angle_deg - shooting_params.uncompensated_yaw_angle_deg) >= Constants.kMaxTurretAdjustmentForShootOnMove) {
                    shooting_params.effective_yaw_angle_deg = Math.signum(shooting_params.effective_yaw_angle_deg - shooting_params.uncompensated_yaw_angle_deg) * Constants.kMaxTurretAdjustmentForShootOnMove + shooting_params.uncompensated_yaw_angle_deg;
                    fallback = true;
                }
                mPeriodicIO.turretAngle = Util.limit(shooting_params.effective_yaw_angle_deg, Constants.kTurretConstants.kMinUnitsLimit, Constants.kTurretConstants.kMaxUnitsLimit);
                if (mPeriodicIO.turretAngle != shooting_params.effective_yaw_angle_deg) {
                    // Turret wrapped, disable feedforward.
                    mPeriodicIO.turretFeedforwardV = 0.0;
                    fallback = true;
                }
            }

            if (fallback) {
                // Otherwise, we default to weak eject.
                mPeriodicIO.hoodAngle = kEjectingHoodAngle;
                mPeriodicIO.shooterRPM = kEjectingRpm;
            }
        }
    }

    private void setShooterSetpoints(double timestamp) {
        if (mLatestAimingParameters.isPresent()) {
            resetFieldRelativeTurretHint();
            mShooterSetpointValidForShooting = true;

            Pose2d turretToField = RobotState.getInstance().getVehicleToTurret(timestamp).inverse()
                .transformBy(RobotState.getInstance().getFieldToVehicle(timestamp).inverse());

            final double kShotTime = 1.05;

            var shooting_params = ShootingUtil.getShootingParameters(mTurret.getAngle(), turretToField, mLatestAimingParameters.get(), kShotTime, mRobotState.getMeasuredVelocity());
            if (Math.abs(shooting_params.effective_yaw_angle_deg - shooting_params.uncompensated_yaw_angle_deg) >= Constants.kMaxTurretAdjustmentForShootOnMove) {
                shooting_params.effective_yaw_angle_deg = Math.signum(shooting_params.effective_yaw_angle_deg - shooting_params.uncompensated_yaw_angle_deg) * Constants.kMaxTurretAdjustmentForShootOnMove + shooting_params.uncompensated_yaw_angle_deg;
                mShooterSetpointValidForShooting = false;
            }
            mPeriodicIO.turretFeedforwardV = shooting_params.effective_yaw_feedforward_velocity_deg_s;
            mPeriodicIO.turretAngle = handleTurretLimit(shooting_params.effective_yaw_angle_deg);
            if (mPeriodicIO.turretAngle != shooting_params.effective_yaw_angle_deg) {
                // Turret wrapped, disable feedforward.
                mPeriodicIO.turretFeedforwardV = 0.0;
                mShooterSetpointValidForShooting = false;
            }
            var hoodAndRpm = getHoodAndRpm(shooting_params.effective_range_m + mPeriodicIO.rangeOffset);
            if (!Constants.kUseSinMap) {
                hoodAndRpm.hoodAngle = Constants.kHoodMap.getInterpolated(new InterpolatingDouble(shooting_params.effective_range_m + mPeriodicIO.rangeOffset)).value;//Constants.kHoodRegression.predict(range + mPeriodicIO.rangeOffset);
                hoodAndRpm.rpm = Constants.kRPMMap.getInterpolated(new InterpolatingDouble(shooting_params.effective_range_m + mPeriodicIO.rangeOffset)).value; //Constants.kRPMRegression.predict(range + mPeriodicIO.rangeOffset);
            }
            mPeriodicIO.hoodAngle = handleHoodLimit(hoodAndRpm.hoodAngle);
            mPeriodicIO.shooterRPM = handleRPMLimit(hoodAndRpm.rpm);
            if (mPeriodicIO.hoodAngle != hoodAndRpm.hoodAngle || mPeriodicIO.shooterRPM != hoodAndRpm.rpm) {
                mShooterSetpointValidForShooting = false;
            }
            mGoalToRobot = getGoalToRobot();
            mHasTarget = true;
        } else if (mFieldRelativeTurretHint.isPresent()) {
            Rotation2d setpoint = RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation().inverse()
                    .rotateBy(mFieldRelativeTurretHint.get());

            mPeriodicIO.turretFeedforwardV = 0.0;
            mPeriodicIO.turretAngle = handleTurretLimit(setpoint.getDegrees());
            mHasTarget = false;
        } else {
            mPeriodicIO.turretFeedforwardV = 0.0;
            mPeriodicIO.turretAngle = 0.0;
            mShooterSetpointValidForShooting = false;

            mHasTarget = false;
        }

        if (mShooterSetpointValidForShooting && mLatestAimingParameters.isPresent()) {
            if (!mLatestAimingParameters.get().getIsLatest()) mShooterSetpointValidForShooting = false;
        }
    }

    private double handleHoodLimit(double hood_setpoint) {
        return Util.limit(hood_setpoint, Constants.kHoodConstants.kMinUnitsLimit, Constants.kMaxHoodAngleForShooting);
    }

    private double handleRPMLimit(double rpm_setpoint) {
        return rpm_setpoint;
        //return Util.limit(rpm_setpoint, Constants.kDefaultShooterRPM, Constants.kMaxShooterRPM);
    }


    private double handleTurretLimit(double turret_setpoint) {
        double posFlipThreshold = 190;
        double negFlipThreshold = -190;
        if (turret_setpoint >= posFlipThreshold) {
            return -90.0;
        } else if (turret_setpoint <= negFlipThreshold) {
            return 90.0;
        }
        return turret_setpoint;
    }

    public void logShooterValues() {
        String rpmName = "kRPMMap";
        String hoodName = "kHoodMap";

        rpmLogger.logData(rpmName, mPeriodicIO.range, mPeriodicIO.shooterRPM);
        hoodLogger.logData(hoodName, mPeriodicIO.range, mPeriodicIO.hoodAngle);
    }

    public synchronized void setFieldRelativeTurretHint(Rotation2d hint) {
        mFieldRelativeTurretHint = Optional.of(hint);
    }

    public synchronized void resetFieldRelativeTurretHint() {
        mFieldRelativeTurretHint = Optional.empty();
    }

    public synchronized boolean hasTarget() {
        return mHasTarget;
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    private Translation2d mGoalToRobot = Translation2d.identity();  // TODO update this when we get vision updates, else set null?
    private Translation2d getGoalToRobot() {
        if (mLatestAimingParameters.isEmpty()) {
            return null;
        }
        Rotation2d goalToRobotRotation =
                mLatestAimingParameters.get().getRobotToGoalRotation()
                        .rotateBy(mLatestAimingParameters.get().getFieldToVehicle().getRotation())  // convert to field relative robot heading
                        .rotateBy(Rotation2d.fromDegrees(180)); // flip from robot -> goal to goal -> robot
        return new Translation2d(
                mLatestAimingParameters.get().getRange() * goalToRobotRotation.cos(),
                mLatestAimingParameters.get().getRange() * goalToRobotRotation.sin()
        );
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Superstructure Wanted State", mWantedState.toString());
        SmartDashboard.putString("Superstructure System State", mSystemState.toString());

        SmartDashboard.putNumber("Range", mPeriodicIO.range);
        SmartDashboard.putNumber("Shooter RPM Setpoint", mPeriodicIO.shooterRPM);
        SmartDashboard.putNumber("Hood Setpoint", mPeriodicIO.hoodAngle);
        SmartDashboard.putNumber("Turret Setpoint", mPeriodicIO.turretAngle);

        SmartDashboard.putBoolean("Has Target", mHasTarget);

        SmartDashboard.putBoolean("Shooter at setpoint", mShootingParameters.isShooterAtSetpoint(mShooter.getLeftRPM(), mPeriodicIO.shooterRPM));
        SmartDashboard.putBoolean("Hood at setpoint",  mShootingParameters.isHoodAtSetpoint(mHood.getAngle(), mPeriodicIO.hoodAngle));
        SmartDashboard.putBoolean("Turret at setpoint", mShootingParameters.isTurretAtSetpoint(mTurret.getPosition(), mPeriodicIO.turretAngle, mPeriodicIO.range));
        mLatestAimingParameters.ifPresent(x -> SmartDashboard.putString("vehicle_to_goal", x.getVehicleToGoal().toString()));
        mLatestAimingParameters.ifPresent(x -> SmartDashboard.putString("field_to_vehicle", x.getFieldToVehicle().toString()));

        SmartDashboard.putString("Goal To Robot", mGoalToRobot.toString());
    }
}