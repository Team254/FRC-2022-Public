package com.team254.frc2022.subsystems;

import com.team254.frc2022.Constants;
import com.team254.frc2022.RobotState;
import com.team254.frc2022.planners.DriveMotionPlanner;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.swerve.SwerveDriveOdometry;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Units;
import com.team254.lib.util.Util;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;

import com.team254.lib.drivers.Subsystem;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;

public class RobotStateEstimator extends Subsystem {
    static RobotStateEstimator mInstance = null;
    private Drive mDrive = null;
    private final SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(Constants.kKinematics, Pose2d.identity());

    public static class OdometryLogValues {
        public double timestamp;
        public double frontLeftPosition;
        public double frontRightPosition;
        public double backLeftPosition;
        public double backRightPosition;

        public double frontLeftVelocity;
        public double frontRightVelocity;
        public double backLeftVelocity;
        public double backRightVelocity;

        public double frontLeftAngle;
        public double frontRightAngle;
        public double backLeftAngle;
        public double backRightAngle;

        public double gyroAngle;

        public double frontLeftAngleSetpoint;
        public double frontRightAngleSetpoint;
        public double backLeftAngleSetpoint;
        public double backRightAngleSetpoint;

        public double frontLeftError;
        public double frontRightError;
        public double backLeftError;
        public double backRightError;

        public double timestamp1;
        public double poseX;
        public double autonXSetpoint;
        public double xError;
        public double timestamp2;
        public double poseY;
        public double autonYSetpoint;
        public double yError;
        public double timestamp3;
        public double poseTheta;
        public double autonHeadingSetpoint;
        public double headingError;


        public OdometryLogValues(double timestamp, double frontLeftPosition, double frontRightPosition, double backLeftPosition, double backRightPosition,
                                 double frontLeftVelocity, double frontRightVelocity, double backLeftVelocity, double backRightVelocity,
                                 double frontLeftAngle, double frontRightAngle, double backLeftAngle, double backRightAngle,
                                 double flCLosedLoopErr,double frCLosedLoopErr,double blCLosedLoopErr,double brCLosedLoopErr,
                                 double gyroAngle, double poseX, double poseY, double poseTheta,
                                 double frontLeftAngleSetpoint, double frontRightAngleSetpoint, double backLeftAngleSetpoint, double backRightAngleSetpoint,
                                 double xSetpoint, double ySetpoint, double headingSetpoint, double xError, double yError, double headingError) {
            this.timestamp = timestamp;
            this.timestamp1 = timestamp;
            this.timestamp2 = timestamp;
            this.timestamp3 = timestamp;
            this.frontLeftPosition = frontLeftPosition;
            this.frontRightPosition = frontRightPosition;
            this.backLeftPosition = backLeftPosition;
            this.backRightPosition = backRightPosition;
            this.frontLeftVelocity = frontLeftVelocity;
            this.frontRightVelocity = frontRightVelocity;
            this.backLeftVelocity = backLeftVelocity;
            this.backRightVelocity = backRightVelocity;
            this.frontLeftAngle = frontLeftAngle;
            this.frontRightAngle = frontRightAngle;
            this.backLeftAngle = backLeftAngle;
            this.backRightAngle = backRightAngle;
            this.gyroAngle = gyroAngle;
            this.poseX = poseX;
            this.poseY = poseY;
            this.poseTheta = poseTheta;
            this.frontLeftAngleSetpoint = frontLeftAngleSetpoint;
            this.frontRightAngleSetpoint = frontRightAngleSetpoint;
            this.backLeftAngleSetpoint = backLeftAngleSetpoint;
            this.backRightAngleSetpoint = backRightAngleSetpoint;
            this.autonXSetpoint = xSetpoint;
            this.autonYSetpoint = ySetpoint;
            this.autonHeadingSetpoint = headingSetpoint;
            this.xError = xError;
            this.yError = yError;
            this.headingError = headingError;
            this.frontLeftError = flCLosedLoopErr;
            this.frontRightError = frCLosedLoopErr;
            this.backLeftError = blCLosedLoopErr;
            this.backRightError = brCLosedLoopErr;
        }
    }

    private ReflectingCSVWriter<OdometryLogValues> mCSVWriter = null;

    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new RobotStateEstimator();
        }

        return mInstance;
    }

    private RobotStateEstimator() {
        mDrive = Drive.getInstance();
    }

    public Pose2d getEstimatedPose() {
        return mOdometry.getPoseMeters();
    }

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {
        @Override
        public synchronized void onStart(double timestamp) {
           //startLogging();
        }

        @Override
        public synchronized void onLoop(double timestamp) {
            synchronized(RobotStateEstimator.this) {
                mOdometry.update(mDrive.getFieldRelativeGyroscopeRotation(), mDrive.getModuleStates());

                Twist2d measured_velocity = mOdometry.getVelocity().toTwist2d();
                Twist2d predicted_velocity = mDrive.getSetpoint().mChassisSpeeds.toTwist2d();

                // TODO(maybe add actual velocity prediction)
                RobotState.getInstance().addObservations(timestamp, mOdometry.getPoseMeters(),
                    measured_velocity, predicted_velocity);

                // TODO(refactor to combine this method with the above)
                RobotState.getInstance().addVehicleToTurretObservation(
                    timestamp,
                    Pose2d.fromRotation(Rotation2d.fromDegrees(Turret.getInstance().getAngle())),
                        Turret.getInstance().getVelocity());

                if (mCSVWriter != null) {
                    try {
                        logCSV();
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stopLogging();
        }
    }

    @Override
    public void stop() {}

    public void resetOdometry(Pose2d initialPose) {
        synchronized(RobotStateEstimator.this) {
            mOdometry.resetPosition(initialPose);
            RobotState.getInstance().resetVision();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), initialPose);
        }
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/ODOMETRY-LOGS.csv", OdometryLogValues.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    private synchronized void logCSV() {
        double currentTime = WPIUtilJNI.now() * 1.0e-6;

        var modules_meas = mDrive.getModuleStates();
        var modules_des = mDrive.getDesiredModuleStates();


        double frontLeftPosition = modules_meas[Drive.kFrontLeftModuleIdx].distanceMeters;
        double frontRightPosition = modules_meas[Drive.kFrontRightModuleIdx].distanceMeters;
        double backLeftPosition = modules_meas[Drive.kBackLeftModuleIdx].distanceMeters;
        double backRightPosition = modules_meas[Drive.kBackRightModuleIdx].distanceMeters;

        double frontLeftVelocity = modules_meas[Drive.kFrontLeftModuleIdx].speedMetersPerSecond;
        double frontRightVelocity = modules_meas[Drive.kFrontRightModuleIdx].speedMetersPerSecond;
        double backLeftVelocity = modules_meas[Drive.kBackLeftModuleIdx].speedMetersPerSecond;
        double backRightVelocity = modules_meas[Drive.kBackRightModuleIdx].speedMetersPerSecond;

        double frontLeftAngle = modules_meas[Drive.kFrontLeftModuleIdx].angle.getRadians();
        double frontRightAngle = modules_meas[Drive.kFrontRightModuleIdx].angle.getRadians();
        double backLeftAngle = modules_meas[Drive.kBackLeftModuleIdx].angle.getRadians();
        double backRightAngle = modules_meas[Drive.kBackRightModuleIdx].angle.getRadians();

        double[] closed_loop_err = mDrive.getSteerClosedLoopErrors();
        double flErr = closed_loop_err[Drive.kFrontLeftModuleIdx];
        double frErr = closed_loop_err[Drive.kFrontRightModuleIdx];
        double brErr = closed_loop_err[Drive.kBackRightModuleIdx];
        double blErr = closed_loop_err[Drive.kBackLeftModuleIdx];

        boolean stationary = mOdometry.getVelocity().toTwist2d().epsilonEquals(Twist2d.identity(), Util.kEpsilon);

        Pose2d currentPose = mOdometry.getPoseMeters();

        var desiredStates = mDrive.getSetpoint().mModuleStates;

        Pose2d autonSetpoint = mDrive.getAutonSetpoint();
        Pose2d autonError = mDrive.getAutonError();

        OdometryLogValues logMsg = new OdometryLogValues(
                currentTime,
                frontLeftPosition,
                frontRightPosition,
                backLeftPosition,
                backRightPosition,
                frontLeftVelocity,
                frontRightVelocity,
                backLeftVelocity,
                backRightVelocity,
                frontLeftAngle,
                frontRightAngle,
                backLeftAngle,
                backRightAngle,
                flErr,
                frErr,
                blErr,
                brErr,
                mDrive.getFieldRelativeGyroscopeRotation().getDegrees(),
                currentPose.getTranslation().x(),
                currentPose.getTranslation().y(),
                currentPose.getRotation().getDegrees(),
                stationary ? frontLeftAngle : desiredStates[Drive.kFrontLeftModuleIdx].angle.getRadians(),
                stationary ? frontRightAngle : desiredStates[Drive.kFrontRightModuleIdx].angle.getRadians(),
                stationary ? backLeftAngle : desiredStates[Drive.kBackLeftModuleIdx].angle.getRadians(),
                stationary ? backRightAngle : desiredStates[Drive.kBackRightModuleIdx].angle.getRadians(),
                Units.inches_to_meters(autonSetpoint.getTranslation().x()),
                Units.inches_to_meters(autonSetpoint.getTranslation().y()),
                autonSetpoint.getRotation().getDegrees(),
                autonError.getTranslation().x(),
                autonError.getTranslation().y(),
                autonError.getRotation().getDegrees()
        );

        mCSVWriter.add(logMsg);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry() {
        RobotState.getInstance().outputToSmartDashboard();

//        if (mCSVWriter != null) {
//            mCSVWriter.write();
//        }
    }
}