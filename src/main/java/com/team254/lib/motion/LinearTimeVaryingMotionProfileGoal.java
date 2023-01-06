package com.team254.lib.motion;

import com.team254.lib.util.Util;

/**
 * A LinearTimeVaryingMotionProfileGoal defines a goal with a constant velocity whose position is a function of an
 * initial position, the velocity, and time. In other words, the goal state is constantly moving according to a time
 * varying linear function.
 */
public class LinearTimeVaryingMotionProfileGoal implements IMotionProfileGoal {
    protected double mT;
    protected double mInitialGoalPosition;
    protected double mGoalVel;
    protected CompletionBehavior mCompletionBehavior = CompletionBehavior.OVERSHOOT;
    protected double mPosTolerance = 1E-2;
    protected double mVelTolerance = 1E-2;

    public LinearTimeVaryingMotionProfileGoal(double t, double initial_goal_position, double goal_vel) {
        this.mT = t;
        this.mInitialGoalPosition = initial_goal_position;
        this.mGoalVel = goal_vel;
    }

    public LinearTimeVaryingMotionProfileGoal(double t, double initial_goal_position, double goal_vel, CompletionBehavior completion_behavior) {
        this(t, initial_goal_position, goal_vel);
        this.mCompletionBehavior = completion_behavior;
    }

    public LinearTimeVaryingMotionProfileGoal(double t, double initial_goal_position, double goal_vel, CompletionBehavior completion_behavior,
        double pos_tolerance, double vel_tolerance) {
        this(t, initial_goal_position, goal_vel, completion_behavior);
        this.mPosTolerance = pos_tolerance;
        this.mVelTolerance = vel_tolerance;
    }

    public double t() {
        return mT;
    }

    @Override
    public CompletionBehavior completion_behavior() {
        return mCompletionBehavior;
    }

    @Override
    public LinearTimeVaryingMotionProfileGoal flipped() {
        return new LinearTimeVaryingMotionProfileGoal(mT, -mInitialGoalPosition, -mGoalVel, mCompletionBehavior, mPosTolerance, mVelTolerance);
    }

    public double pos(double t) {
        return mInitialGoalPosition + (t-mT) * mGoalVel;
    }

    @Override
    public boolean atGoalState(MotionState state) {
        return Util.epsilonEquals(state.pos(), pos(state.t()), mPosTolerance) &&
               (Util.epsilonEquals(state.vel(), mGoalVel, mVelTolerance)
                || mCompletionBehavior == CompletionBehavior.VIOLATE_MAX_ABS_VEL);
    }

    /**
     * Create a MotionProfileGoal that has a zero velocity goal state, for use with superposition to work with a moving reference frame.
     *
     * @return A MotionProfileGoal for use with motion profile and setpoint generation.
     */
    public MotionProfileGoal toMotionProfileGoal(double t) {
        return new MotionProfileGoal(pos(t), 0.0, mCompletionBehavior, mPosTolerance, mVelTolerance);
    }

    @Override
    public double pos() {
        return mInitialGoalPosition;
    }

    @Override
    public double max_vel() {
        return mGoalVel;
    }

    @Override
    public double min_vel() {
        return mGoalVel;
    }

    @Override
    public double pos_tolerance() {
        return mPosTolerance;
    }

    @Override
    public double vel_tolerance() {
        return mVelTolerance;
    }

    @Override
    public boolean atGoalPos(double pos) {
        return Util.epsilonEquals(pos, pos(mT), mPosTolerance);
    }

}
