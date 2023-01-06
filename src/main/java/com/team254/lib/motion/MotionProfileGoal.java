package com.team254.lib.motion;

import static com.team254.lib.util.Util.epsilonEquals;

/**
 * A MotionProfileGoal defines a desired position and maximum velocity (at this position), along with the behavior that
 * should be used to determine if we are at the goal and what to do if it is infeasible to reach the goal within the
 * desired velocity bounds.
 */
public class MotionProfileGoal implements IMotionProfileGoal {
    protected double pos;
    protected double max_vel;
    protected CompletionBehavior completion_behavior = CompletionBehavior.OVERSHOOT;
    protected double pos_tolerance = 1E-3;
    protected double vel_tolerance = 1E-2;

    public MotionProfileGoal() {}

    public MotionProfileGoal(double pos) {
        this.pos = pos;
        sanityCheck();
    }

    public MotionProfileGoal(double pos, double max_abs_vel) {
        this.pos = pos;
        this.max_vel = max_abs_vel;
        sanityCheck();
    }

    public MotionProfileGoal(double pos, double max_abs_vel, CompletionBehavior completion_behavior) {
        this.pos = pos;
        this.max_vel = max_abs_vel;
        this.completion_behavior = completion_behavior;
        sanityCheck();
    }

    public MotionProfileGoal(double pos, double max_abs_vel, CompletionBehavior completion_behavior,
                             double pos_tolerance, double vel_tolerance) {
        this.pos = pos;
        this.max_vel = max_abs_vel;
        this.completion_behavior = completion_behavior;
        this.pos_tolerance = pos_tolerance;
        this.vel_tolerance = vel_tolerance;
        sanityCheck();
    }

    public MotionProfileGoal(MotionProfileGoal other) {
        this(other.pos, other.max_vel, other.completion_behavior, other.pos_tolerance, other.vel_tolerance);
    }

    /**
     * @return A flipped MotionProfileGoal (where the position is negated, but all other attributes remain the same).
     */
    @Override
    public MotionProfileGoal flipped() {
        return new MotionProfileGoal(-pos, max_vel, completion_behavior, pos_tolerance, vel_tolerance);
    }

    @Override
    public double pos() {
        return pos;
    }

    @Override
    public double max_vel() {
        return max_vel;
    }

    @Override
    public double min_vel() {
        return -max_vel;
    }

    @Override
    public double pos_tolerance() {
        return pos_tolerance;
    }

    @Override
    public double vel_tolerance() {
        return vel_tolerance;
    }

    @Override
    public CompletionBehavior completion_behavior() {
        return completion_behavior;
    }

    @Override
    public boolean atGoalState(MotionState state) {
        return atGoalPos(state.pos()) && (Math.abs(state.vel()) < (max_vel + vel_tolerance)
                || completion_behavior == CompletionBehavior.VIOLATE_MAX_ABS_VEL);
    }

    @Override
    public boolean atGoalPos(double pos) {
        return epsilonEquals(pos, this.pos, pos_tolerance);
    }

    /**
     * This method makes sure that the completion behavior is compatible with the max goal velocity.
     */
    protected void sanityCheck() {
        if (max_vel > vel_tolerance && completion_behavior == CompletionBehavior.OVERSHOOT) {
            completion_behavior = CompletionBehavior.VIOLATE_MAX_ACCEL;
        }
    }

    @Override
    public String toString() {
        return "pos: " + pos + " (+/- " + pos_tolerance + "), max_vel: " + max_vel + " (+/- " + vel_tolerance
                + "), completion behavior: " + completion_behavior.name();
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof MotionProfileGoal)) {
            return false;
        }
        final MotionProfileGoal other = (MotionProfileGoal) obj;
        return (other.completion_behavior() == completion_behavior()) && (other.pos() == pos())
                && (other.max_vel() == max_vel()) && (other.min_vel() == min_vel()) && (other.pos_tolerance() == pos_tolerance())
                && (other.vel_tolerance() == vel_tolerance());
    }
}
