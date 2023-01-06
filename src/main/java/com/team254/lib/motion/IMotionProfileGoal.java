package com.team254.lib.motion;

/**
 * A MotionProfileGoal defines a desired position and maximum velocity (at this position), along with the behavior that
 * should be used to determine if we are at the goal and what to do if it is infeasible to reach the goal within the
 * desired velocity bounds.
 */
public interface IMotionProfileGoal {
    /**
     * What should we do if we would
     * reach the goal at a velocity greater than the maximum? This enum allows a user to specify a preference on
     * behavior in this case.
     * <p>
     * Example use-cases of each:
     * <p>
     * OVERSHOOT - Generally used with a goal max_abs_vel of 0.0 to stop at the desired pos without violating any
     * constraints.
     * <p>
     * VIOLATE_MAX_ACCEL - If we absolutely do not want to pass the goal and are unwilling to violate the max_abs_vel
     * (for example, there is an obstacle in front of us - slam the brakes harder than we'd like in order to avoid
     * hitting it).
     * <p>
     * VIOLATE_MAX_ABS_VEL - If the max velocity is just a general guideline and not a hard performance limit, it's
     * better to slightly exceed it to avoid skidding wheels.
     */
    public enum CompletionBehavior {
        // Overshoot the goal if necessary (at a velocity greater than max_abs_vel) and come back.
        // Only valid if the goal velocity is 0.0 (otherwise VIOLATE_MAX_ACCEL will be used).
        OVERSHOOT,
        // If we cannot slow down to the goal velocity before crossing the goal, allow exceeding the max accel
        // constraint.
        VIOLATE_MAX_ACCEL,
        // If we cannot slow down to the goal velocity before crossing the goal, allow exceeding the goal velocity.
        VIOLATE_MAX_ABS_VEL
    }

    public IMotionProfileGoal flipped();

    public double pos();

    public double max_vel();

    public double min_vel();

    public double pos_tolerance();

    public double vel_tolerance();

    public CompletionBehavior completion_behavior();

    public boolean atGoalState(MotionState state);

    public boolean atGoalPos(double pos);
}
