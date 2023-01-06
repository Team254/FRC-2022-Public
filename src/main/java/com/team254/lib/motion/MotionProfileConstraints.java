package com.team254.lib.motion;

/**
 * Constraints for constructing a MotionProfile.
 */
public class MotionProfileConstraints {
    protected double max_vel = Double.POSITIVE_INFINITY;
    protected double min_vel = Double.NEGATIVE_INFINITY;
    protected double max_abs_acc = Double.POSITIVE_INFINITY;

    public MotionProfileConstraints(double max_vel, double min_vel, double max_acc) {
        this.max_vel = Math.abs(max_vel);
        this.min_vel = -Math.abs(min_vel);
        this.max_abs_acc = Math.abs(max_acc);
    }

    /**
     * @return The (positive) maximum allowed velocity.
     */
    public double max_vel() {
        return max_vel;
    }

    /**
     * @return The (negative) minimum allowed velocity.
     */
    public double min_vel() {
        return min_vel;
    }

    /**
     * @return The (positive) maximum allowed acceleration.
     */
    public double max_abs_acc() {
        return max_abs_acc;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof MotionProfileConstraints)) {
            return false;
        }
        final MotionProfileConstraints other = (MotionProfileConstraints) obj;
        return (other.max_abs_acc() == max_abs_acc()) && (other.max_vel() == max_vel()) && (other.min_vel() == min_vel());
    }
}
