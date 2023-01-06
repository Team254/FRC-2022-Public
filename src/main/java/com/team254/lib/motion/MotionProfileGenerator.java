package com.team254.lib.motion;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import javax.sound.sampled.Line;

import com.team254.lib.motion.IMotionProfileGoal.CompletionBehavior;

/**
 * A MotionProfileGenerator generates minimum-time MotionProfiles to travel from a given MotionState to a given
 * MotionProfileGoal while obeying a set of MotionProfileConstraints.
 */
public class MotionProfileGenerator {
    // Static class.
    private MotionProfileGenerator() {}

    protected static MotionProfile generateFlippedProfile(MotionProfileConstraints constraints,
                                                          MotionProfileGoal goal_state, MotionState prev_state, boolean origFlipped) {
        MotionProfile profile = generateProfileImpl(constraints, goal_state.flipped(), prev_state.flipped(), !origFlipped);
        for (MotionSegment s : profile.segments()) {
            s.setStart(s.start().flipped());
            s.setEnd(s.end().flipped());
        }
        return profile;
    }

    public synchronized static MotionProfile generateProfile(MotionProfileConstraints constraints,
                                                             IMotionProfileGoal goal_state,
                                                             MotionState prev_state) {
        if (goal_state instanceof MotionProfileGoal) {
            return generateProfile(constraints, (MotionProfileGoal)goal_state, prev_state);
        } else {
            return generateProfile(constraints, (LinearTimeVaryingMotionProfileGoal)goal_state, prev_state);
        }
    }

    /**
     * Generate a motion profile.
     *
     * @param constraints The constraints to use.
     * @param goal_state  The goal to use.
     * @param prev_state  The initial state to use.
     * @return A motion profile from prev_state to goal_state that satisfies constraints.
     */
    public synchronized static MotionProfile generateProfile(MotionProfileConstraints constraints,
                                                             MotionProfileGoal goal_state,
                                                             MotionState prev_state) {
        return generateProfileImpl(constraints, goal_state, prev_state, false);
    }

    /**
     * Generate a motion profile to track a linear time-varying goal.
     *
     * @param constraints The constraints to use. Note that max_vel must == -min_vel.
     * @param goal_state  The goal to use. If goal_vel is outside the constraints, it is clamped.
     * @param prev_state  The initial state to use.
     * @return A motion profile from prev_state to goal_state that satisfies constraints.
     */
    public synchronized static MotionProfile generateProfile(MotionProfileConstraints constraints,
                                                             LinearTimeVaryingMotionProfileGoal goal_state,
                                                             MotionState prev_state) {
        // The trick we use to solve this problem simply requires symmetrical velocity constraints.
        assert(constraints.max_vel() == -constraints.min_vel());
        if (goal_state.max_vel() >= constraints.max_vel() || goal_state.max_vel() <= constraints.min_vel()) {
            // Reaching the goal exactly is impossible, so just move towards the goal as quickly as possible.
            return generateProfile(constraints, new MotionProfileGoal(Math.signum(goal_state.max_vel()) * Double.MAX_VALUE, goal_state.max_vel()), prev_state);
        }

        // Turn the linear time-varying goal into a stationary goal by superimposing the (opposite of) goal state motion
        // onto the initial state and all constraints. Then add the goal state motion back to the result.
        MotionState superposedPrevState = new MotionState(prev_state.t(), prev_state.pos(), prev_state.vel() - goal_state.max_vel(), prev_state.acc());
        MotionProfileGoal superposedGoal = goal_state.toMotionProfileGoal(prev_state.t());
        MotionProfileConstraints superposedConstraints =
            new MotionProfileConstraints(constraints.max_vel() - goal_state.max_vel(),
                                         constraints.min_vel() - goal_state.max_vel(),
                                         constraints.max_abs_acc());
        var profile = generateProfileImpl(superposedConstraints, superposedGoal, superposedPrevState, false);
        if (profile.size() == 0) return profile;

        // Now add goal velocity back in.
        for (MotionSegment s : profile.segments()) {
            s.setStart(shiftByVelocity(s.start(), goal_state.max_vel(), prev_state.t()));
            s.setEnd(shiftByVelocity(s.end(), goal_state.max_vel(), prev_state.t()));
        }

        // As a result of the shift, individual segments may now (a) be able to be combined, (b) have to be split in order
        // to ensure that no segment has both positive and negative velocities.

        // Combine segments with identical states after shifting.
        var segments = profile.segments();
        MotionSegment cur = null;
        for (Iterator<MotionSegment> iterator = segments.iterator(); iterator.hasNext() && segments.size() > 1; ) {
            if (cur == null) {
                cur = iterator.next();
            }
            if (iterator.hasNext()) {
                MotionSegment next = iterator.next();
                if (cur.end().equals(next.start())) {
                    // Concatenate 'next' into 'cur' and remove 'next'.
                    cur.setEnd(next.end());
                    iterator.remove();
                } else {
                    cur = next;
                }
            }
        }

        // Split segments that change direction after shifting.
        List<MotionSegment> new_segments = new ArrayList<MotionSegment>(profile.size());
        for (var segment : profile.segments()) {
            if (segment.start().vel() * segment.end().vel() >= 0.0) {
                new_segments.add(segment);
            } else {
                // Velocity sign changes midway through segment. Figure out where and make two new segments around it.
                double interp = -segment.start().vel() / (segment.end().vel() - segment.start().vel());
                double t = segment.start().t() + interp * (segment.end().t() - segment.start().t());
                var transition_state = segment.start().extrapolate(t);
                new_segments.add(new MotionSegment(segment.start(), transition_state));
                new_segments.add(new MotionSegment(transition_state, segment.end()));
            }
        }
        return new MotionProfile(new_segments);
    }

    protected static MotionState shiftByVelocity(MotionState state, double velocity_shift, double start_time) {
        return new MotionState(state.t(), state.pos() + velocity_shift * (state.t() - start_time), state.vel() + velocity_shift, state.acc());
    }

    protected static MotionProfile generateProfileImpl(MotionProfileConstraints constraints,
                                                             MotionProfileGoal goal_state,
                                                             MotionState prev_state, boolean flipped) {
        double delta_pos = goal_state.pos() - prev_state.pos();
        if (delta_pos < 0.0 || (delta_pos == 0.0 && prev_state.vel() < 0.0)) {
            // For simplicity, we always assume the goal requires positive movement. If negative, we flip to solve, then
            // flip the solution.
            return generateFlippedProfile(constraints, goal_state, prev_state, flipped);
        }
        final double max_vel = flipped ? -constraints.min_vel() : constraints.max_vel();
        // Invariant from this point on: delta_pos >= 0.0
        // Clamp the start state to be valid.
        MotionState start_state = new MotionState(prev_state.t(), prev_state.pos(),
                Math.signum(prev_state.vel()) * Math.min(Math.abs(prev_state.vel()), max_vel),
                Math.signum(prev_state.acc()) * Math.min(Math.abs(prev_state.acc()), constraints.max_abs_acc()));
        MotionProfile profile = new MotionProfile();
        profile.reset(start_state);
        // If our velocity is headed away from the goal, the first thing we need to do is to stop.
        if (start_state.vel() < 0.0 && delta_pos > 0.0) {
            final double stopping_time = Math.abs(start_state.vel() / constraints.max_abs_acc());
            profile.appendControl(constraints.max_abs_acc(), stopping_time);
            start_state = profile.endState();
            delta_pos = goal_state.pos() - start_state.pos();
        }
        // Invariant from this point on: start_state.vel() >= 0.0
        final double min_abs_vel_at_goal_sqr = start_state.vel2() - 2.0 * constraints.max_abs_acc() * delta_pos;
        final double min_abs_vel_at_goal = Math.sqrt(Math.abs(min_abs_vel_at_goal_sqr));
        final double max_abs_vel_at_goal = Math.sqrt(start_state.vel2() + 2.0 * constraints.max_abs_acc() * delta_pos);
        double goal_vel = goal_state.max_vel();
        double max_acc = constraints.max_abs_acc();
        if (min_abs_vel_at_goal_sqr > 0.0
                && min_abs_vel_at_goal > (goal_state.max_vel() + goal_state.vel_tolerance())) {
            // Overshoot is unavoidable with the current constraints. Look at completion_behavior to see what we should
            // do.
            if (goal_state.completion_behavior() == CompletionBehavior.VIOLATE_MAX_ABS_VEL) {
                // Adjust the goal velocity.
                goal_vel = min_abs_vel_at_goal;
            } else if (goal_state.completion_behavior() == CompletionBehavior.VIOLATE_MAX_ACCEL) {
                if (Math.abs(delta_pos) < goal_state.pos_tolerance()) {
                    // Special case: We are at the goal but moving too fast. This requires 'infinite' acceleration,
                    // which will result in NaNs below, so we can return the profile immediately.
                    profile.appendSegment(new MotionSegment(
                            new MotionState(profile.endTime(), profile.endPos(), profile.endState().vel(),
                                    Double.NEGATIVE_INFINITY),
                            new MotionState(profile.endTime(), profile.endPos(), goal_vel, Double.NEGATIVE_INFINITY)));
                    profile.consolidate();
                    return profile;
                }
                // Adjust the max acceleration.
                max_acc = Math.abs(goal_vel * goal_vel - start_state.vel2()) / (2.0 * delta_pos);
            } else {
                // We are going to overshoot the goal, so the first thing we need to do is come to a stop.
                final double stopping_time = Math.abs(start_state.vel() / constraints.max_abs_acc());
                profile.appendControl(-constraints.max_abs_acc(), stopping_time);
                // Now we need to travel backwards, so generate a flipped profile.
                profile.appendProfile(generateFlippedProfile(constraints, goal_state, profile.endState(), flipped));
                profile.consolidate();
                return profile;
            }
        }
        goal_vel = Math.min(goal_vel, max_abs_vel_at_goal);
        // Invariant from this point forward: We can achieve goal_vel at goal_state.pos exactly using no more than +/-
        // max_acc.

        // What is the maximum velocity we can reach (Vmax)? This is the intersection of two curves: one accelerating
        // towards the goal from profile.finalState(), the other coming from the goal at max vel (in reverse). If Vmax
        // is greater than constraints.max_abs_vel, we will clamp and cruise.
        // Solve the following three equations to find Vmax (by substitution):
        // Vmax^2 = Vstart^2 + 2*a*d_accel
        // Vgoal^2 = Vmax^2 - 2*a*d_decel
        // delta_pos = d_accel + d_decel
        final double v_max = Math.min(max_vel,
                Math.sqrt((start_state.vel2() + goal_vel * goal_vel) / 2.0 + delta_pos * max_acc));

        // Accelerate to v_max
        if (v_max > start_state.vel()) {
            final double accel_time = (v_max - start_state.vel()) / max_acc;
            profile.appendControl(max_acc, accel_time);
            start_state = profile.endState();
        }
        // Figure out how much distance will be covered during deceleration.
        final double distance_decel = Math.max(0.0,
                (start_state.vel2() - goal_vel * goal_vel) / (2.0 * constraints.max_abs_acc()));
        final double distance_cruise = Math.max(0.0, goal_state.pos() - start_state.pos() - distance_decel);
        // Cruise at constant velocity.
        if (distance_cruise > 0.0) {
            final double cruise_time = distance_cruise / start_state.vel();
            profile.appendControl(0.0, cruise_time);
            start_state = profile.endState();
        }
        // Decelerate to goal velocity.
        if (distance_decel > 0.0) {
            final double decel_time = (start_state.vel() - goal_vel) / max_acc;
            profile.appendControl(-max_acc, decel_time);
        }

        profile.consolidate();
        return profile;
    }
}
