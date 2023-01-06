package com.team254.lib.motion;

import com.team254.lib.motion.IMotionProfileGoal.CompletionBehavior;
import com.team254.lib.util.Util;
import org.hamcrest.BaseMatcher;
import org.hamcrest.Description;
import org.hamcrest.Matcher;
import org.junit.Test;

import static com.team254.lib.motion.MotionUtil.kEpsilon;
import static org.junit.Assert.assertThat;
import static org.junit.Assert.assertTrue;

public class MotionProfileGeneratorTest {

    public static Matcher<Double> epsilonEqualTo(final double b, final double epsilon) {
        return new BaseMatcher<Double>() {
            @Override
            public boolean matches(Object a) {
                return Util.epsilonEquals((Double) a, b, epsilon);
            }

            @Override
            public void describeTo(Description description) {
                description.appendText("Should be within +/- ").appendValue(epsilon).appendText(" of ")
                        .appendValue(b);
            }
        };
    }

    protected static void validateProfile(MotionProfileConstraints constraints, MotionProfileGoal goal,
                                          MotionState start, MotionProfile profile) {
        // Profile should be valid.
        assertTrue(profile.isValid());
        // Profile should start at the start state (clamped to accel and velocity
        // limits).
        assertThat(profile.startState().t(), epsilonEqualTo(start.t(), kEpsilon));
        assertThat(profile.startState().pos(), epsilonEqualTo(start.pos(), kEpsilon));
        assertThat(profile.startState().vel(),
                epsilonEqualTo(Math.max(Math.min(start.vel(), constraints.max_vel()), constraints.min_vel()),
                        kEpsilon));
        // Profile should not violate constraints.
        for (MotionSegment s : profile.segments()) {
            assertTrue(s.start().vel() <= constraints.max_vel());
            assertTrue(s.start().vel() >= constraints.min_vel());
            assertTrue(s.end().vel() <= constraints.max_vel());
            assertTrue(s.end().vel() >= constraints.min_vel());
            if (goal.completion_behavior() != CompletionBehavior.VIOLATE_MAX_ACCEL) {
                assertTrue(Math.abs(s.start().acc()) <= constraints.max_abs_acc());
                assertTrue(Math.abs(s.end().acc()) <= constraints.max_abs_acc());
            }
        }
        // Profile should end at the goal state.
        if (goal.completion_behavior() != CompletionBehavior.VIOLATE_MAX_ABS_VEL) {
            assertTrue(goal.atGoalState(profile.endState()));
        } else {
            assertTrue(goal.atGoalPos(profile.endPos()));
        }
    }
    protected static void validateProfile(MotionProfileConstraints constraints, LinearTimeVaryingMotionProfileGoal goal,
                                          MotionState start, MotionProfile profile, boolean ignore_goal) {
        // Profile should be valid.
        assertTrue(profile.isValid());
        // Profile should start at the start state (clamped to accel and velocity
        // limits).
        assertThat(profile.startState().t(), epsilonEqualTo(start.t(), kEpsilon));
        assertThat(profile.startState().pos(), epsilonEqualTo(start.pos(), kEpsilon));
        assertThat(profile.startState().vel(),
                epsilonEqualTo(Math.max(Math.min(start.vel(), constraints.max_vel()), constraints.min_vel()),
                        kEpsilon));
        // Profile should not violate constraints.
        for (MotionSegment s : profile.segments()) {
            assertTrue(s.start().vel() <= constraints.max_vel());
            assertTrue(s.start().vel() >= constraints.min_vel());
            assertTrue(s.end().vel() <= constraints.max_vel());
            assertTrue(s.end().vel() >= constraints.min_vel());
            if (goal.completion_behavior() != CompletionBehavior.VIOLATE_MAX_ACCEL) {
                assertTrue(Math.abs(s.start().acc()) <= constraints.max_abs_acc());
                assertTrue(Math.abs(s.end().acc()) <= constraints.max_abs_acc());
            }
        }
        // Profile should end at the goal state.
        if (!ignore_goal) {
            var goal_state_at_end_time = goal.pos(profile.endTime());
            assertTrue(goal.atGoalState(profile.endState()));
        }
    }

    protected static void testProfile(MotionProfileConstraints constraints, MotionProfileGoal goal,
                                      MotionState start, double expected_duration, double expected_length) {
        testProfileForward(constraints, goal, start, expected_duration, expected_length);
        testProfileFlipped(new MotionProfileConstraints(-constraints.min_vel(), -constraints.max_vel(), constraints.max_abs_acc()),
                goal, start, expected_duration, expected_length);
    }

    protected static void testProfileForward(MotionProfileConstraints constraints, MotionProfileGoal goal,
                                             MotionState start, double expected_duration, double expected_length) {
        MotionProfile positive_profile = MotionProfileGenerator.generateProfile(constraints, goal, start);
        assertThat(positive_profile.toString(), positive_profile.duration(),
                epsilonEqualTo(expected_duration, kEpsilon));
        assertThat(positive_profile.toString(), positive_profile.length(),
                epsilonEqualTo(expected_length, kEpsilon));
        validateProfile(constraints, goal, start, positive_profile);
    }

    protected static void testProfileFlipped(MotionProfileConstraints constraints, MotionProfileGoal goal,
                                             MotionState start, double expected_duration, double expected_length) {
        MotionProfile negative_profile = MotionProfileGenerator.generateProfile(constraints, goal.flipped(),
                start.flipped());
        assertThat(negative_profile.toString(), negative_profile.duration(),
                epsilonEqualTo(expected_duration, kEpsilon));
        assertThat(negative_profile.toString(), negative_profile.length(),
                epsilonEqualTo(expected_length, kEpsilon));
        validateProfile(constraints, goal.flipped(), start.flipped(), negative_profile);
    }

    protected static void testProfile(MotionProfileConstraints constraints, LinearTimeVaryingMotionProfileGoal goal,
                                      MotionState start, double expected_duration, double expected_length) {
        testProfileForward(constraints, goal, start, expected_duration, expected_length);
        testProfileFlipped(new MotionProfileConstraints(-constraints.min_vel(), -constraints.max_vel(), constraints.max_abs_acc()),
                goal, start, expected_duration, expected_length);
    }

    protected static void testProfileForward(MotionProfileConstraints constraints, LinearTimeVaryingMotionProfileGoal goal,
                                             MotionState start, double expected_duration, double expected_length) {
        MotionProfile positive_profile = MotionProfileGenerator.generateProfile(constraints, goal, start);
        assertThat(positive_profile.toString(), positive_profile.duration(),
                epsilonEqualTo(expected_duration, kEpsilon));
        assertThat(positive_profile.toString(), positive_profile.length(),
                epsilonEqualTo(expected_length, kEpsilon));
        validateProfile(constraints, goal, start, positive_profile, false);
    }

    protected static void testProfileFlipped(MotionProfileConstraints constraints, LinearTimeVaryingMotionProfileGoal goal,
                                             MotionState start, double expected_duration, double expected_length) {
        MotionProfile negative_profile = MotionProfileGenerator.generateProfile(constraints, goal.flipped(),
                start.flipped());
        assertThat(negative_profile.toString(), negative_profile.duration(),
                epsilonEqualTo(expected_duration, kEpsilon));
        assertThat(negative_profile.toString(), negative_profile.length(),
                epsilonEqualTo(expected_length, kEpsilon));
        validateProfile(constraints, goal.flipped(), start.flipped(), negative_profile, false);
    }

    @Test
    public void testAlreadyFinished() {
        // No initial velocity.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(0.0),
                new MotionState(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);
        // Initial velocity matches goal state.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(2.0, 5.0),
                new MotionState(0.0, 2.0, 5.0, 7.0), 0.0, 0.0);
        // Initial velocity is too high, so allow infinite accel.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0),
                new MotionProfileGoal(2.0, 5.0, CompletionBehavior.VIOLATE_MAX_ABS_VEL),
                new MotionState(0.0, 2.0, 9.0, 7.0), 0.0, 0.0);
        // Initial velocity is too high, so allow infinite accel.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0),
                new MotionProfileGoal(2.0, 5.0, CompletionBehavior.VIOLATE_MAX_ACCEL),
                new MotionState(0.0, 2.0, 9.0, 7.0), 0.0, 0.0);
    }

    @Test
    public void testStationaryToStationary() {
        // Trapezoidal move.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(100.0),
                new MotionState(0.0, 0.0, 0.0, 0.0), 11.0, 100.0);
        // Triangle move.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(10.0),
                new MotionState(0.0, 0.0, 0.0, 0.0), 2.0, 10.0);

        // Test asymmetric velocity limits.
        // Trapezoidal move.
        testProfile(new MotionProfileConstraints(10.0, -5.0, 5.0), new MotionProfileGoal(100.0),
                new MotionState(0.0, 0.0, 0.0, 0.0), 12.0, 100.0);

        // Triangle move.
        testProfile(new MotionProfileConstraints(10.0, -5.0, 5.0), new MotionProfileGoal(5.0),
                new MotionState(0.0, 0.0, 0.0, 0.0), 2.0, 5.0);
    }

    @Test
    public void testMovingTowardsToStationary() {
        // Moving towards goal, trapezoidal move.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(100.0),
                new MotionState(0.0, 0.0, 10.0, 0.0), 10.5, 100.0);
        // Moving towards goal, triangle move.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(10.0),
                new MotionState(0.0, 0.0, 5.0, 0.0), 1.625, 10.0);
        // Moving towards goal, cruise and stop.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(10.0),
                new MotionState(0.0, 0.0, 10.0, 0.0), 1.5, 10.0);
        // Moving towards goal, overshoot and come back.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(0.0),
                new MotionState(0.0, 0.0, 10.0, 0.0), 1.0 + Math.sqrt(2.0), 10.0);
        // Moving towards goal, violate max vel.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0),
                new MotionProfileGoal(1.0, 0.0, CompletionBehavior.VIOLATE_MAX_ABS_VEL),
                new MotionState(0.0, 0.0, 10.0, 0.0), (10.0 - Math.sqrt(80.0)) / 10.0, 1.0);
        // Moving towards goal, violate max accel.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0),
                new MotionProfileGoal(1.0, 0.0, CompletionBehavior.VIOLATE_MAX_ACCEL),
                new MotionState(0.0, 0.0, 10.0, 0.0), 1.0 / 5.0, 1.0);
    }

    @Test
    public void testMovingAwayToStationary() {
        // Moving away from goal, trapezoidal move.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(100.0),
                new MotionState(0.0, 0.0, -10.0, 0.0), 12.5, 110.0);
        // Moving away from goal, triangle move.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(5.0),
                new MotionState(0.0, 0.0, -10.0, 0.0), 3.0, 15.0);
    }

    @Test
    public void testStationaryToMoving() {
        // Accelerate and cruise.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(100.0, 10.0),
                new MotionState(0.0, 0.0, 0.0, 0.0), 10.5, 100.0);

        // Trapezoidal move.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(100.0, 5.0),
                new MotionState(0.0, 0.0, 0.0, 0.0), 10.625, 100.0);

        // Pure acceleration.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(1.0, 10.0),
                new MotionState(0.0, 0.0, 0.0, 0.0), Math.sqrt(1.0 / 5.0), 1.0);

        // Triangle move.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(8.75, 5.0),
                new MotionState(0.0, 0.0, 0.0, 0.0), 1.5, 8.75);
    }

    @Test
    public void testMovingTowardsToMoving() {
        // Moving towards goal, pure acceleration.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(3.75, 10.0),
                new MotionState(0.0, 0.0, 5.0, 0.0), 0.5, 3.75);
        // Moving towards goal, pure deceleration.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(10.0, 5.0),
                new MotionState(0.0, 0.0, 10.0, 0.0), 1.125, 10.0);
        // Moving towards goal, cruise.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(100.0, 10.0),
                new MotionState(0.0, 0.0, 10.0, 0.0), 10.0, 100.0);
        // Moving towards goal, accelerate and cruise.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(100.0, 10.0),
                new MotionState(0.0, 0.0, 5.0, 0.0), 10.125, 100.0);
        // Moving towards goal, trapezoidal move.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(100.0, 5.0),
                new MotionState(0.0, 0.0, 5.0, 0.0), 10.25, 100.0);
        // Moving towards goal, triangle move.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(1.0, 10.0),
                new MotionState(0.0, 0.0, 4.0, 0.0), 0.2, 1.0);
        // Moving towards goal, cruise and decelerate.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(10.0, 5.0),
                new MotionState(0.0, 0.0, 10.0, 0.0), 1.125, 10.0);
        // Moving towards goal, violate max vel.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0),
                new MotionProfileGoal(1.0, 1.0, CompletionBehavior.VIOLATE_MAX_ABS_VEL),
                new MotionState(0.0, 0.0, 10.0, 0.0), (10.0 - Math.sqrt(80.0)) / 10.0, 1.0);
        // Moving towards goal, violate max accel.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0),
                new MotionProfileGoal(1.0, 2.0, CompletionBehavior.VIOLATE_MAX_ACCEL),
                new MotionState(0.0, 0.0, 10.0, 0.0), 1.0 / 6.0, 1.0);
    }

    @Test
    public void testMovingAwayToMoving() {
        // Moving away from goal, stop and accelerate.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(1.0, 10.0),
                new MotionState(0.0, 0.0, -4.0, 0.0), 1.0, 2.6);
        // Moving away from goal, stop, accelerate, and cruise.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(100.0, 10.0),
                new MotionState(0.0, 0.0, -10.0, 0.0), 12.0, 110.0);
        // Moving away from goal, stop and trapezoid move.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(100.0, 5.0),
                new MotionState(0.0, 0.0, -10.0, 0.0), 12.125, 110.0);
        // Moving away from goal, stop and triangle move.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new MotionProfileGoal(8.75, 5.0),
                new MotionState(0.0, 5.0, -10.0, 0.0), 2.5, 13.75);
    }

    @Test
    public void problematicCase1() {
        MotionProfile profile = MotionProfileGenerator.generateProfile(new MotionProfileConstraints(50.0, -50.0, 25.0),
                new MotionProfileGoal(200.0), new MotionState(0.0, 0.0, 0.0, 0.0));
        System.out.println(profile);
        assertTrue(profile.firstStateByPos(160.0).get().vel() > 0.0);
    }

    @Test
    public void testAsymmetricVelocityLimits() {
        // Moving away from goal.
        testProfile(new MotionProfileConstraints(10.0, -5.0, 10.0),
                new MotionProfileGoal(10.0, 0.0),
                new MotionState(0.0, 0.0, -5.0, 0.0), 2.625, 12.5);

        // Moving towards goal.
        testProfile(new MotionProfileConstraints(10.0, -5.0, 10.0),
                new MotionProfileGoal(10.0, 0.0),
                new MotionState(0.0, 0.0, 5.0, 0.0), 1.625, 10.0);

        // Moving towards negative goal.
        testProfile(new MotionProfileConstraints(10.0, -5.0, 10.0),
                new MotionProfileGoal(-10.0, 0.0),
                new MotionState(0.0, 0.0, 5.0, 0.0), 3.25, 12.5);

        // Moving away from goal, stop and trapezoid move.
        testProfile(new MotionProfileConstraints(10.0, -5.0, 10.0), new MotionProfileGoal(100.0, 5.0),
                new MotionState(0.0, 0.0, -5.0, 0.0), 11.25, 102.5);
    }

    @Test
    public void testLinearTimeVaryingGoal() {
        // No goal velocity.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new LinearTimeVaryingMotionProfileGoal(0.0, 100.0, 0.0),
                new MotionState(0.0, 0.0, 0.0, 0.0), 11.0, 100.0);

        // Goal moving away.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new LinearTimeVaryingMotionProfileGoal(0.0, 10.0, 5.0),
                new MotionState(0.0, 0.0, 0.0, 0.0), 3.25, 26.25);

        // Goal moving towards us.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new LinearTimeVaryingMotionProfileGoal(0.0, 10.0, -5.0),
                new MotionState(0.0, 0.0, 0.0, 0.0), 1.621320, 4.393398);

        // Goal moving towards us, we are moving towards goal.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new LinearTimeVaryingMotionProfileGoal(0.0, 10.0, -5.0),
            new MotionState(0.0, 0.0, 2.0, 0.0), 1.5315914, 4.842043);

        // Goal moving towards us, we are moving towards goal really fast.
        testProfile(new MotionProfileConstraints(10.0, -10.0, 10.0), new LinearTimeVaryingMotionProfileGoal(0.0, 10.0, -5.0),
            new MotionState(0.0, 0.0, 10.0, 0.0), 2.207107, 11.0355339);

        // Goal moving away faster than we can move.
        {
            var constraints = new MotionProfileConstraints(10.0, -10.0, 10.0);
            var goal = new LinearTimeVaryingMotionProfileGoal(0.0, 10.0, 10.0);
            var start = new MotionState(0.0, 0.0, 0.0, 0.0);
            var profile = MotionProfileGenerator.generateProfile(constraints, goal, start);
            validateProfile(constraints, goal, start, profile, true);
            goal = new LinearTimeVaryingMotionProfileGoal(0.0, -10.0, -10.0);
            profile = MotionProfileGenerator.generateProfile(constraints, goal, start);
            validateProfile(constraints, goal, start, profile, true);
        }
    }
}