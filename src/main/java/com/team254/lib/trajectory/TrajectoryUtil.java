package com.team254.lib.trajectory;

import com.team254.lib.geometry.*;
import com.team254.lib.spline.QuinticHermiteSpline;
import com.team254.lib.spline.Spline;
import com.team254.lib.spline.SplineGenerator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.Util;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryUtil {
    public static <S extends IPose2d<S>, T extends IRotation2d<T>> Trajectory<S, T> mirror(final Trajectory<S, T> trajectory) {
        List<S> waypoints = new ArrayList<>(trajectory.length());
        List<T> headings = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            waypoints.add(trajectory.getPoint(i).state().mirror());
            headings.add(trajectory.getPoint(i).heading().mirror());
        }
        return new Trajectory<>(waypoints, headings);
    }

    public static <S extends IPose2d<S>, T extends IRotation2d<T>> Trajectory<TimedState<S>, TimedState<T>> mirrorTimed(final Trajectory<TimedState<S>, TimedState<T>> trajectory) {    // todo fix
        List<TimedState<S>> waypoints = new ArrayList<>(trajectory.length());
        List<TimedState<T>> headings = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            TimedState<S> timed_state = trajectory.getPoint(i).state();
            waypoints.add(new TimedState<>(timed_state.state().mirror(), timed_state.t(), timed_state.velocity(), timed_state.acceleration()));
            headings.add(new TimedState<>(trajectory.getPoint(i).heading().state().mirror()));
        }
        return new Trajectory<>(waypoints, headings);
    }

    public static <S extends IPose2d<S>, T extends IRotation2d<T>> Trajectory<S, T> transform(final Trajectory<S, T> trajectory, Pose2d transform) {
        List<S> waypoints = new ArrayList<>(trajectory.length());
        List<T> headings = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            waypoints.add(trajectory.getPoint(i).state().transformBy(transform));
            headings.add(trajectory.getPoint(i).heading().rotateBy(transform.getRotation()));
        }
        return new Trajectory<>(waypoints, headings);
    }

    /**
     * Creates a Trajectory by sampling a TrajectoryView at a regular interval.
     *
     * @param trajectory_view
     * @param interval
     * @return
     */
    public static <S extends State<S>, T extends State<T>> Trajectory<S, T> resample(
            final TrajectoryView<S, T> trajectory_view, double interval) {
        if (interval <= Util.kEpsilon) {
            return new Trajectory<S, T>();
        }
        final int num_states = (int) Math
                .ceil((trajectory_view.last_interpolant() - trajectory_view.first_interpolant()) / interval);
        ArrayList<S> states = new ArrayList<S>(num_states);
        ArrayList<T> headings = new ArrayList<>(num_states);

        for (int i = 0; i < num_states; ++i) {
            states.add(trajectory_view.sample(i * interval + trajectory_view.first_interpolant()).state());
            headings.add(trajectory_view.sample(i * interval + trajectory_view.first_interpolant()).heading());
        }
        return new Trajectory<>(states, headings);
    }

    public static Trajectory<Pose2dWithCurvature, Rotation2d> trajectoryFromWaypoints(final List<Pose2d> waypoints, final List<Rotation2d> headings, double
            maxDx, double maxDy, double maxDTheta) {
        List<QuinticHermiteSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new QuinticHermiteSpline(waypoints.get(i - 1), waypoints.get(i)));
        }
        QuinticHermiteSpline.optimizeSpline(splines);
        return trajectoryFromSplinesAndHeadings(splines, headings, maxDx, maxDy, maxDTheta);
    }

    public static Trajectory<Pose2dWithCurvature, Rotation2d> trajectoryFromSplinesAndHeadings(final List<? extends Spline> splines, final List<Rotation2d> headings, double
            maxDx, double maxDy, double maxDTheta) {
        return new Trajectory<>(SplineGenerator.parameterizeSplines(splines, headings, maxDx, maxDy,
                maxDTheta));
    }
}
