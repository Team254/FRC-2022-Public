package com.team254.lib.trajectory;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.State;

public class TrajectoryIterator<S extends State<S>, T extends State<T>> {
    protected final TrajectoryView<S, T> view_;
    protected double progress_ = 0.0;
    protected TrajectorySamplePoint<S, T> current_sample_;

    public TrajectoryIterator(final TrajectoryView<S, T> view) {
        view_ = view;

        // No effect if view is empty.
        current_sample_ = view_.sample(view_.first_interpolant());
        progress_ = view_.first_interpolant();
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getProgress() {
        return progress_;
    }

    public double getRemainingProgress() {
        return Math.max(0.0, view_.last_interpolant() - progress_);
    }

    public TrajectorySamplePoint<S, T> getSample() {
        return current_sample_;
    }

    public S getState() {
        return getSample().state();
    }

    public T getHeading() {
        return getSample().heading();
    }

    public TrajectorySamplePoint<S, T> advance(double additional_progress) {
        progress_ = Math.max(view_.first_interpolant(),
                Math.min(view_.last_interpolant(), progress_ + additional_progress));
        current_sample_ = view_.sample(progress_);
        return current_sample_;
    }

    public TrajectorySamplePoint<S, T> preview(double additional_progress) {
        final double progress = Math.max(view_.first_interpolant(),
                Math.min(view_.last_interpolant(), progress_ + additional_progress));
        return view_.sample(progress);
    }

    public Trajectory<S, T> trajectory() {
        return view_.trajectory();
    }
}
