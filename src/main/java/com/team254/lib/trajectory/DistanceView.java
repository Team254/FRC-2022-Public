package com.team254.lib.trajectory;

import com.team254.lib.geometry.State;
import com.team254.lib.util.Util;

public class DistanceView<S extends State<S>, T extends State<T>> implements TrajectoryView<S, T> {
    protected final Trajectory<S, T> trajectory_;
    protected final double[] distances_;

    public DistanceView(final Trajectory<S, T> trajectory) {
        trajectory_ = trajectory;
        distances_ = new double[trajectory_.length()];
        distances_[0] = 0.0;
        for (int i = 1; i < trajectory_.length(); ++i) {
            distances_[i] = distances_[i - 1] + trajectory_.getPoint(i - 1).state().distance(trajectory_.getPoint(i).state());
        }
    }

    @Override
    public TrajectorySamplePoint<S, T> sample(double distance) {
        if (distance >= last_interpolant())
            return new TrajectorySamplePoint<>(trajectory_.getPoint(trajectory_.length() - 1));
        if (distance <= 0.0)
            return new TrajectorySamplePoint<>(trajectory_.getPoint(0));
        for (int i = 1; i < distances_.length; ++i) {
            final TrajectoryPoint<S, T> s = trajectory_.getPoint(i);
            if (distances_[i] >= distance) {
                final TrajectoryPoint<S, T> prev_s = trajectory_.getPoint(i - 1);
                if (Util.epsilonEquals(distances_[i], distances_[i - 1])) {
                    return new TrajectorySamplePoint<>(s);
                } else {
                    return new TrajectorySamplePoint<>(
                            prev_s.state().interpolate(s.state(),
                                    (distance - distances_[i - 1]) / (distances_[i] - distances_[i - 1])),
                            prev_s.heading().interpolate(s.heading(),
                                    (distance - distances_[i - 1]) / (distances_[i] - distances_[i - 1])), i - 1, i);
                }
            }
        }
        throw new RuntimeException();
    }

    @Override
    public double last_interpolant() {
        return distances_[distances_.length - 1];
    }

    @Override
    public double first_interpolant() {
        return 0.0;
    }

    @Override
    public Trajectory<S, T> trajectory() {
        return trajectory_;
    }
}
