package com.team254.lib.trajectory;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import java.util.Arrays;
import java.util.List;

@RunWith(JUnit4.class)
public class TrajectoryIteratorTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    public static final List<Translation2d> kWaypoints = Arrays.asList(
            new Translation2d(0.0, 0.0),
            new Translation2d(24.0, 0.0),
            new Translation2d(36.0, 12.0),
            new Translation2d(60.0, 12.0));

    List<Rotation2d> kHeadings = Arrays.asList(
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(30),
            Rotation2d.fromDegrees(60),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(180));

    @Test
    public void test() {
        Trajectory<Translation2d, Rotation2d> traj = new Trajectory<>(kWaypoints, kHeadings);
        TrajectoryIterator<Translation2d, Rotation2d> iterator = new TrajectoryIterator<>(traj.getIndexView());

        // Initial conditions.
        Assert.assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        Assert.assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        Assert.assertEquals(kWaypoints.get(0), iterator.getState());
        Assert.assertEquals(kHeadings.get(0), iterator.getHeading());
        Assert.assertFalse(iterator.isDone());

        // Advance forward.
        Assert.assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.5), iterator.preview(0.5).state());
        Assert.assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), 0.5), iterator.preview(0.5).heading());
        TrajectorySamplePoint<Translation2d, Rotation2d> newPoint = iterator.advance(0.5);
        Assert.assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.5), newPoint.state());
        Assert.assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), 0.5), newPoint.heading());
        Assert.assertEquals(0.5, iterator.getProgress(), kTestEpsilon);
        Assert.assertEquals(2.5, iterator.getRemainingProgress(), kTestEpsilon);
        Assert.assertFalse(iterator.isDone());

        // Advance backwards.
        Assert.assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.25), iterator.preview(-0.25).state());
        Assert.assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), 0.25), iterator.preview(-0.25).heading());
        newPoint = iterator.advance(-0.25);
        Assert.assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.25), newPoint.state());
        Assert.assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), 0.25), newPoint.heading());
        Assert.assertEquals(0.25, iterator.getProgress(), kTestEpsilon);
        Assert.assertEquals(2.75, iterator.getRemainingProgress(), kTestEpsilon);
        Assert.assertFalse(iterator.isDone());

        // Advance past end.
        Assert.assertEquals(kWaypoints.get(3), iterator.preview(5.0).state());
        Assert.assertEquals(kHeadings.get(3), iterator.preview(5.0).heading());
        newPoint = iterator.advance(5.0);
        Assert.assertEquals(kWaypoints.get(3), newPoint.state());
        Assert.assertEquals(kHeadings.get(3), newPoint.heading());
        Assert.assertEquals(3.0, iterator.getProgress(), kTestEpsilon);
        Assert.assertEquals(0.0, iterator.getRemainingProgress(), kTestEpsilon);
        Assert.assertTrue(iterator.isDone());

        // Advance past beginning.
        Assert.assertEquals(kWaypoints.get(0), iterator.preview(-5.0).state());
        Assert.assertEquals(kHeadings.get(0), iterator.preview(-5.0).heading());
        newPoint = iterator.advance(-5.0);
        Assert.assertEquals(kWaypoints.get(0), newPoint.state());
        Assert.assertEquals(kHeadings.get(0), newPoint.heading());
        Assert.assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        Assert.assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        Assert.assertFalse(iterator.isDone());
    }

}
