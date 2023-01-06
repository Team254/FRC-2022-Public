package com.team254.lib.physics;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import java.text.DecimalFormat;

public class SwerveDrive {
    // All units must be SI!

    // Self-explanatory.  Measure by rolling the robot a known distance and counting encoder ticks.
    protected final double wheel_radius_;  // m

    // "Effective" kinematic wheelbase radius.  Might be larger than theoretical to compensate for skid steer.  Measure
    // by turning the robot in place several times and figuring out what the equivalent wheelbase radius is.
    protected final double effective_wheelbase_radius_;  // m

    public SwerveDrive(final double wheel_radius,
                       final double effective_wheelbase_radius) {
        wheel_radius_ = wheel_radius;
        effective_wheelbase_radius_ = effective_wheelbase_radius;
    }

    // Can refer to velocity or acceleration depending on context.
    public static class ChassisState {
        public Translation2d movement;
        public Rotation2d heading;

        public ChassisState(Translation2d movement, Rotation2d heading) {
            this.heading = heading;
            this.movement = movement;
        }

        public ChassisState(Translation2d movement) {
            this.movement = movement;
            this.heading = Rotation2d.identity();
        }


        public ChassisState() {
            this.movement = Translation2d.identity();
            this.heading = Rotation2d.identity();
        }

        @Override
        public String toString() {
            DecimalFormat fmt = new DecimalFormat("#0.000");
            return fmt.format(movement.norm())/* + ", " + fmt.format(heading.getRadians())*/;
        }
    }
}