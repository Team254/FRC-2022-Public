package com.team254.lib.geometry;

public interface IRotation2d<S> extends State<S> {
    Rotation2d getRotation();

    S rotateBy(Rotation2d other);

    S mirror();
}
