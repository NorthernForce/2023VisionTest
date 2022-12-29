// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    // Targeting constants
    public static final String CAMERA_ID = "webcam";
    //drive constants
    public static final int LEFT_PRIMARY_ID = 2;
    public static final int RIGHT_PRIMARY_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_FOLLOWER_ID = 3;

    public static final double DRIVE_RAMP_RATE = 0.2;

    public static final double ROTATION_SPEED_PROPORTION = 0.75;
    public static final double SPEED_PROPORTION = 1;

    public static final int kUnitsPerRevolution = 2048;
    public static final double wheelDiameter = 0.05;
    public static final double distancePerRevolution = (wheelDiameter * Math.PI);

    public static final Pose2d targetPose = new Pose2d(5, 0, Rotation2d.fromDegrees(0));
}
