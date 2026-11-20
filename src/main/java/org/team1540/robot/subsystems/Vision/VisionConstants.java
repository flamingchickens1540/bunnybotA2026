package org.team1540.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    public static final String F_CAMERA_NAME = "FrontCamera";
    public static final String B_CAMERA_NAME = "BackCamera";

    public static final Transform3d F_CAMERA_TRANSFORM = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));

    public static final Transform3d B_CAMERA_TRANSFORM = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
}
