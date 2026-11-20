package org.team1540.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public abstract class VisionIO {

    public final String name;

    public VisionIO(String name) {
        this.name = name;
    }

    @AutoLog
    public static class VisionIOInputs {
        public PoseObservation[] poseObs;
        public SingleTagObservation[] singleTagObs;
        public int[] seenTags;
    }

    public record PoseObservation(
            Pose3d poseMeters, int numTagsSeen, double avgTagDistance, double timeStampSecs, double ambiguity) {}

    public record SingleTagObservation(
            int id, Transform3d cameraPosition, Rotation2d yaw, Rotation2d pitch, double distanceMeters) {}

    public void updateInputs(VisionIOInputs inputs) {}
}
