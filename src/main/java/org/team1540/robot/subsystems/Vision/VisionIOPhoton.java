package org.team1540.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team1540.robot.FieldConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

public class VisionIOPhoton extends VisionIO {

    final PhotonCamera camera;
    final Transform3d cameraTransform;
    final PhotonPoseEstimator poseEstimator;

    private Set<Integer> lastSeenTagIds;

    public VisionIOPhoton(String name, Transform3d cameraTransform) {
        super(name);
        camera = new PhotonCamera(name);
        this.cameraTransform = cameraTransform;
        poseEstimator = new PhotonPoseEstimator(
                FieldConstants.aprilTagLayout,
                cameraTransform
        );

    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        ArrayList<PoseObservation> poseObs = new ArrayList<>();
        ArrayList<SingleTagObservation> singleTagObs = new ArrayList<>();

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> estimatedResult = poseEstimator.estimateCoprocMultiTagPose(result);
            if (estimatedResult.isPresent()) {
                EstimatedRobotPose estimatedRobotPose = estimatedResult.get();

                double totalDist = 0;
                double totalAmbiguity = 0;
                List<PhotonTrackedTarget> targets = estimatedRobotPose.targetsUsed;
                for (PhotonTrackedTarget target : targets) {
                    totalDist+=target.getBestCameraToTarget().getTranslation().getNorm();
                    totalAmbiguity+=target.poseAmbiguity;
                }
                poseObs.add(new PoseObservation(
                        estimatedRobotPose.estimatedPose,
                        targets.size(),
                        totalDist/targets.size(),
                        estimatedRobotPose.timestampSeconds,
                        totalAmbiguity
                ));
            }

            for (PhotonTrackedTarget target : result.getTargets()) {
                singleTagObs.add(new SingleTagObservation(
                        target.getFiducialId(),
                        target.getBestCameraToTarget(),
                        Rotation2d.fromDegrees(target.getYaw()),
                        Rotation2d.fromDegrees(target.getPitch()),
                        target.getBestCameraToTarget().getTranslation().getNorm()
                ));
                lastSeenTagIds.add(target.getFiducialId());
            }
        }
        inputs.singleTagObs = (SingleTagObservation[])singleTagObs.toArray();
        inputs.poseObs = (PoseObservation[])poseObs.toArray();
        inputs.seenTags = lastSeenTagIds.stream().mapToInt(Integer::intValue).toArray();
    }
}
