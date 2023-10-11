package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PoseWrapper;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera camera;
    PhotonPipelineResult result;
    Field2d m_field = new Field2d();
    PhotonPoseEstimator photonPoseEstimator;
    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d cameraToRobot;

    public VisionSubsystem() {
        camera = new PhotonCamera(Constants.Vision.CAMERA_NAME);
        camera.setLED(VisionLEDMode.kOff);
        SmartDashboard.putData("Field", m_field);
        cameraToRobot = new Transform3d(new Translation3d(Constants.Vision.CAMERA_TO_ROBOT_OFFSET_FORWARD, 0.0,
                Constants.Vision.CAMERA_HEIGHT_METERS), new Rotation3d(0.0, 0.0, 0.0));
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
            SmartDashboard.putString("field layout load error", e.toString());
        }

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, camera,
                cameraToRobot);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    public Double findCube() {
        camera.setPipelineIndex(0);
        result = camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();

            // m_field.getObject("cube").setPose(
            // new Pose2d(
            // PhotonUtils.estimateCameraToTargetTranslation(
            // PhotonUtils.calculateDistanceToTargetMeters(
            // Constants.Constantsq.CAMERA_HEIGHT_METERS,
            // 0,
            // 0,
            // 0),
            // new Rotation2d(target.getYaw())
            // ).plus(getPose().toPose2d().getTranslation()),
            // new Rotation2d(0.0,0.0)
            // )
            // );

            return target.getYaw();
        }

        return null;
    }

    public Translation2d getCubeLocation(){
        return new Translation2d();
    }

    public Pose3d getPose() {
        // SmartDashboard.putString("grr", "yayaya");
        camera.setPipelineIndex(1);
        PhotonPipelineResult tresult;
        // https://github.wpilib.org/allwpilib/docs/beta/java/edu/wpi/first/apriltag/AprilTagFieldLayout.html

        EstimatedRobotPose estimatedPose;
        Pose3d robotPose;

        tresult = camera.getLatestResult();

        if (tresult.hasTargets()) {
            SmartDashboard.putBoolean("targets?", true);
            // hasTargets.set(true);

            // PhotonTrackedTarget target = vision.getBestTarget();

            // SmartDashboard.putString("grr2", "yayaya");

            // SmartDashboard.putString("grr3", "yayaya");
            // camera.getLatestResult();

            var thingy = photonPoseEstimator.update(tresult);
            try {
                estimatedPose = thingy.get();
                robotPose = estimatedPose.estimatedPose;
            } catch (Exception e) {
                estimatedPose = null;
                robotPose = null;
                return null;
            }

            // estimatedPose = photonPoseEstimator.update(result).get();
            // robotPose = estimatedPose.estimatedPose;

            try {

                // time.set(estimatedPose.timestampSeconds);
                List<PhotonTrackedTarget> targetsUsed = estimatedPose.targetsUsed;

                // numTargets.set(targetsUsed.size());

                m_field.setRobotPose(robotPose.toPose2d());
                // return (new PoseWrapper(targetsUsed.size(), robotPose,
                // estimatedPose.timestampSeconds));

                SmartDashboard.putString("robotPose", robotPose.toString());

                return robotPose;

            } catch (Exception e) {
                SmartDashboard.putString("robotPose", "none");
                m_field.setRobotPose(new Pose2d());
            }
        } else {
            SmartDashboard.putBoolean("targets?", false);
            SmartDashboard.putString("robotPose", "none");
            m_field.setRobotPose(new Pose2d());
        }

        return null;

    }
}
