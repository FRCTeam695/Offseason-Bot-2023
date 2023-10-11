package frc.robot.paths;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.Paths.Path;

public class PathPicker {

    public Trajectory getTrajectory(Path path){
        switch (path){
            case SUBSTATION_RED:
                return SubstationSideRed.getTrajectory();
            case BUMP_RED:
                return BumpSideRed.getTrajectory();
            case MOVE_OUT_COMMUNITY_RED:
                return MoveOutCommunityRed.getTrajectory();
            case MOVE_TO_CHARGE_STATION:
                return MoveToChargeStation.getTrajectory();
            case MOVE_OUT_COMMUNITY_BLUE:
                return DoNothing.getTrajectory(); //Maybe finish this later
            default:
                return DoNothing.getTrajectory();
        }
    }

    public Trajectory getTrajectory(Path path, Translation2d cubeTranslation, Pose2d robotPose){
        return GoToCube.getTrajectory(cubeTranslation, robotPose);
    }
}
