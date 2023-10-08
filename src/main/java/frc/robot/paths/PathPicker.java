package frc.robot.paths;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.Auton.Path;

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
                return null;
            default:
                return DoNothing.getTrajectory();
        }
    }
}
