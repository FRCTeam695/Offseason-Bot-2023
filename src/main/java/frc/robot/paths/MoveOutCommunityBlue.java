package frc.robot.paths;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;


public class MoveOutCommunityBlue {
    
    public static Trajectory getTrajectory(){
        double endPoint = Units.inchesToMeters(160);
    
        TrajectoryConfig trajectoryConfig1 = new TrajectoryConfig(Constants.SummerSwerve.MAX_SPEED_METERS_PER_SECONDS * 0.3, Constants.SummerSwerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED).setKinematics(Constants.SummerSwerve.kDriveKinematics);
        Trajectory trajectory1 =
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(Units.inchesToMeters(50), 0.3)),
          new Pose2d(endPoint, Units.inchesToMeters(160), new Rotation2d(0)),
          trajectoryConfig1);
        
          return trajectory1;
    }
}
