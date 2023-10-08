package frc.robot.paths;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
public class MoveToChargeStation {
    
    public static Trajectory getTrajectory(){
        double endPoint = Units.inchesToMeters(30);
    
        TrajectoryConfig trajectoryConfig2 = new TrajectoryConfig(Constants.SummerSwerve.MAX_SPEED_METERS_PER_SECONDS * 0.2, Constants.SummerSwerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED).setKinematics(Constants.SummerSwerve.kDriveKinematics);
        trajectoryConfig2.setReversed(true);
        Trajectory trajectory2 =
        TrajectoryGenerator.generateTrajectory(
          List.of(
          new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(70), new Rotation2d(0)),
          new Pose2d(endPoint, Units.inchesToMeters(70), new Rotation2d(0))),
          trajectoryConfig2);
        
          return trajectory2;
    }
}
