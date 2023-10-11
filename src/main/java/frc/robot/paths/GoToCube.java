package frc.robot.paths;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

public class GoToCube {

    static Translation2d endpoint;
    static Pose2d startpoint;

    public static Trajectory getTrajectory(Translation2d cubeTranslation, Pose2d robotPose){
        endpoint = cubeTranslation;
        startpoint = robotPose;
        
        TrajectoryConfig trajectoryConfig1 = new TrajectoryConfig(Constants.SummerSwerve.MAX_SPEED_METERS_PER_SECONDS * 0.5, Constants.SummerSwerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED).setKinematics(Constants.SummerSwerve.kDriveKinematics);
        Trajectory trajectory1 =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                startpoint,
                new Pose2d(endpoint.getX(), endpoint.getY(), endpoint.getAngle())),
          trajectoryConfig1);
          
        return trajectory1;
    }
}
