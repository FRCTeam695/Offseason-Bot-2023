package frc.robot.paths;

import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;

public class exampleTrajectory {

  //Don't worry about this for now, code not in use
    private final SwerveDriveKinematics driveKinematics;
    private final double maxSpeedMPS;
    private final double maxAccelerationMPS;

    public exampleTrajectory(){
        driveKinematics = Constants.SummerSwerve.kDriveKinematics;
        maxSpeedMPS = Constants.SummerSwerve.MAX_SPEED_METERS_PER_SECONDS;
        maxAccelerationMPS = Constants.SummerSwerve.MAX_ACCELERATION_METERS_PER_SECOND;
    }
    
    public Trajectory generateTrajectory(){
        Pose2d startingPoint = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d endingPoint = new Pose2d(2, 0, Rotation2d.fromDegrees(145));

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(1, 0));
        interiorWaypoints.add(new Translation2d(1, -1));

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(maxSpeedMPS * 0.5, maxAccelerationMPS).setKinematics(driveKinematics);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startingPoint, interiorWaypoints, endingPoint, trajectoryConfig);

        return trajectory;
    }
}
