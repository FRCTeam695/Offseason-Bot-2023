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

public class BumpSideRed {

    public static Trajectory getTrajectory() {
        double endPoint = Units.inchesToMeters(160);

        TrajectoryConfig trajectoryConfig3 = new TrajectoryConfig(
                Constants.SummerSwerve.MAX_SPEED_METERS_PER_SECONDS * 0.5,
                Constants.SummerSwerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED)
                .setKinematics(Constants.SummerSwerve.kDriveKinematics);
        Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, Units.inchesToMeters(140), new Rotation2d(0)),
                List.of(new Translation2d(Units.inchesToMeters(50), Units.inchesToMeters(140) + 0.3),
                        new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(140))),
                new Pose2d(endPoint, Units.inchesToMeters(70), new Rotation2d(0)),
                trajectoryConfig3);

        return trajectory3;
    }

}
