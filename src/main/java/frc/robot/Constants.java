package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class Intake{
        public static final double stallTimeout = 0.5;
        public static final int motor1ID = 9;
        public static final int motor2ID = 8;
    }

    public static final class FlintSwerve{
        public static final double MAX_SPEED_METERS_PER_SECONDS = 4.572;
        public static final double MAX_ANGULAR_SPEED_METERS_PER_SECOND = 2 * Math.PI; // Physical max speed is 4 * PI
        public static final double TURNING_GEAR_RATIO = 150.0/7;
        public static final double DRIVING_GEAR_RATIO = 6.12;
        public static final double WHEEL_CIRCUMFERENCE_METERS = 4 * Math.PI;
        public static final double TRACK_WIDTH = Units.inchesToMeters(24.5);
        public static final double WHEEL_BASE = Units.inchesToMeters(21.25);
    
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                    //Positive x values represent moving towards the front of the robot, positive y values represent moving to the left
                    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), //Front right wheel
                    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), //Front left wheel
                    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), //Back left wheel
                    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); //Back right wheel
    }

    public static final class SummerSwerve{
        public static final double MAX_SPEED_METERS_PER_SECONDS = 4.572;
        public static final double MAX_ANGULAR_SPEED_METERS_PER_SECOND = 2 * Math.PI; // Physical max speed is 4 * PI
        public static final double TURNING_GEAR_RATIO = 150.0/7;
        public static final double DRIVING_GEAR_RATIO = 6.12;
        public static final double WHEEL_CIRCUMFERENCE_METERS = 4 * Math.PI;
        public static final double TRACK_WIDTH = Units.inchesToMeters(19);
        public static final double WHEEL_BASE = Units.inchesToMeters(19);
    
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                    //Positive x values represent moving towards the front of the robot, positive y values represent moving to the left
                    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), //Front right wheel
                    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), //Front left wheel
                    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), //Back left wheel
                    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); //Back right wheel
    }
}
