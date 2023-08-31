package frc.robot.subsystems;

//import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;

public class SwerveSubsystem extends SubsystemBase{


    //initialize all the swerve modules
    //private final SwerveModule frontRight = new SwerveModule(13, 12, true, 158, 11);
    //private final SwerveModule frontLeft = new SwerveModule(23, 22, true, 222, 21);
    //private final SwerveModule bottomLeft = new SwerveModule(33, 32, true, 247, 31);
    //private final SwerveModule bottomRight = new SwerveModule(43, 42, false, 29, 41);


    private final SwerveModule frontRight = new SwerveModule(13, 12, true, 169, 11);
    private final SwerveModule frontLeft = new SwerveModule(23, 22, true, 49, 21);
    private final SwerveModule bottomLeft = new SwerveModule(33, 32, true, 353, 31);
    private final SwerveModule bottomRight = new SwerveModule(43, 42, true, 26, 41);

    //creates the odometry class
    SwerveDriveOdometry odometry;
    private final double maxSpeedMPS;


    //initialize the gyro
    private final AHRS gyro = new AHRS(SPI.Port.kMXP); 


    public SwerveSubsystem() {
        if (Robot.m_SwerveChooser.getSelected() == Robot.originalSwerve){
            SwerveDriveKinematics driveKinematics = Constants.FlintSwerve.kDriveKinematics;
            odometry = new SwerveDriveOdometry(driveKinematics, new Rotation2d(), new SwerveModulePosition[] 
    {frontRight.getPosition(), frontLeft.getPosition(), bottomLeft.getPosition(), bottomRight.getPosition()});
            maxSpeedMPS = Constants.FlintSwerve.MAX_SPEED_METERS_PER_SECONDS;
            
        }
        else{
            SwerveDriveKinematics driveKinematics = Constants.SummerSwerve.kDriveKinematics;
            odometry = new SwerveDriveOdometry(driveKinematics, new Rotation2d(), new SwerveModulePosition[] 
    {frontRight.getPosition(), frontLeft.getPosition(), bottomLeft.getPosition(), bottomRight.getPosition()});
            maxSpeedMPS = Constants.SummerSwerve.MAX_SPEED_METERS_PER_SECONDS;
        }

        //resets the gyro, it is calibrating when this code is reached so we reset it on a different thread with a delay
        new Thread(() -> {
            try {
                Thread.sleep(500);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        return -1 * Math.IEEEremainder(gyro.getAngle(), 360);  //Multiply by negative one because on wpilib as you go counterclockwise angles should get bigger
    }

    public void setModules(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeedMPS);
        frontRight.setDesiredState(desiredStates[0], 1);
        frontLeft.setDesiredState(desiredStates[1], 2);
        bottomLeft.setDesiredState(desiredStates[2], 3);
        bottomRight.setDesiredState(desiredStates[3], 4);
    }

    public double getAbsoluteEncoderValue(int motor) {
        switch(motor){
            case 1: return frontRight.getAbsoluteEncoderRadians();
            case 2: return frontLeft.getAbsoluteEncoderRadians();
            case 3: return bottomLeft.getAbsoluteEncoderRadians();
            case 4: return bottomRight.getAbsoluteEncoderRadians();
            default: return -1;
        }
    }

    public double getRelativeTurnEncoderValue(int motor) {
        switch(motor){
            case 1: return frontRight.getTurnPosition(false);
            case 2: return frontLeft.getTurnPosition(false);
            case 3: return bottomLeft.getTurnPosition(false);
            case 4: return bottomRight.getTurnPosition(false);
            default: return -1;
        }
    }

    public void setRelativeTurnEncoderValue(int motor){

        //Uses the absolute encoder value to set relative encoders
        switch(motor){
            case 1: frontRight.setTurnEncoder(motor, frontRight.getAbsoluteEncoderRadians());break;
            case 2: frontLeft.setTurnEncoder(motor, frontLeft.getAbsoluteEncoderRadians());break;
            case 3: bottomLeft.setTurnEncoder(motor, bottomLeft.getAbsoluteEncoderRadians());break;
            case 4: bottomRight.setTurnEncoder(motor, bottomRight.getAbsoluteEncoderRadians());break;
            default: ;
        }
    }

    @Override
    public void periodic(){
        odometry.update(new Rotation2d(getHeading() * Math.PI / 180), new SwerveModulePosition[] 
        {frontRight.getPosition(), frontLeft.getPosition(), bottomLeft.getPosition(), bottomRight.getPosition()});
    }

    public void setOdometer(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose){
        odometry.resetPosition(gyroAngle, modulePositions, pose);
    }
}
