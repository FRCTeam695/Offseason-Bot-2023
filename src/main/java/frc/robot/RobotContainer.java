// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.intakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final intakeSubsystem m_IntakeSubsystem = new intakeSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final XboxController controller = new XboxController(0);

  private final JoystickButton leftBumper = new JoystickButton(controller, 5);
  private final JoystickButton rightBumper = new JoystickButton(controller, 6);
  private final JoystickButton a_Button = new JoystickButton(controller, 1);
  private final JoystickButton b_Button = new JoystickButton(controller, 2);
  private final JoystickButton x_Button = new JoystickButton(controller,3);
  private final JoystickButton y_Button = new JoystickButton(controller,4);
  private final JoystickButton back_Button = new JoystickButton(controller, 7);

  //private final POVButton pov_Up = new POVButton(controller, 0);
  //private final POVButton pov_Down = new POVButton(controller, 180);

  private final DoubleSupplier left_xAxis = () -> (controller.getRawAxis(0));
  private final DoubleSupplier left_yAxis = () -> (controller.getRawAxis(1));
  private final DoubleSupplier right_xAxis = () -> (controller.getRawAxis(4));

  private int chargeStationState = 1;
  
  private PIDController xController = new PIDController(1, 0, 0);
  private PIDController yController = new PIDController(1, 0, 0);
  private ProfiledPIDController thetaController = new ProfiledPIDController(Constants.SummerSwerve.PROFILED_KP_VALUE, 0, 0, Constants.SummerSwerve.TRAPEZOID_THETA_CONSTRAINTS);

  SendableChooser<Command> m_pathChooser = new SendableChooser<>();

  public RobotContainer() {
    thetaController.enableContinuousInput(-Math.PI, -Math.PI);
    //m_pathChooser.setDefaultOption("Score Preload", scorePreload());

    m_pathChooser.setDefaultOption("Cube/Balance/Left", fullAutonCommand(1));
    m_pathChooser.addOption("Cube/Balance/Right", fullAutonCommand(2));

    SmartDashboard.putData("Auton Routine", m_pathChooser);

    configureBindings();
    instantCommands();
    defaultCommands();
  }

  private void configureBindings() {
//    leftBumper.whileTrue(new intakeCommand(m_IntakeSubsystem, -0.25)); // intake the cube
    rightBumper.whileTrue(new intakeCommand(m_IntakeSubsystem, 0.05)); //outtake the cube, lower level
    a_Button.whileTrue(new intakeCommand(m_IntakeSubsystem, 0.95)); //blast outtake the cube, high level
    b_Button.whileTrue(new intakeCommand(m_IntakeSubsystem, 0.25)); //outtake, mid level
  }

  private void instantCommands() {
    
    leftBumper.whileTrue(
        new FunctionalCommand(
  
          // init
          ()-> 
          {
            m_ArmSubsystem.setLevel(1);
          },
  
          // execute
          ()-> 
          {
            m_IntakeSubsystem.runIntake(-0.25);
          },
  
          // end
          interrupted-> 
          {
            m_ArmSubsystem.setLevel(2);
            m_IntakeSubsystem.runIntake(0);
          },
  
          // end condition
          ()-> false));
    

    back_Button.onTrue(new InstantCommand(()-> {swerveSubsystem.zeroHeading();}, swerveSubsystem));

    
    x_Button.onTrue(new InstantCommand(()-> {m_ArmSubsystem.setLevel(2);}, m_ArmSubsystem));
    
    y_Button.onTrue(new InstantCommand(()-> {m_ArmSubsystem.setLevel(1);}, m_ArmSubsystem));

  }

  private void defaultCommands() {
    swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(swerveSubsystem, left_xAxis, left_yAxis, right_xAxis, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    swerveSubsystem.setRelativeTurnEncoderValue();
    //return moveLeft();
    //return scorePreload().andThen(moveBackwards()).andThen(moveLeft());
    //return scorePreload().andThen(initialAutonSequence(moveBackwards(), moveLeft())).andThen(chargeStationBalance());
    //return engageChargeStation();
    return m_pathChooser.getSelected();
    //return chargeStationBalance();
  }

  public Command fullAutonCommand(int auton){
    SmartDashboard.putNumber("You picked auton number ", auton);
    if(auton == 1){
      return scorePreload().andThen(initialAutonSequence(substationMoveBackwardsRed(), substationMoveLeftRed())).andThen(chargeStationBalance(1));
    }
    return scorePreload().andThen(initialAutonSequence(bumpMoveBackwardsRed(), bumpMoveRightRed())).andThen(chargeStationBalance(2));
  }

  public Command chargeStationBalance(int auton){
    //if(auton == 1){
      //return new ParallelRaceGroup(new SwerveControllerCommand(substationMoveForwardRed(), swerveSubsystem::getPose, Constants.SummerSwerve.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModules, swerveSubsystem), 
      //engageChargeStation());
    //}
    return new ParallelRaceGroup(new SwerveControllerCommand(bumpMoveForwardRed(), swerveSubsystem::getPose, Constants.SummerSwerve.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModules, swerveSubsystem), 
    engageChargeStation());
  }

  public Command scorePreload()
  {
    return new InstantCommand(()-> {m_ArmSubsystem.setLevel(2);}, m_ArmSubsystem)
    .andThen(new WaitCommand(1))
    .andThen(new intakeCommand(m_IntakeSubsystem, 0.95))
    .andThen(new WaitCommand(1))
    .andThen(new intakeCommand(m_IntakeSubsystem, 0));
  }


  public Trajectory substationMoveForwardRed(){
    double endPoint = Units.inchesToMeters(30);
    
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.SummerSwerve.MAX_SPEED_METERS_PER_SECONDS * 0.2, Constants.SummerSwerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED).setKinematics(Constants.SummerSwerve.kDriveKinematics);
    Trajectory trajectory =
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(80), new Rotation2d(0)),
      List.of(new Translation2d(Units.inchesToMeters(130), Units.inchesToMeters(80)), new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(80))),
      new Pose2d(endPoint, Units.inchesToMeters(80), new Rotation2d(0)),
      trajectoryConfig);

    return trajectory;
  }


  public Trajectory bumpMoveForwardRed(){
    double endPoint = Units.inchesToMeters(30);
    
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.SummerSwerve.MAX_SPEED_METERS_PER_SECONDS * 0.2, Constants.SummerSwerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED).setKinematics(Constants.SummerSwerve.kDriveKinematics);
    Trajectory trajectory =
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(-80), new Rotation2d(0)),
      List.of(new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(-80)), new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(-80))),
      new Pose2d(endPoint, Units.inchesToMeters(-80), new Rotation2d(0)),
      trajectoryConfig);

    return trajectory;
  }

  public Trajectory substationMoveBackwardsRed(){
    double endPoint = Units.inchesToMeters(160);
    
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.SummerSwerve.MAX_SPEED_METERS_PER_SECONDS * 0.5, Constants.SummerSwerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED).setKinematics(Constants.SummerSwerve.kDriveKinematics);
    Trajectory trajectory =
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(endPoint/4, 0), new Translation2d(2 * endPoint/4, 0)),
      new Pose2d(endPoint, 0, new Rotation2d(0)),
      trajectoryConfig);

    return trajectory;
  }

  public Trajectory bumpMoveBackwardsRed(){
    double endPoint = Units.inchesToMeters(160);
    
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.SummerSwerve.MAX_SPEED_METERS_PER_SECONDS * 0.5, Constants.SummerSwerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED).setKinematics(Constants.SummerSwerve.kDriveKinematics);
    Trajectory trajectory =
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(Units.inchesToMeters(50), 0.33), new Translation2d(Units.inchesToMeters(100), 0.33)),
      new Pose2d(endPoint, 0, new Rotation2d(0)),
      trajectoryConfig);

    return trajectory;
  }

  public Trajectory substationMoveLeftRed(){
    double endPoint = Units.inchesToMeters(80);
  
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.SummerSwerve.MAX_SPEED_METERS_PER_SECONDS * 0.5, Constants.SummerSwerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED).setKinematics(Constants.SummerSwerve.kDriveKinematics);
    Trajectory trajectory =
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(Units.inchesToMeters(160), 0, new Rotation2d(0)),
      List.of(new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(30)), new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(60))),
      new Pose2d(Units.inchesToMeters(160), endPoint, new Rotation2d(0)),
      trajectoryConfig);

    return trajectory;
  }

  public Trajectory bumpMoveRightRed(){
    double endPoint = Units.inchesToMeters(-80);
  
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.SummerSwerve.MAX_SPEED_METERS_PER_SECONDS * 0.5, Constants.SummerSwerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED).setKinematics(Constants.SummerSwerve.kDriveKinematics);
    Trajectory trajectory =
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(Units.inchesToMeters(160), 0, new Rotation2d(0)),
      List.of(new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(-30)), new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(-60))),
      new Pose2d(Units.inchesToMeters(160), endPoint, new Rotation2d(0)),
      trajectoryConfig);

    return trajectory;
  }

  public Command initialAutonSequence(Trajectory trajectory1, Trajectory trajectory2){
    var trajectoryConcat = trajectory1.concatenate(trajectory2);
    var swerveControllerCommand = new SwerveControllerCommand(trajectoryConcat, swerveSubsystem::getPose, Constants.SummerSwerve.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModules, swerveSubsystem);

    swerveSubsystem.resetOdometry(trajectory1.getInitialPose());

    return swerveControllerCommand;//.andThen(() -> swerveSubsystem.stopModules());
  }

  public Command engageChargeStation(){
    return new FunctionalCommand(

    // init
    ()-> 
    {
      swerveSubsystem.startTickCount();
      //swerveSubsystem.driveSwerve(0, 0, -0.5, true);
      //-15, -5
    },

    // execute
    ()-> 
    {
        if(swerveSubsystem.getPitch() <= -15){
          chargeStationState = 2;
        }
        if(swerveSubsystem.getPitch() >= -7 && chargeStationState == 2){
          chargeStationState = 3;
        }
        //swerveSubsystem.driveSwerve(0, 0, 0.5, false);
    },

    // end
    interrupted-> 
    {
      swerveSubsystem.driveSwerve(0, 0, 0, true);
    },

    // end condition
    ()-> chargeStationState == 3);  //if we have balanced
  }
}
