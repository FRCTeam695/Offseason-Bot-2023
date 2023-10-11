// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.intakeCommand;
import frc.robot.commands.VisionPoseUpdateCommand;
import frc.robot.paths.PathPicker;
import frc.robot.Constants.Paths.Path;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
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
  public int alliance;
  
  private PIDController xController = new PIDController(1, 0, 0);
  private PIDController yController = new PIDController(1, 0, 0);
  private ProfiledPIDController thetaController = new ProfiledPIDController(Constants.SummerSwerve.PROFILED_KP_VALUE, 0, 0, Constants.SummerSwerve.TRAPEZOID_THETA_CONSTRAINTS);

  private final PathPicker pathPicker = new PathPicker();

  SendableChooser<Command> m_pathChooser = new SendableChooser<>();

  public RobotContainer() {
    thetaController.enableContinuousInput(-Math.PI, -Math.PI);
    //m_pathChooser.setDefaultOption("Score Preload", scorePreload());

    m_pathChooser.setDefaultOption("Cube/Balance/Substation", substationSideCommandWrapper());
    m_pathChooser.addOption("Cube/Balance/Bump", bumpSideCommandWrapper());
    m_pathChooser.addOption("Score Preload", scorePreload());
    m_pathChooser.addOption("Score/Leave", getAutonomousCommand());

    SmartDashboard.putData("Auton Routine", m_pathChooser);

    xController.reset();
    yController.reset();
    thetaController.reset(null);

    configureBindings();
    instantCommands();
    defaultCommands();
  }

  private void configureBindings() {
    leftBumper.whileTrue(new intakeCommand(m_IntakeSubsystem, m_ArmSubsystem,-0.25)); // intake the cube
    rightBumper.whileTrue(new intakeCommand(m_IntakeSubsystem, m_ArmSubsystem, 0.15)); //outtake the cube, lower level
    a_Button.whileTrue(new intakeCommand(m_IntakeSubsystem, m_ArmSubsystem, 0.95)); //blast outtake the cube, high level
    b_Button.whileTrue(new intakeCommand(m_IntakeSubsystem, m_ArmSubsystem, 0.25)); //outtake, mid level 
    //new ParallelRaceGroup(goToCubeCommand(), new intakeCommand(m_IntakeSubsystem, m_ArmSubsystem, -0.25));  //GET THIS WORKING WITH NETWORKTABLE BUTTON
  }

  private void instantCommands() {
    back_Button.onTrue(new InstantCommand(()-> {swerveSubsystem.zeroHeading();}, swerveSubsystem));
    x_Button.onTrue(new InstantCommand(()-> {m_ArmSubsystem.setLevel(1);}, m_ArmSubsystem));
    y_Button.onTrue(new InstantCommand(()-> {m_ArmSubsystem.setLevel(2);}, m_ArmSubsystem));
  }

  private void defaultCommands() {
    m_VisionSubsystem.setDefaultCommand(new VisionPoseUpdateCommand(m_VisionSubsystem, swerveSubsystem));
    swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(swerveSubsystem, left_xAxis, left_yAxis, right_xAxis, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command goToCubeCommand() {
    Trajectory trajectory = pathPicker.getTrajectory(Path.MOVE_TO_CUBE, m_VisionSubsystem.getCubeLocation(), swerveSubsystem.getPose());

    var swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose, Constants.SummerSwerve.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModules, swerveSubsystem);

    return new InstantCommand(()-> {swerveSubsystem.resetOdometry(trajectory.getInitialPose());}, swerveSubsystem).andThen(swerveControllerCommand).andThen(() -> swerveSubsystem.stopModules());
  }
  
  public Command getAutonomousCommand() {
    swerveSubsystem.setRelativeTurnEncoderValue();
    return m_pathChooser.getSelected();
  }

  public Command scoreLeaveCommunity(){
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      return scorePreload().andThen(moveOutCommunityRed());
    }
    else{
      return scorePreload().andThen(moveOutCommunityBlue());
    }
  }

  public Command substationSideCommandWrapper(){
    return substationSideCommand();
  }

  public Command bumpSideCommandWrapper(){
    return bumpSideCommand();
  }

  public Command substationSideCommand(){
    System.out.println(DriverStation.getAlliance());
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      return scorePreload()
      .andThen(new InstantCommand(()-> {m_IntakeSubsystem.testing();}, m_IntakeSubsystem))
      .andThen(new InstantCommand(()-> {m_IntakeSubsystem.testingSendableChooser();}, m_IntakeSubsystem))
      .andThen(substationSideRed()) //Gets to position infront of the charge station
      .andThen(new ParallelRaceGroup(moveToChargeStation(), engageChargeStation()));
    }else{
      return scorePreload()
    .andThen(new InstantCommand(()-> {m_IntakeSubsystem.testing2();}, m_IntakeSubsystem))
    .andThen(new InstantCommand(()-> {m_IntakeSubsystem.testingSendableChooser();}, m_IntakeSubsystem))
    .andThen(bumpSideRed()) //Gets to position infront of the charge station
    .andThen(new ParallelRaceGroup(moveToChargeStation(), engageChargeStation()));
    }
  }

  public Command bumpSideCommand(){
    System.out.println(DriverStation.getAlliance());

    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
      return scorePreload()
      .andThen(new InstantCommand(()-> {m_IntakeSubsystem.testing();}, m_IntakeSubsystem))
      .andThen(new InstantCommand(()-> {m_IntakeSubsystem.testingSendableChooser2();}, m_IntakeSubsystem))
      .andThen(substationSideRed()) //Gets to position infront of the charge station
      .andThen(new ParallelRaceGroup(moveToChargeStation(), engageChargeStation()));
    }
    else{
      return scorePreload()
    .andThen(new InstantCommand(()-> {m_IntakeSubsystem.testing2();}, m_IntakeSubsystem))
    .andThen(new InstantCommand(()-> {m_IntakeSubsystem.testingSendableChooser2();}, m_IntakeSubsystem))
    .andThen(bumpSideRed()) //Gets to position infront of the charge station
    .andThen(new ParallelRaceGroup(moveToChargeStation(), engageChargeStation()));
    }
  }

  public Command scorePreload()
  {
    return new InstantCommand(()-> {m_ArmSubsystem.setLevel(2);}, m_ArmSubsystem)
    .andThen(new WaitCommand(1))
    .andThen(new intakeCommand(m_IntakeSubsystem, m_ArmSubsystem, 0.95))
    .andThen(new WaitCommand(1))
    .andThen(new intakeCommand(m_IntakeSubsystem, m_ArmSubsystem, 0));
  }

  public Command substationSideRed(){
    
    Trajectory trajectory1 = pathPicker.getTrajectory(Path.SUBSTATION_RED);

    var swerveControllerCommand = new SwerveControllerCommand(trajectory1, swerveSubsystem::getPose, Constants.SummerSwerve.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModules, swerveSubsystem);
  
    return new InstantCommand(()-> {swerveSubsystem.resetOdometry(trajectory1.getInitialPose());}, swerveSubsystem).andThen(swerveControllerCommand).andThen(() -> swerveSubsystem.stopModules());
  }

  public Command moveToChargeStation(){
    Trajectory trajectory2 = pathPicker.getTrajectory(Path.MOVE_TO_CHARGE_STATION);

    var swerveControllerCommand = new SwerveControllerCommand(trajectory2, swerveSubsystem::getPose, Constants.SummerSwerve.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModules, swerveSubsystem);
  
    return swerveControllerCommand.andThen(() -> swerveSubsystem.stopModules());
  }

  public Command bumpSideRed(){
    Trajectory trajectory3 = pathPicker.getTrajectory(Path.BUMP_RED); //Get trajectory takes a translation2d bcs the cube vision needs it

    var swerveControllerCommand = new SwerveControllerCommand(trajectory3, swerveSubsystem::getPose, Constants.SummerSwerve.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModules, swerveSubsystem);
  
    return new InstantCommand(()-> {swerveSubsystem.resetOdometry(trajectory3.getInitialPose());}, swerveSubsystem).andThen(swerveControllerCommand).andThen(() -> swerveSubsystem.stopModules());
  }

  public Command moveOutCommunityRed(){
    
    Trajectory trajectory1 = pathPicker.getTrajectory(Path.MOVE_OUT_COMMUNITY_RED); //Get trajectory takes a translation2d bcs the cube vision needs it

    var swerveControllerCommand = new SwerveControllerCommand(trajectory1, swerveSubsystem::getPose, Constants.SummerSwerve.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModules, swerveSubsystem);
  
    return new InstantCommand(()-> {swerveSubsystem.resetOdometry(trajectory1.getInitialPose());}, swerveSubsystem).andThen(swerveControllerCommand).andThen(() -> swerveSubsystem.stopModules());
  }

  public Command moveOutCommunityBlue(){
    Trajectory trajectory1 = pathPicker.getTrajectory(Path.MOVE_OUT_COMMUNITY_BLUE); //Get trajectory takes a translation2d bcs the cube vision needs it

    var swerveControllerCommand = new SwerveControllerCommand(trajectory1, swerveSubsystem::getPose, Constants.SummerSwerve.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModules, swerveSubsystem);
  
    return new InstantCommand(()-> {swerveSubsystem.resetOdometry(trajectory1.getInitialPose());}, swerveSubsystem).andThen(swerveControllerCommand).andThen(() -> swerveSubsystem.stopModules());
  }
  

  public Command engageChargeStation(){
    //Constantly checks to see if we have balanced
    return new FunctionalCommand(

    // init
    ()-> 
    {
      swerveSubsystem.startTickCount();
    },

    // execute
    ()-> 
    {
        if(swerveSubsystem.getPitch() <= -15){
          chargeStationState = 2;
        }
        if(swerveSubsystem.getPitch() >= -10 && chargeStationState == 2){
          chargeStationState = 3;
        }
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
