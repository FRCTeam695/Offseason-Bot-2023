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
import frc.robot.paths.exampleTrajectory;
import frc.robot.paths.trajectoryPicker;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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

  private final POVButton pov_Up = new POVButton(controller, 0);
  private final POVButton pov_Down = new POVButton(controller, 180);

  private final DoubleSupplier left_xAxis = () -> (controller.getRawAxis(0));
  private final DoubleSupplier left_yAxis = () -> (controller.getRawAxis(1));
  private final DoubleSupplier right_xAxis = () -> (controller.getRawAxis(4));

  //private final ProfiledPIDController thetaController;
  //private final PIDController xController;
  //private final PIDController yController;
  private final SwerveDriveKinematics driveKinematics;
  private final double maxSpeedMPS;
  private final double maxAccelMPS;

  public RobotContainer() {

    driveKinematics = Constants.SummerSwerve.kDriveKinematics;
    maxSpeedMPS = Constants.SummerSwerve.MAX_SPEED_METERS_PER_SECONDS;
    maxAccelMPS = Constants.SummerSwerve.MAX_ACCELERATION_METERS_PER_SECOND;
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
    /*
        y_Button.onTrue(
          new FunctionalCommand(
    
            // init
            ()-> 
            {
            },
    
            // execute
            ()-> 
            {
                m_ArmSubsystem.runArm(-1);
            },
    
            // end
            interrupted-> 
            {
              m_ArmSubsystem.runArm(0);
            },
    
            // end condition
            ()-> m_ArmSubsystem.getArmPosition() >= 0.94));
        */
      
        /*
      pov_Up.whileTrue(
      new FunctionalCommand(

        // init
        ()-> 
        {
        },

        // execute
        ()-> 
        {
          System.out.println(m_IntakeSubsystem.getArmPosition());
          if (m_IntakeSubsystem.getArmPosition() > 0.5)
          {
            m_IntakeSubsystem.runArm(0.25);
          }
        },

        // end
        interrupted-> 
        {
          m_IntakeSubsystem.runArm(0.0);
        },

        // end condition
        ()-> false));
    

    pov_Down.whileTrue(
      new FunctionalCommand(

        // init
        ()-> 
        {
        },

        // execute
        ()-> 
        {
          System.out.println(m_IntakeSubsystem.getArmPosition());
          if (m_IntakeSubsystem.getArmPosition() < 0.9)
          {
            m_IntakeSubsystem.runArm(-0.25);
          }
        },

        // end
        interrupted-> 
        {
          m_IntakeSubsystem.runArm(0.0);
        },

        // end condition
        ()-> false));
      */
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
    /*
    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    var thetaController = new ProfiledPIDController(Constants.SummerSwerve.THETA_KP_VALUE, 0, 0, Constants.SummerSwerve.TRAPEZOID_THETA_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, -Math.PI);

    
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(maxSpeedMPS * 0.8, maxAccelMPS).setKinematics(driveKinematics);
    Trajectory trajectory =
    TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 1, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(0, 2), new Translation2d(0, 3)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(0, 2, new Rotation2d(0)),
      trajectoryConfig);
      
    var swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose, driveKinematics, xController, yController, thetaController, swerveSubsystem::setModules, swerveSubsystem);


    swerveSubsystem.setOdometer(new SwerveModulePosition[] {swerveSubsystem.getModulePosition(1),swerveSubsystem.getModulePosition(2), swerveSubsystem.getModulePosition(3), swerveSubsystem.getModulePosition(4)}, new Pose2d(0, 0, new Rotation2d(0)));
    swerveSubsystem.setRelativeTurnEncoderValue();
    return swerveControllerCommand.andThen(() -> swerveSubsystem.stopModules());
    */
    return scorePreload();
  }

  public Command scorePreload()
  {
    return new InstantCommand(()-> {m_ArmSubsystem.setLevel(2);}, m_ArmSubsystem)
      .andThen(new WaitCommand(1))
//    .andThen(new InstantCommand(()-> {m_ArmSubsystem.setLevel(2);}, m_ArmSubsystem))
  //  .andThen(new InstantCommand(()-> {new WaitCommand(1);}))
  //  //.andThen(new InstantCommand(()-> {m_IntakeSubsystem.testing();}, m_IntakeSubsystem))
    .andThen(new intakeCommand(m_IntakeSubsystem, 0.95))
  .andThen(new WaitCommand(1))
  .andThen(new InstantCommand(()-> {m_IntakeSubsystem.testing();}, m_IntakeSubsystem))
  .andThen(new intakeCommand(m_IntakeSubsystem, 0))
    ;
  }
}
