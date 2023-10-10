// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class VisionPoseUpdateCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  VisionSubsystem visionSubsystem;
  SwerveSubsystem swerveSubsystem;

  public VisionPoseUpdateCommand(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d currentPose3d;
    Pose2d currentPose2d;
    currentPose3d = visionSubsystem.getPose();
    currentPose2d = currentPose3d.toPose2d();
    if(currentPose2d == null){
        return;
    }
    swerveSubsystem.resetOdometry(currentPose2d);
    swerveSubsystem.resetDriveMotorEncoders();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
