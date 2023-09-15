// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.intakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class intakeCommand extends CommandBase {

  private final intakeSubsystem subsystem;  
  private double speed;

  public intakeCommand(intakeSubsystem subsystem, double speed) {
    this.subsystem = subsystem;
    this.speed = speed * 0.25;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.runIntake(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subsystem.endCommand();
  }
}
