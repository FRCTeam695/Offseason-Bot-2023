// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final SwerveSubsystem m_Subsystem;
  private final DoubleSupplier xSpeed, ySpeed, turningSpeed;
  private final boolean fieldOriented;
  private double previousAngle; //-180 to 180
  private double previousZj; // -1 to 1


  public SwerveDriveCommand(SwerveSubsystem subsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier turningSpeed, boolean fieldOriented) {
    this.m_Subsystem = subsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.turningSpeed = turningSpeed;
    this.fieldOriented = fieldOriented;
    previousZj = turningSpeed.getAsDouble();

    m_Subsystem.startTickCount();

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousAngle = m_Subsystem.getHeading();
    m_Subsystem.setRelativeTurnEncoderValue();  //Uses the absolute encoder value to set the relative encoders
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(DriverStation.isAutonomous()){
      return;
    }
    
    //Gets values from double suppliers
    Double Xj = -1 * xSpeed.getAsDouble(); //Inverted because WPIlib coordinate system is weird, link to docs below
    Double Yj = -1 * ySpeed.getAsDouble(); //The controller is inverted
    Double Zj = -1 * turningSpeed.getAsDouble(); //Inverted because WPIlib coordinate system is weird, link to docs below

    //Calculate difference between expected turn and real-time turn
    double diff = (previousZj - Zj) * 180 - (previousAngle - m_Subsystem.getHeading()) ;
    double turningOffset = MathUtil.clamp(diff, -180, 180);
    double zRes = Zj + turningOffset;

    SmartDashboard.putNumber("X", Xj);
    SmartDashboard.putNumber("Y", Yj);
    SmartDashboard.putNumber("Z", Zj);
    SmartDashboard.putNumber("Difference", diff);
    SmartDashboard.putNumber("Turn offset", turningOffset);
    SmartDashboard.putNumber("zRes", zRes);
    
    SmartDashboard.putNumber("Ticks", m_Subsystem.getTicks());

    m_Subsystem.driveSwerve(Xj, zRes, Yj, fieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
