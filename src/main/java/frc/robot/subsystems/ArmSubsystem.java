// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax armMotor = new CANSparkMax(Constants.Arm.ARM_MOTOR_ID, MotorType.kBrushless);
    private RelativeEncoder armEncoder = armMotor.getEncoder();
    private PIDController armController = new PIDController(0.7, 0, 0);
    private final double[] TICK_LEVELS = {Constants.Arm.LEVEL_1_TICKS, Constants.Arm.LEVEL_2_TICKS, Constants.Arm.LEVEL_3_TICKS};
    private int newLevel;
    private double setpoint;
    private double currentState;
    private double pos;

    public ArmSubsystem() {
        armMotor.restoreFactoryDefaults();
        armMotor.setIdleMode(IdleMode.kBrake);

        armEncoder.setPosition(0);
    }

    public int getLevel() {
        pos = armEncoder.getPosition();
        for(int i = 2; i > 0; i--){
            if (pos >= (TICK_LEVELS[i] - 1000)){
                return i;
            }
        }
        return 0;
    }

    public void setLevel(int level){
        newLevel = level;
    }

    @Override
    public void periodic(){
        
        if(newLevel == getLevel()){
            return;
        }

        

    }

}