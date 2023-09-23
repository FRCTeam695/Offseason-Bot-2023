// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax armMotor = new CANSparkMax(Constants.Arm.ARM_MOTOR_ID, MotorType.kBrushless);
    // arm encoder
    private DigitalInput armDI = new DigitalInput(9);
    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(armDI);
    private PIDController armController = new PIDController(7.5, 0, 0);

    private int newLevel;
    private double setpoint = 0;
    private double currentState;
    private double output;

    public ArmSubsystem() {
        armMotor.restoreFactoryDefaults();
        armMotor.setIdleMode(IdleMode.kBrake);

    }
/*
    public int getLevel() {
        pos = armEncoder.getPosition();
        for(int i = 2; i > 0; i--){
            if (pos >= (TICK_LEVELS[i] - 1000)){
                return i;
            }
        }
        return 0;
    }
*/

    public void runArm(double speed)
    {
        armMotor.set(speed);
    }

    public void setLevel(int level){
        newLevel = level;
    }

    public double getArmPosition()
    {
        return(armEncoder.getAbsolutePosition());
    }

    @Override
    public void periodic(){
        if(newLevel == 2){
            setpoint = 0.57;
            currentState = getArmPosition();
            output = -1 * armController.calculate(currentState, setpoint);
            runArm(MathUtil.clamp(output, -1, 1));
        }
        
        if(newLevel == 1){
            setpoint = 0.94;
            currentState = getArmPosition();
            output = -1 * armController.calculate(currentState, setpoint);
            runArm(MathUtil.clamp(output, -1, 1));
        }
        /*
        if(newLevel == 2){
            if(getArmPosition() >= 0.57)
        {
          runArm(0.25);
        }
            if (getArmPosition() <= 0.56)
        {
          runArm(-0.25);
        }
        }
        */
    }

}