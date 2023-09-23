package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

public class intakeSubsystem extends SubsystemBase{

    //init motors
    private CANSparkMax intakeMotor1 = new CANSparkMax(Constants.Intake.MOTOR_1_ID, MotorType.kBrushless);
    private CANSparkMax intakeMotor2 = new CANSparkMax(Constants.Intake.MOTOR_2_ID, MotorType.kBrushless);
    //private CANSparkMax armMotor = new CANSparkMax(53, MotorType.kBrushless);

    //get encoders
    private RelativeEncoder encoder1 = intakeMotor1.getEncoder();
    private RelativeEncoder encoder2 = intakeMotor2.getEncoder();

    // arm encoder
    //private DigitalInput armDI = new DigitalInput(9);
    //private DutyCycleEncoder armEncoder = new DutyCycleEncoder(armDI);

    private Timer stallTimer = new Timer();
    private double lastPosition1;
    private double lastPosition2;

    private double runspeed = 0;

    private boolean stallHold;
    private boolean running;


    public intakeSubsystem(){
        intakeMotor1.restoreFactoryDefaults();
        intakeMotor1.setIdleMode(IdleMode.kBrake);
        intakeMotor1.setSmartCurrentLimit(50);

        intakeMotor2.restoreFactoryDefaults();
        intakeMotor2.setIdleMode(IdleMode.kBrake);
        intakeMotor2.setSmartCurrentLimit(50);

        //armMotor.restoreFactoryDefaults();
        //armMotor.setIdleMode(IdleMode.kBrake);
        //armMotor.setSmartCurrentLimit(50);

        encoder1.setPosition(0);
        encoder2.setPosition(0);

        lastPosition1 = encoder1.getPosition();
        lastPosition2 = encoder2.getPosition();

        stallTimer.reset();
        stallTimer.start();

        stallHold = false;
        running = false;
    }

    //-1 for intake 1 for outtake
    public void runIntake(double speed){

        if (getStallHold()){
            runspeed = 0;
            setSpeed(runspeed);
        }

        else{
            setSpeed(speed);
        }
    }

    public void runArm(double speed)
    {
        //armMotor.set(speed);
    }

    private void setSpeed(double speed) {
        runspeed = MathUtil.clamp(speed, -1, 1);
        setMotors(runspeed);
    }

    public void testing(){
        System.out.println("Running intake");
    }

    private void setMotors(double speed){
        running = true;
        intakeMotor1.set(speed);
        intakeMotor2.set(speed * -1); //this one has to move backwards to intake a cube/cone
    }

    private boolean getStallHold(){
        return stallHold;
    }
    
    public void stop(){
        setMotors(0);
        running = false;
        stallHold = false;
    }

    //used for ending the command
    public boolean endCommand(){

        if(getStallHold()){
            System.out.println("Stalled");
            return true;
        }

        return false;
    }

    public double getArmPosition()
    {
        return 1;
        //return(armEncoder.getAbsolutePosition());
    }

    @Override
    public void periodic(){
        if (running) {
            
            double currentPosition1 = encoder1.getPosition();
            double currentPosition2 = encoder2.getPosition();

            if (stallTimer.get() > Constants.Intake.stallTimeout) {
                if (Math.abs(currentPosition1 - lastPosition1) < 2 || Math.abs(currentPosition2 - lastPosition2) < 2) {
                    intakeMotor1.setSmartCurrentLimit(3);
                    intakeMotor2.setSmartCurrentLimit(3);
                    stallHold = true;
                }

                else{
                    lastPosition1 = currentPosition1;
                    lastPosition2 = currentPosition2;
                }

            stallTimer.reset();
            stallTimer.start();

            } else {
                intakeMotor1.setSmartCurrentLimit(50);
                intakeMotor2.setSmartCurrentLimit(50);
                stallHold = false;
            }

        } else {
            stallTimer.reset();
            stallHold = false;
        }
    }
    
}
