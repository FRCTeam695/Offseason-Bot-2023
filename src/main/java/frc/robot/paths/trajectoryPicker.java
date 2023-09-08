package frc.robot.paths;
import frc.robot.Robot;

import edu.wpi.first.math.trajectory.Trajectory;

public class trajectoryPicker {

    //Don't worry about this for now, code not in use
    public static Trajectory pickTrajectory(){
        if(Robot.m_TrajectoryChooser.getSelected() == Robot.exampleTrajectory){
            exampleTrajectory trajectoryGenerator = new exampleTrajectory();
            return trajectoryGenerator.generateTrajectory();
        }
        else{
            exampleTrajectory trajectoryGenerator = new exampleTrajectory();
            return trajectoryGenerator.generateTrajectory();
        }
    }
}
