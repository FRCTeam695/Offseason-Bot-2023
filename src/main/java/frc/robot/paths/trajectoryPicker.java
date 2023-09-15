package frc.robot.paths;

import edu.wpi.first.math.trajectory.Trajectory;

public class trajectoryPicker {

    public static Trajectory pickTrajectory(){
        exampleTrajectory trajectoryGenerator = new exampleTrajectory();
        return trajectoryGenerator.generateTrajectory();
    }
}
