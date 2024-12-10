package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ChoreoSubsystem;

public class ChoreoWheelTestAuto extends SequentialCommandGroup{

    
public ChoreoWheelTestAuto(Trajectories trajectories) {

    super(
        
        
        new SequentialCommandGroup(
            new InstantCommand(() -> AlignmentTransitions.scheduleShoot()),
            new WaitCommand(0.6),
            new AutoPickupNote().alongWith(ChoreoSubsystem.getInstance().setupAutonomousChoreoPath(trajectories.traj)).withTimeout(5),
            new InstantCommand(() -> AlignmentTransitions.scheduleShoot()),
            new AutoPickupNote().alongWith(ChoreoSubsystem.getInstance().setupAutonomousChoreoPath(trajectories.traj2))
        )
        


    );
}

public static class Trajectories {
    public ChoreoTrajectory traj;
    public ChoreoTrajectory traj2;

    public Trajectories() {
        traj = Choreo.getTrajectory("WheelTest");
        traj2 = Choreo.getTrajectory("WheelTest2");
    }
}

}
