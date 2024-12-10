package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ChoreoSubsystem;

public class Auto_FourP extends SequentialCommandGroup{

    
public Auto_FourP(Trajectories trajectories) {

    super(
        
        
        new SequentialCommandGroup(
            new InstantCommand(() -> AlignmentTransitions.scheduleShoot()),
            new WaitCommand(0.6),
            new AutoPickupNote(40).alongWith(ChoreoSubsystem.getInstance().setupAutonomousChoreoPath(trajectories.traj)),
            new InstantCommand(() -> AlignmentTransitions.scheduleShoot()),
            new AutoPickupNote(30).alongWith(ChoreoSubsystem.getInstance().setupAutonomousChoreoPath(trajectories.traj2)),
            new InstantCommand(() -> AlignmentTransitions.scheduleShoot())
        )
        


    );
}

public static class Trajectories {
    public ChoreoTrajectory traj;
    public ChoreoTrajectory traj2;

    public Trajectories() {
        traj = Choreo.getTrajectory("NewFourP1");
        traj2 = Choreo.getTrajectory("FourP2");
    }
}

}
