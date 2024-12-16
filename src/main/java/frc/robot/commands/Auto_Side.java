package frc.robot.commands;

import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ChoreoSubsystem;

public class Auto_Side extends SequentialCommandGroup{

    public Auto_Side(ChoreoTrajectory traj) {
        //addRequirements(Swerve.getInstance(), IntakeSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ShooterSubsystem.getInstance());
        super(
    
        new ParallelCommandGroup(

            
            new SequentialCommandGroup(
                new WaitCommand(0.2),
                new InstantCommand(() -> AlignmentTransitions.scheduleShoot()),
                new WaitCommand(0.6),
                new ParallelCommandGroup(
                    ChoreoSubsystem.getInstance().setupAutonomousChoreoPath(traj),
                    new SequentialCommandGroup(
                        new WaitCommand(2.4), // Timestamp: 2.4
                        new InstantCommand(() -> AlignmentTransitions.scheduleOnlyIntake()),
                        new WaitCommand(2.6),
                        new InstantCommand(() -> AlignmentTransitions.scheduleShoot()),
                        new WaitCommand(2.8),
                        new InstantCommand(() -> AlignmentTransitions.scheduleOnlyIntake()),
                        new WaitCommand(2.7),
                        new InstantCommand(() -> AlignmentTransitions.scheduleShoot())
                    )
                )
            )

        ));
    

    }

    

}
