package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ChoreoSubsystem;

public class NoahsAuto extends SequentialCommandGroup{
    public NoahsAuto(){
        super(
                new WaitCommand(0.2),
                new InstantCommand(() -> AlignmentTransitions.scheduleShoot()),
                new WaitCommand(0.6),
                new ParallelCommandGroup(
                    ChoreoSubsystem.getInstance().setupAutonomousChoreoPath(Choreo.getTrajectory("Noah's auto")),
                    new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new InstantCommand(() -> AlignmentTransitions.scheduleIntake()),
                    new WaitCommand(0.64),
                    new InstantCommand(() -> AlignmentTransitions.scheduleShoot())
                    )
                )  
            );
        
        }
    }
