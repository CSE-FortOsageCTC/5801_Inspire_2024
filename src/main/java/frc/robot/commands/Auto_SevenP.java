package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ChoreoSubsystem;

public class Auto_SevenP extends SequentialCommandGroup{

    public Auto_SevenP() {
        super(
    
        new ParallelCommandGroup(

            ChoreoSubsystem.getInstance().setupAutonomousChoreoPath(Choreo.getTrajectory("SevenP")),
            new SequentialCommandGroup(

                new WaitCommand(0.5),
                new InstantCommand(() -> AlignmentTransitions.scheduleIntake())

            )

        ));
    

    }

    

}
