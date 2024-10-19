package frc.robot.commands;

import javax.swing.GroupLayout.Alignment;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ChoreoSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

public class Auto_SevenP extends SequentialCommandGroup{

    public Auto_SevenP() {
        addRequirements(Swerve.getInstance(), IntakeSubsystem.getInstance(), ElevatorSubsystem.getInstance(), ShooterSubsystem.getInstance(), ChoreoSubsystem.getInstance());
        addCommands(
    
        

            
                new WaitCommand(0.2),//
                new InstantCommand(() -> AlignmentTransitions.scheduleShoot()),
                new WaitCommand(0.6),//.6
                new ParallelCommandGroup(
                    ChoreoSubsystem.getInstance().setupAutonomousChoreoPath(Choreo.getTrajectory("SevenP")),
                    new SequentialCommandGroup(
                        new WaitCommand(1.45), // Timestamp: 1
                        new InstantCommand(() -> AlignmentTransitions.scheduleIntake()),
                        new WaitCommand(1.65), // Timestamp: 2.6
                        new InstantCommand(() -> AlignmentTransitions.scheduleFlywheels()),
                        new WaitCommand(0.7), // Timestamp: 3.3
                        new InstantCommand(() -> AlignmentTransitions.scheduleOnlyIntake()),
                        new WaitCommand(1.4), // Timestamp: 4.7
                        new InstantCommand(() -> AlignmentTransitions.scheduleOnlyIntake()),
                        new WaitCommand(1.45), // Timestamp: 6.7
                        new InstantCommand(() -> AlignmentTransitions.stopFlywheels()),
                        new WaitCommand(0.15),
                        new InstantCommand(() -> AlignmentTransitions.scheduleOnlyIntake()),
                        new WaitCommand(2.25),
                        new InstantCommand(() -> AlignmentTransitions.scheduleShoot()),
                        new WaitCommand(1),
                        new InstantCommand(() -> AlignmentTransitions.stopFlywheels()),
                        new WaitCommand(1.3),
                        new InstantCommand(() -> AlignmentTransitions.scheduleOnlyIntake())
                        
                        
                    )
                )
            );
    

    }

    

}
