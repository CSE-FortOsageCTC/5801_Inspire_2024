package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeCommand extends Command{
    IntakeSubsystem intakeSubsystem;

    public IntakeCommand(){
    intakeSubsystem = IntakeSubsystem.getInstance();
    addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.intakeIn();
    }


}
