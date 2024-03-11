package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class FixIntakeCommand extends Command{
    
    IntakeSubsystem intakeSubsystem;

    public FixIntakeCommand() {
        intakeSubsystem = IntakeSubsystem.getInstance();
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.intakeFix();
    }

    @Override
    public void end(boolean isFinished) {
        intakeSubsystem.intakeStop();
    }

}
