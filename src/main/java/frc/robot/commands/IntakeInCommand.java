package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeInCommand extends Command{
    IntakeSubsystem intakeSubsystem;

    public IntakeInCommand(){
    intakeSubsystem = IntakeSubsystem.getInstance();
    addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.intakeIn();
    }

    @Override
    public boolean isFinished() {

        return false;

    }

    @Override
    public void end(boolean isFinished) {
        intakeSubsystem.intakeStop();
    }

}
