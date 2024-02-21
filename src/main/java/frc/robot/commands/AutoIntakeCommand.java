package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class AutoIntakeCommand extends Command{
    IntakeSubsystem intakeSubsystem;
    double count;

    public AutoIntakeCommand(){
    intakeSubsystem = IntakeSubsystem.getInstance();
    count = 0;
    addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        count += 1;
        intakeSubsystem.intakeIn();
    }

    @Override
    public boolean isFinished() {
        return count > 100;
    }
    

    @Override
    public void end(boolean isFinished) {
        intakeSubsystem.intakeStop();
        AutoAlignMethods.scheduleShoot();
    }

}