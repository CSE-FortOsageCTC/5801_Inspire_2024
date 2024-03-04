package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class AutoOnlyIntakeCommand extends Command{
    IntakeSubsystem intakeSubsystem;
    double count;
    int detectedDelayCount;

    public AutoOnlyIntakeCommand(){
    intakeSubsystem = IntakeSubsystem.getInstance();
    count = 0;
    detectedDelayCount = 0;
    addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        count += 1;
        intakeSubsystem.intakeIn();
        if (intakeSubsystem.isRingDetected()) {
            detectedDelayCount += 1;
        }
        
    }

    @Override
    public boolean isFinished() {
        return detectedDelayCount > 5 || count > 70;
    }
    

    @Override
    public void end(boolean isFinished) {
        count = 0;
        intakeSubsystem.intakeStop();
    }

}