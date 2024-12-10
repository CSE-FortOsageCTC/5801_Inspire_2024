package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoOnlyFlywheelCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private double count;

    public AutoOnlyFlywheelCommand(){
        intakeSubsystem = IntakeSubsystem.getInstance();
        shooterSubsystem = ShooterSubsystem.getInstance();
        count = 0;
        AlignPosition.setPosition(AlignPosition.SpeakerPos);
        addRequirements(shooterSubsystem);

    }
    @Override
    public void execute(){
        shooterSubsystem.setFlyWheels(-1);
        
        if (intakeSubsystem.isRingDetected()) {
            count += 1;
        } else {
            shooterSubsystem.resetKicker();
            count = 0;
        }
        if (count > 3) {
            shooterSubsystem.setKicker();
        }
    }

    //TODO: no elevator input until limit is hit

    @Override
    public void end(boolean end){
        System.out.println("It ended");
        //AlignPosition.setPosition(AlignPosition.Manual);
        count = 0;
    }
}

