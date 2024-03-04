package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoFlywheelCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private double count;

    public AutoFlywheelCommand(){
        shooterSubsystem = ShooterSubsystem.getInstance();
        count = 0;
        AlignPosition.setPosition(AlignPosition.SpeakerPos);
        addRequirements(shooterSubsystem);

    }
    @Override
    public void execute(){
        shooterSubsystem.setFlyWheels(-1);
        count += 1;
        if (count > 35){
            new AutoShootCommand().schedule();
        }
    }
    @Override
    public boolean isFinished(){
        return count > 40;
    }
    @Override
    public void end(boolean end){
        System.out.println("It ended");
        //AlignPosition.setPosition(AlignPosition.Manual);
        count = 0;
    }
}

