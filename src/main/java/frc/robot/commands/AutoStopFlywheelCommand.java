package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoStopFlywheelCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private double count;

    public AutoStopFlywheelCommand(){
        shooterSubsystem = ShooterSubsystem.getInstance();
        count = 0;
        AlignPosition.setPosition(AlignPosition.SpeakerPos);
        addRequirements(shooterSubsystem);

    }
    @Override
    public void execute(){
        shooterSubsystem.setFlyWheels(0);
    }

    @Override
    public void end(boolean end){
        System.out.println("It ended");
        //AlignPosition.setPosition(AlignPosition.Manual);
        count = 0;
    }
}

