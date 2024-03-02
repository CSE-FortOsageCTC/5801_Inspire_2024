package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoFlywheelCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private double count;

    public AutoFlywheelCommand(){
        shooterSubsystem = ShooterSubsystem.getInstance();
        count = 0;
        addRequirements(shooterSubsystem);
    }
    @Override
    public void execute(){
        shooterSubsystem.setFlyWheels(-1);
        count += 1;
        if (count > 50){
            new AutoShootCommand().schedule();
        }
    }
    @Override
    public boolean isFinished(){
        return count > 110;
    }
    @Override
    public void end(boolean end){
        System.out.println("It ended");
        count = 0;
    }
}

