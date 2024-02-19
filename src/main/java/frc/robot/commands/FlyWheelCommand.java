package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class FlyWheelCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    public FlyWheelCommand(){
        shooterSubsystem = ShooterSubsystem.getInstance();
        
    }
    @Override
    public void execute(){
        shooterSubsystem.setFlyWheels(-1);

    }
    @Override
    public void end(boolean end){
        shooterSubsystem.setFlyWheels(0);
    }
}
