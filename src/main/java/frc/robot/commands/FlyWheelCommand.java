package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class FlyWheelCommand extends Command{
    public double speed;
    private final ShooterSubsystem shooterSubsystem;
    public FlyWheelCommand(double speed){
        shooterSubsystem = ShooterSubsystem.getInstance();
        this.speed = speed;
    }
    @Override
    public void execute(){
        shooterSubsystem.setFlyWheels(speed);

    }
    @Override
    public void end(boolean end){
        shooterSubsystem.setFlyWheels(0);
    }
}
