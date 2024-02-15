package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ElevatorCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private boolean up;
    public ElevatorCommand(boolean up){
        this.up = up;
        shooterSubsystem = ShooterSubsystem.getInstance();
        
    }
    @Override
    public void execute(){
        if (up == true){shooterSubsystem.setElevatorSpeed(.3);}
        else{shooterSubsystem.setElevatorSpeed(-.3);}
        
    }
    @Override
    public void end(boolean end){
        shooterSubsystem.setElevatorSpeed(0);
    }
        
    
}
