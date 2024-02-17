package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinKickerCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    public SpinKickerCommand(){
        shooterSubsystem = ShooterSubsystem.getInstance();
        
    }
    @Override
    public void execute(){
        shooterSubsystem.spinKicker(-.5);

    }
    @Override
    public void end(boolean end){
        shooterSubsystem.spinKicker(0);
    }
}
