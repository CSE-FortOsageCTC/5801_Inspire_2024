package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {

    private final ShooterSubsystem m_ShooterSubsystem;

    public ShootCommand(){
        m_ShooterSubsystem = ShooterSubsystem.getInstance();

        addRequirements(m_ShooterSubsystem);
        //initializes this command with the ShooterSubsystem as a requirement
    }

    @Override
    public void execute(){
        m_ShooterSubsystem.shoot(.5);
    }
    
    @Override
    public void end(boolean end){
        m_ShooterSubsystem.shoot(0);
    }

}
