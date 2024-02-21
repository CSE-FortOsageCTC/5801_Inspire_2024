package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;
    private boolean finished;

    public AutoShootCommand(){
        m_ShooterSubsystem = ShooterSubsystem.getInstance();
        addRequirements(m_ShooterSubsystem);
        //initializes this command with the ShooterSubsystem as a requirement
    }

    @Override
    public void execute(){
        m_ShooterSubsystem.resetKicker();
        finished = true;
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
    
    @Override
    public void end(boolean isFinished){
        m_ShooterSubsystem.setKicker();
        AlignPosition.setPosition(AlignPosition.AutoPickup);
        Swerve.getInstance().resetAutoRotateUtil();
    }
}

