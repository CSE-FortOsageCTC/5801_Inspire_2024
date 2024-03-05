package frc.robot.commands;

import javax.swing.GroupLayout.Alignment;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;
    private int counter;

    public AutoShootCommand(){
        m_ShooterSubsystem = ShooterSubsystem.getInstance();
        addRequirements(m_ShooterSubsystem);
        counter = 0;
        //initializes this command with the ShooterSubsystem as a requirement
    }

    @Override
    public void execute(){
        counter += 1;
        m_ShooterSubsystem.setKicker();
        if (counter > 5){
         m_ShooterSubsystem.resetKicker();}
    }

    @Override
    public boolean isFinished(){
        return counter > 10;
    }
    
    @Override
    public void end(boolean isFinished){
        counter = 0;
        AlignmentTransitions.transitionToNote();
        m_ShooterSubsystem.setFlyWheels(0);
        //Swerve.getInstance().resetAutoRotateUtil();
    }
}

