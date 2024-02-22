package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {

    private final ShooterSubsystem m_ShooterSubsystem;
    private int shootTrigger = XboxController.Axis.kRightTrigger.value;
    private Joystick controller;

    public ShootCommand(Joystick controller){
        m_ShooterSubsystem = ShooterSubsystem.getInstance();
        this.controller = controller;
        addRequirements(m_ShooterSubsystem);
        //initializes this command with the ShooterSubsystem as a requirement
    }

    @Override
    public void execute(){
        if(Constants.stickDeadband < controller.getRawAxis(shootTrigger)){
            m_ShooterSubsystem.setKicker();
        } else {
            m_ShooterSubsystem.resetKicker();
        }
    }
    
    @Override
    public void end(boolean end){
        m_ShooterSubsystem.resetKicker();
    }

}
