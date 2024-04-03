package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbingSubsystem;

public class ClimbReset extends Command{
    ClimbingSubsystem s_climbingSubsystem;
    double leftSpeed = -1;
    double rightSpeed = -1;
    Joystick controller;
    int back;

    public ClimbReset(Joystick controller){
        s_climbingSubsystem = ClimbingSubsystem.getInstance();
        this.controller = controller;
        back = XboxController.Button.kBack.value;

        addRequirements(s_climbingSubsystem);
    }

    @Override
    public void execute(){ // add a counter later guys :)
        if (controller.getRawButton(back)){
            s_climbingSubsystem.climbControl(leftSpeed, rightSpeed);
        }else
        {
            s_climbingSubsystem.climbControl(0, 0);
        }
        
    }

    @Override
    public void end(boolean isFinished){
        s_climbingSubsystem.climbControl(0, 0);
    }
}
