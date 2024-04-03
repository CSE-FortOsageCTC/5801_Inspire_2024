package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbingSubsystem;

public class Climb extends Command{
    ClimbingSubsystem s_climbingSubsystem;
    double leftSpeed;
    double rightSpeed;

    public Climb(double leftSpeed, double rightSpeed){
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        s_climbingSubsystem = ClimbingSubsystem.getInstance();
        addRequirements(s_climbingSubsystem);
    }

    @Override
    public void execute(){ // add a counter later guys :)
        s_climbingSubsystem.climbControl(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean isFinished){
        s_climbingSubsystem.climbControl(0, 0);
    }
}
