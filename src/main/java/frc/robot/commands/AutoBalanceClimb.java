package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbingSubsystem;

public class AutoBalanceClimb extends Command{
    ClimbingSubsystem s_climbingSubsystem;
    double leftSpeed;
    double rightSpeed;

    public AutoBalanceClimb(){
        s_climbingSubsystem = ClimbingSubsystem.getInstance();
        addRequirements(s_climbingSubsystem);
    }

    @Override
    public void execute(){ // add a counter later guys :)
        if(s_climbingSubsystem.s_Swerve.getGyroRoll() < -5){
            // left climber is going go negative in speed, right will be positive
            leftSpeed = (s_climbingSubsystem.s_Swerve.getGyroRoll()) / 100;
            rightSpeed = -(s_climbingSubsystem.s_Swerve.getGyroRoll()) / 100;
        } else if (s_climbingSubsystem.s_Swerve.getGyroRoll() > 5){
            leftSpeed = -(s_climbingSubsystem.s_Swerve.getGyroRoll()) / 100;
            rightSpeed = (s_climbingSubsystem.s_Swerve.getGyroRoll()) / 100;
        } else {
            leftSpeed = 0.75;
            rightSpeed = 0.75;
        }

        s_climbingSubsystem.climbControl(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean isFinished){
        leftSpeed = 0;
        rightSpeed = 0;

        s_climbingSubsystem.climbControl(leftSpeed, rightSpeed);
    }
}
