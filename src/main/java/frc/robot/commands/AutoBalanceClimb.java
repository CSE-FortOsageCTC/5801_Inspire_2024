package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.Swerve;

public class AutoBalanceClimb extends Command{
    ClimbingSubsystem s_climbingSubsystem;
    double leftSpeed;
    double rightSpeed;

    public AutoBalanceClimb(){
        s_climbingSubsystem = ClimbingSubsystem.getInstance();
        addRequirements(s_climbingSubsystem);
    }
    
    @Override
    public void execute(){
        double gyroRoll = Swerve.getInstance().getGyroRoll();

        if(gyroRoll < -5){
            // left climber is going go negative in speed, right will be positive
            leftSpeed = (gyroRoll) / 100;
            rightSpeed = -(gyroRoll) / 100;
        } else if (gyroRoll > 5){
            leftSpeed = -(gyroRoll) / 100;
            rightSpeed = (gyroRoll) / 100;
        }

        s_climbingSubsystem.climbControl(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean isFinished){
        s_climbingSubsystem.climbControl(0, 0);
    }
}
