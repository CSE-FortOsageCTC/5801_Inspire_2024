package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AngleShooterUtil;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDefaultCommand extends Command{

    private ElevatorSubsystem elevatorSubsystem;
    private AngleShooterUtil angleShooterUtil;

    public ElevatorDefaultCommand(){
        elevatorSubsystem = ElevatorSubsystem.getInstance();
        addRequirements(elevatorSubsystem);
        angleShooterUtil = new AngleShooterUtil(0);
    }
    
    @Override
    public void initialize() {
        angleShooterUtil.initialize();
    }    

    @Override    
    public void execute(){
        double target = 0; //NEEDS TO BE REPLACED LATER WITH THE TARGET ANGLE
    
        Pose2d botPose = elevatorSubsystem.s_Limelight.getBotPose(); 
        double botX = botPose.getX();
        double botY = botPose.getY();
        if (botX == 0 && botY == 0){
            botPose = elevatorSubsystem.s_Swerve.getEstimatedPosition();
            botX = botPose.getX();
            botY = botPose.getY();
            // SmartDashboard.putNumber("Bot Pose X", botX);
            // SmartDashboard.putNumber("Bot Pose Y", botY);
        }
        else{
            elevatorSubsystem.s_Swerve.updateWithVision(botPose, Timer.getFPGATimestamp());
        }

        double elevatorValue = elevatorSubsystem.getElevatorValue();    
        angleShooterUtil.updateTargetDiff(elevatorValue - target);
    }

    @Override
    public void end(boolean end){
        elevatorSubsystem.setElevatorSpeed(0);
        angleShooterUtil.reset();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
