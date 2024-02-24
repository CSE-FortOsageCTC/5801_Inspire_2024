package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AngleShooterUtil;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDefaultCommand extends Command{

    private ElevatorSubsystem elevatorSubsystem;
    private AngleShooterUtil angleShooterUtil;
    private Swerve s_Swerve;
    private Pair<Double, Double> speakerCoordinate;

    public ElevatorDefaultCommand(){
        elevatorSubsystem = ElevatorSubsystem.getInstance();
        s_Swerve = Swerve.getInstance();
        addRequirements(elevatorSubsystem);
        angleShooterUtil = new AngleShooterUtil(0);
    }
    
    @Override
    public void initialize() {
        angleShooterUtil.initialize();
    }    

    @Override    
    public void execute(){
        ChassisSpeeds chassisSpeeds = s_Swerve.getEstimatedFieldRelativeSpeeds();
        
        SmartDashboard.putNumber("xSpeed", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("ySpeed", chassisSpeeds.vyMetersPerSecond);

        if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            speakerCoordinate = new Pair<Double, Double>(8.0, 1.5);
        } else {
            speakerCoordinate = new Pair<Double, Double>(-8.0, 1.5);
        }

        Pose2d botPose = elevatorSubsystem.s_Limelight.getBotPose(); 
        botPose = s_Swerve.getLimelightBotPose();

        double xDiff = botPose.getX() - speakerCoordinate.getFirst(); // gets distance of x between robot and target
        double yDiff = botPose.getY() - speakerCoordinate.getSecond();

        double distance = Math.sqrt(yDiff * yDiff + xDiff * xDiff);

        double target = -.00384 * distance * distance + 1.17 * distance - 94.8; //NEEDS TO BE REPLACED LATER WITH THE TARGET ANGLE
    
        double elevatorValue = elevatorSubsystem.getElevatorValue();    
        angleShooterUtil.updateTargetDiff(elevatorValue - target);

        SmartDashboard.putNumber("ShooterEncoder", elevatorValue);
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
