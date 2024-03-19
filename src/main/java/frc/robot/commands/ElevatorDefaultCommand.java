package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.AngleShooterUtil;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDefaultCommand extends Command{

    private ElevatorSubsystem elevatorSubsystem;
    private AngleShooterUtil angleShooterUtil;
    private Swerve s_Swerve;
    private Pair<Double, Double> speakerCoordinate;
    private int stickSup = XboxController.Axis.kLeftY.value;
    
    private Joystick operator;

    public ElevatorDefaultCommand(Joystick operator){
        elevatorSubsystem = ElevatorSubsystem.getInstance();
        s_Swerve = Swerve.getInstance();
        this.operator = operator;
        addRequirements(elevatorSubsystem);
        angleShooterUtil = new AngleShooterUtil(0);
    }
    
    @Override
    public void initialize() {
        angleShooterUtil.initialize();
    }    

    @Override    
    public void execute(){ 
        Pose2d lightBotPose = s_Swerve.getLimelightBotPose();

        boolean isRed = DriverStation.getAlliance().get().equals(Alliance.Red);

        //SmartDashboard.putBoolean("Is Red Alliance", isRed);

        double xDiff = lightBotPose.getX() - (isRed? Units.inchesToMeters(652.73):Units.inchesToMeters(-1.5));
        double yDIff = lightBotPose.getY() - Units.inchesToMeters(218.42);
        double distance = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDIff, 2));

        // distance = Units.metersToInches(distance);

        // ChassisSpeeds speeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(s_Swerve.getModuleStates());
        // distance = s_Swerve.getVelocityCorrectionDistance(distance, speeds);
        // distance = s_Swerve.getVelocityCorrectionDistance(distance, speeds); // called twice for better accuracy


        //double feetDistance = Units.metersToFeet(distance);

        //SmartDashboard.putNumber("Speaker Distance (ft.)", feetDistance);

        double angle = Units.radiansToDegrees(Math.atan2(Constants.speakerHeightMeters, distance));

        //SmartDashboard.putNumber("ElevatorDegrees", angle);

        double degreesToEncoderAngle = (angle - Constants.Swerve.minElevatorAngle) * Constants.Swerve.degreesToEncoderValue;

        //SmartDashboard.putNumber("Final Encoder Value", degreesToEncoderAngle);

        double elevatorValue = elevatorSubsystem.getElevatorValue();  
        
        // double target = (-0.00384 * (distance * distance)) + (1.17 * distance) - 94.8 + 3;
        double target = elevatorValue - degreesToEncoderAngle;

        //SmartDashboard.putNumber("Encoder Error", target);
        boolean isAlignedAmp = AlignPosition.getPosition().equals(AlignPosition.AmpPos);
    
        if (!isAlignedAmp && Math.abs(operator.getRawAxis(stickSup)) > Constants.stickDeadband) {

            elevatorSubsystem.setElevatorSpeed(operator.getRawAxis(stickSup) < 0? -0.5 : 0.5);

        } else if (!isAlignedAmp && Math.abs(operator.getRawAxis(stickSup)) < Constants.stickDeadband) {

            angleShooterUtil.updateTargetDiff(target);
            elevatorSubsystem.setElevatorSpeed(angleShooterUtil.calculateElevatorSpeed());
            
        } else if (isAlignedAmp) {

            angleShooterUtil.updateTargetDiff(elevatorValue - Constants.Swerve.maxElevatorValue);
            elevatorSubsystem.setElevatorSpeed(angleShooterUtil.calculateElevatorSpeed());

        }

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
