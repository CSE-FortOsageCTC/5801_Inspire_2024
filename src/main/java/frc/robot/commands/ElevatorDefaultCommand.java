package frc.robot.commands;

import java.sql.Driver;

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
import frc.robot.subsystems.LEDSubsystem;

public class ElevatorDefaultCommand extends Command{

    private ElevatorSubsystem elevatorSubsystem;
    private AngleShooterUtil angleShooterUtil;
    private LEDSubsystem ledSubsystem;
    private Swerve s_Swerve;
    private Pair<Double, Double> speakerCoordinate;
    private int stickSup = XboxController.Axis.kLeftY.value;
    private double setpoint;
    private boolean isManual;
    private AlignPosition lastAlignment;
    
    private Joystick operator;
    private Joystick driver;

    public ElevatorDefaultCommand(Joystick operator, Joystick driver){
        elevatorSubsystem = ElevatorSubsystem.getInstance();
        ledSubsystem = LEDSubsystem.getInstance();
        s_Swerve = Swerve.getInstance();
        this.operator = operator;
        this.driver = driver;
        addRequirements(elevatorSubsystem);
        angleShooterUtil = new AngleShooterUtil(0);
        setpoint = 0;
    }

    public void increment()
    {
        // isManual = true;
        if (ledSubsystem.ledCycle != 1) {
            ledSubsystem.ledCycle += .01;
        }
        // lastAlignment = AlignPosition.getPosition();
    }

    public void decrement()
    {
        // isManual = true;
        if (ledSubsystem.ledCycle != -1) {
            ledSubsystem.ledCycle -= .01;
        }
        // lastAlignment = AlignPosition.getPosition();
    }

    public void setToAuto()
    {
        isManual = lastAlignment == AlignPosition.getPosition();
        
    }

    @Override
    public void initialize() {
        angleShooterUtil.initialize();
    }    

    @Override    
    public void execute(){ 
        setToAuto();

        boolean isRed = DriverStation.getAlliance().get().equals(Alliance.Red);

        boolean feedMode = AlignPosition.getPosition().equals(AlignPosition.StagePos);

        double xDiff = 0;
        double yDiff = 0;
        double distance = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));

        if (feedMode)  {
            distance /= 2;
        }

        // ChassisSpeeds speeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(s_Swerve.getModuleStates());
        // distance = s_Swerve.getVelocityCorrectionDistance(distance, speeds);
        // distance = s_Swerve.getVelocityCorrectionDistance(distance, speeds); // called twice for better accuracy

        double distanceInch = Units.metersToInches(distance);

        SmartDashboard.putNumber("Speaker Distance (in.)", distanceInch);

        double angle = Units.radiansToDegrees(Math.atan2(feedMode ? Constants.stageHeightMeters:Constants.speakerHeightMeters, distance));

        // SmartDashboard.putNumber("ElevatorDegrees", angle);

        double degreesToEncoderAngle = (angle - Constants.Swerve.minElevatorAngle) * Constants.Swerve.degreesToEncoderValue;

        // SmartDashboard.putNumber("Final Encoder Value", degreesToEncoderAngle);

        double elevatorValue = elevatorSubsystem.getElevatorValue();  

        SmartDashboard.putNumber("Current Encoder Value", elevatorValue);
        
        double equationTarget = (-0.00353 * (distanceInch * distanceInch)) + ((1.18) * distanceInch) - 98.6 - 2; // (-0.00384 * (distanceInch * distanceInch)) + ((1.17 + 0.01) * distanceInch) - 94.8;
        equationTarget = elevatorValue - equationTarget;
        double tangentTarget = elevatorValue - degreesToEncoderAngle;

        double target = (equationTarget + tangentTarget) / 2;

        boolean isAlignedAmp = AlignPosition.getPosition().equals(AlignPosition.AmpPos);

        // if (isManual){
        //     target = elevatorValue - SmartDashboard.getNumber("Setpoint", 0);
        //     SmartDashboard.putNumber("Setpoint", setpoint);
        // } else {
        //     setpoint = elevatorValue;
        // }

        elevatorSubsystem.isAligned = false;

        boolean inRange = distanceInch < 152.474;
        SmartDashboard.putBoolean("In Range?", inRange);
        
        // SmartDashboard.putNumber("Encoder Error", target);

        SmartDashboard.putBoolean("Aligned Shoot", elevatorSubsystem.isAligned);
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
