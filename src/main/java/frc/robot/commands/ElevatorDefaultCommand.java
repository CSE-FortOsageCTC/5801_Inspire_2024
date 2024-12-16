package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.AngleShooterUtil;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.AmpArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ElevatorDefaultCommand extends Command{

    private ElevatorSubsystem elevatorSubsystem;
    private AmpArmSubsystem ampArmSubsystem;
    private AngleShooterUtil angleShooterUtil;
    private LEDSubsystem ledSubsystem;
    private Swerve s_Swerve;
    private int stickSup = XboxController.Axis.kLeftY.value;
    private Joystick operator;
    public ElevatorDefaultCommand(Joystick operator, Joystick driver){
        elevatorSubsystem = ElevatorSubsystem.getInstance();
        ampArmSubsystem = AmpArmSubsystem.getInstance();
        ledSubsystem = LEDSubsystem.getInstance();
        s_Swerve = Swerve.getInstance();
        this.operator = operator;
        addRequirements(elevatorSubsystem);
        angleShooterUtil = new AngleShooterUtil(0);
    }

    public void increment()
    {
        if (ledSubsystem.ledCycle != 1) {
            ledSubsystem.ledCycle += .01;
        }
    }

    public void decrement()
    {
        
        if (ledSubsystem.ledCycle != -1) {
            ledSubsystem.ledCycle -= .01;
        }
    }

   
    @Override
    public void initialize() {
        angleShooterUtil.initialize();
    }    

    @Override    
    public void execute(){ 
        

        Pose2d lightBotPose = DriverStation.isAutonomousEnabled()? s_Swerve.getAutoLimelightBotPose():s_Swerve.getTeleopLimelightBotPose();

        boolean isRed = DriverStation.getAlliance().get().equals(Alliance.Red);

        boolean feedMode = AlignPosition.getPosition().equals(AlignPosition.StagePos);

        double xDiff;
        double yDiff;
        
        if (AlignPosition.getPosition().equals(AlignPosition.StagePos)) { 
            xDiff = lightBotPose.getX() - AlignPosition.getAlignPose().getX();
            yDiff = lightBotPose.getY() - AlignPosition.getAlignPose().getY();
        } else {
            xDiff = lightBotPose.getX() - (isRed? Units.inchesToMeters(652.73):Units.inchesToMeters(-1.5));
            yDiff = lightBotPose.getY() - Units.inchesToMeters(218.42);
        }
        double distance = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));

        if (feedMode)  {
            distance /= 2;
        }

        double distanceInch = Units.metersToInches(distance);

        SmartDashboard.putNumber("Speaker Distance (in.)", distanceInch);

        double angle = Units.radiansToDegrees(Math.atan2(feedMode ? Constants.stageHeightMeters:Constants.speakerHeightMeters, distance));

        double degreesToEncoderAngle = (angle - Constants.Swerve.minElevatorAngle) * Constants.Swerve.degreesToEncoderValue;

        double elevatorValue = elevatorSubsystem.getElevatorValue();  

        SmartDashboard.putNumber("Current Encoder Value", elevatorValue);
        
        double equationTarget = (-0.00353 * (distanceInch * distanceInch)) + ((1.18) * distanceInch) - 98.6 - 2;
        equationTarget = elevatorValue - equationTarget;
        double tangentTarget = elevatorValue - degreesToEncoderAngle;

        elevatorSubsystem.isAligned = false;
        
        if (!ampArmSubsystem.isUp && Math.abs(operator.getRawAxis(stickSup)) > Constants.stickDeadband) {

            elevatorSubsystem.setElevatorSpeed(operator.getRawAxis(stickSup) < 0? -0.5 : 0.5);

        } else if (!ampArmSubsystem.isUp && Math.abs(operator.getRawAxis(stickSup)) < Constants.stickDeadband) {

            if (DriverStation.isAutonomousEnabled()) {

                angleShooterUtil.updateTargetDiff(tangentTarget);

            } else if (DriverStation.isTeleopEnabled()) {

                angleShooterUtil.updateTargetDiff(tangentTarget);

            }

            double elevatorSpeed = angleShooterUtil.calculateElevatorSpeed();
            elevatorSubsystem.setElevatorSpeed(elevatorSpeed);
            elevatorSubsystem.isAligned = Math.abs(elevatorSpeed) <= 0.5;
            
        } else if (ampArmSubsystem.isUp) {

            angleShooterUtil.updateTargetDiff(elevatorValue - (-32.5));
            elevatorSubsystem.setElevatorSpeed(angleShooterUtil.calculateElevatorSpeed());

        }
        boolean inRange = distanceInch < 152.474;
        SmartDashboard.putBoolean("In Range?", inRange);

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
