package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoRotateUtil;
import frc.robot.Constants;
import frc.robot.subsystems.DefaultTeleopSub;

public class DefaultTeleop extends Command{

    private XboxController controller = new XboxController(0);
    private DefaultTeleopSub s_DefaultTeleop;
    private AutoRotateUtil s_AutoRotateUtil;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private PIDController rotationPidController;
    public boolean isAligning = false;
    private Pair<Double, Double> speakerCoordinate;

    public DefaultTeleop(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        s_DefaultTeleop = DefaultTeleopSub.getInstance();
        s_AutoRotateUtil = new AutoRotateUtil(0);
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        addRequirements(s_DefaultTeleop);

    }

    @Override
    public void initialize() {
        double kP = SmartDashboard.getNumber("Auto Rotate kP", 0);
        double kI = SmartDashboard.getNumber("Auto Rotate kI", 0);
        double kD = SmartDashboard.getNumber("Auto Rotate kD", 0);
        rotationPidController.setP(kP);
        rotationPidController.setP(kI);
        rotationPidController.setP(kD);
        if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            speakerCoordinate = new Pair<Double, Double>(8.0, 1.5);
        } else {
            speakerCoordinate = new Pair<Double, Double>(-8.0, 1.5);
        }
            
    } 
    

    @Override
    public void execute() { 
        Pose2d botPose = s_DefaultTeleop.s_Limelight.getBotPose(); // gets botpose based on approximated position from limelight
        double xDiff = botPose.getX() - speakerCoordinate.getFirst(); // gets distance of x between robot and target
        double yDiff = botPose.getY() - speakerCoordinate.getSecond(); // gets distance of y between robot and target
        s_AutoRotateUtil.updateTargetAngle(Math.atan2(yDiff, xDiff)); // updates pid angle setpoint to the angle that faces towards the target by using arctangent2 (which keeps negative and junk for us so it'll be nice)
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal;
        
        if ((MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) > 0 && isAligning) || (controller.getBButtonPressed() && isAligning)) {
            isAligning = false;
        } else if (controller.getBButtonPressed() && !isAligning) {
            isAligning = true;
        }

        rotationVal = (isAligning? s_AutoRotateUtil.calculateRotationSpeed():MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband));

        s_DefaultTeleop.s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), rotationVal * Constants.Swerve.maxAngularVelocity, !robotCentricSup.getAsBoolean(), true);

    }
}