package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Timestamp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.AutoRotateUtil;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.DefaultTeleopSub;
import com.ctre.phoenix6.Timestamp;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class DefaultTeleop extends Command{


    private DefaultTeleopSub s_DefaultTeleop;
    private AutoRotateUtil s_AutoRotateUtil;

    private int translationSup;
    private int strafeSup;
    private int rotationSup;
    private int throttle;
    private boolean robotCentricSup;
    private Joystick driver;
    private Joystick operator;

    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(1.8); 
    private SlewRateLimiter throttleLimiter = new SlewRateLimiter(2);

    private PIDController rotationPidController = new PIDController(0, 0, 0);
    private Pose2d alignPose;


    public DefaultTeleop(Joystick driver, Joystick operator) {
        s_DefaultTeleop = DefaultTeleopSub.getInstance();
        s_AutoRotateUtil = new AutoRotateUtil(0); 
        this.driver = driver;
        this.operator = operator;
        throttle = XboxController.Axis.kRightTrigger.value;
        translationSup = XboxController.Axis.kLeftY.value;
        strafeSup = XboxController.Axis.kLeftX.value;
        rotationSup = XboxController.Axis.kRightX.value;
        robotCentricSup = true;
        addRequirements(s_DefaultTeleop);

    }

    @Override
    public void initialize() {
        
    } 
    

    @Override
    public void execute() { 

        double kP = SmartDashboard.getNumber("Auto Rotate kP", 0);
        double kI = SmartDashboard.getNumber("Auto Rotate kI", 0);
        double kD = SmartDashboard.getNumber("Auto Rotate kD", 0);
        rotationPidController.setP(kP);
        rotationPidController.setP(kI);
        rotationPidController.setP(kD);

        

        Pose2d botPose = s_DefaultTeleop.s_Limelight.getBotPose(); 
        double botX = botPose.getX();
        double botY = botPose.getY();
        if (botX == 0 && botY == 0){
            botPose = s_DefaultTeleop.s_Swerve.getEstimatedPosition();
            botX = botPose.getX();
            botY = botPose.getY();
        }
        else{
            s_DefaultTeleop.s_Swerve.updateWithVision(botPose, Timer.getFPGATimestamp());
        }
        
        double yAxis = -driver.getRawAxis(translationSup);
        double xAxis = -driver.getRawAxis(strafeSup);
        double rotationAxis = driver.getRawAxis(rotationSup);

        double xDiff = botX - alignPose.getX(); // gets distance of x between robot and target
        double yDiff = botY - alignPose.getY(); // gets distance of y between robot and target
        SmartDashboard.putNumber("Bot Pose X", botX);
        SmartDashboard.putNumber("Bot Pose Y", botY);
        // Pose2d botPose = s_DefaultTeleop.s_Limelight.getBotPose(); // gets botpose based on approximated position from limelight
        // double xDiff = botPose.getX() - alignPose.getX(); // gets distance of x between robot and target
        // double yDiff = botPose.getY() - alignPose.getY(); // gets distance of y between robot and target
        SmartDashboard.putNumber("Bot Pose X", botPose.getX());
        SmartDashboard.putNumber("Bot Pose Y", botPose.getY());

        double angle = Units.radiansToDegrees(Math.atan2(xDiff, yDiff));
        SmartDashboard.putNumber("Align Swerve Angle", angle);
        s_AutoRotateUtil.updateTargetAngle(angle - 90); // updates pid angle setpoint to the angle that faces towards the target by using arctangent2 (which keeps negative and junk for us so it'll be nice)
        double translationVal = MathUtil.applyDeadband(yAxis, Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(xAxis, Constants.stickDeadband);
        double rotationVal;


        double throttleAxis = driver.getRawAxis(throttle);

        throttleAxis = (Math.abs(throttleAxis) < Constants.stickDeadband) ? .1 : throttleAxis;
        rotationAxis = (Math.abs(rotationAxis) < Constants.stickDeadband) ? .1 : rotationAxis;

        if (AlignPosition.getPosition() == AlignPosition.Manual) {
            rotationVal = rotationLimiter.calculate(rotationAxis) * (throttleLimiter.calculate(throttleAxis));
        } else {
            rotationVal = s_AutoRotateUtil.calculateRotationSpeed();
        }

        
        

        SmartDashboard.putNumber("Translation Val", translationVal);
        SmartDashboard.putNumber("Strave Val", strafeVal);
        SmartDashboard.putNumber("Rotation Value", rotationVal);
        SmartDashboard.putNumber("Gyro", s_DefaultTeleop.s_Swerve.getGyroYaw().getDegrees());

        double throttleCalc = throttleLimiter.calculate(throttleAxis);

        SmartDashboard.putNumber("throttle calculation", throttleCalc);

        Translation2d translation = new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed * throttleCalc);

        double rotationSpeed = s_DefaultTeleop.s_Swerve.rotateToNote();
        SmartDashboard.putNumber("Rotation Speed", rotationSpeed);

        s_DefaultTeleop.s_Swerve.drive(translation,  rotationVal * Constants.Swerve.maxAngularVelocity, robotCentricSup, true);
        // s_DefaultTeleop.s_Swerve.drive(translation, s_DefaultTeleop.s_Swerve.rotateToSpeaker() * Constants.Swerve.maxAngularVelocity, robotCentricSup, true);


    }

    @Override
    public void end(boolean isFinished) {
        s_AutoRotateUtil.end();
    }
}