package frc.robot.commands;

import java.util.Optional;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(5.0); 
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
        addRequirements(s_DefaultTeleop.s_Swerve);

    }

    @Override
    public void initialize() {
        
    } 
    

    @Override
    public void execute() { 
        Alliance alliance = DriverStation.getAlliance().get();
        double yAxis = alliance.equals(Alliance.Red) ? -driver.getRawAxis(translationSup) : driver.getRawAxis(translationSup);
        double xAxis = alliance.equals(Alliance.Red) ? -driver.getRawAxis(strafeSup) : driver.getRawAxis(strafeSup);
        double rotationAxis = driver.getRawAxis(rotationSup);
        alignPose = AlignPosition.getAlignPose();
        

        // if(alignPose != null && AlignPosition.getPosition().equals(AlignPosition.SpeakerPos)){
        //     // System.out.println("it got to the speaker pos if >:)");
        //     double xDiff = botX - alignPose.getX(); // gets distance of x between robot and target
        //     double yDiff = botY - alignPose.getY(); // gets distance of y between robot and target
        //     SmartDashboard.putNumber("AlingPos X Vaule", alignPose.getX());
        //     SmartDashboard.putNumber("AlingPos Y Vaule", alignPose.getY());
        //     double angle = Units.radiansToDegrees(Math.atan2(yDiff, xDiff));
        //     SmartDashboard.putNumber("Align Swerve Angle", angle);
        //     s_AutoRotateUtil.updateTargetAngle(angle - 90); // updates pid angle setpoint to the angle that faces towards the target by using arctangent2 (which keeps negative and junk for us so it'll be nice)
        // }

        double translationVal = MathUtil.applyDeadband(yAxis, Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(xAxis, Constants.stickDeadband);
        double rotationVal;


        double throttleAxis = driver.getRawAxis(throttle);

        throttleAxis = (Math.abs(throttleAxis) < Constants.stickDeadband) ? .15 : throttleAxis;
        rotationAxis = (Math.abs(rotationAxis) < Constants.stickDeadband) ? 0 : rotationAxis;

        if (rotationAxis != 0) {
            AlignPosition.setPosition(AlignPosition.Manual);
        }

        if (AlignPosition.getPosition().equals(AlignPosition.Manual)) {
            rotationVal = rotationLimiter.calculate(rotationAxis) * (throttleLimiter.calculate(throttleAxis));
            robotCentricSup = true;
        } else if (AlignPosition.getPosition().equals(AlignPosition.SpeakerPos)){
            rotationVal = s_DefaultTeleop.s_Swerve.rotateToSpeaker(); // s_AutoRotateUtil.calculateRotationSpeed();
            robotCentricSup = true;
        } else if (AlignPosition.getPosition().equals(AlignPosition.AmpPos)) {
            rotationVal = s_DefaultTeleop.s_Swerve.rotateToAmp();
            robotCentricSup = true;
            
        } else if (AlignPosition.getPosition().equals(AlignPosition.AutoPickup)) {
           rotationVal = s_DefaultTeleop.s_Swerve.rotateToNote();
           robotCentricSup = false;
        } else {
            rotationVal = rotationLimiter.calculate(rotationAxis) * (throttleLimiter.calculate(throttleAxis));
            robotCentricSup = true;
        }
        
        

        // SmartDashboard.putNumber("Translation Val", translationVal);
        // SmartDashboard.putNumber("Strave Val", strafeVal);
        // SmartDashboard.putNumber("Rotation Value", rotationVal);
        // SmartDashboard.putNumber("Gyro", s_DefaultTeleop.s_Swerve.getGyroYaw().getDegrees());

        double throttleCalc = throttleLimiter.calculate(throttleAxis);

        //SmartDashboard.putNumber("throttle calculation", throttleCalc);

        Translation2d translation = new Translation2d(translationVal, strafeVal).times(-Constants.Swerve.maxSpeed * throttleCalc);

        //SmartDashboard.putNumber("Rotation Speed", rotationSpeed);

        s_DefaultTeleop.s_Swerve.drive(translation,  rotationVal * Constants.Swerve.maxAngularVelocity, robotCentricSup, true);
        // s_DefaultTeleop.s_Swerve.drive(translation, s_DefaultTeleop.s_Swerve.rotateToSpeaker() * Constants.Swerve.maxAngularVelocity, robotCentricSup, true);


    }

    @Override
    public void end(boolean isFinished) {
        s_AutoRotateUtil.end();
    }
}