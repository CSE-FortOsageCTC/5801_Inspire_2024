package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Timestamp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
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
import frc.robot.AutoRotateUtil;
import frc.robot.Constants;
import frc.robot.subsystems.DefaultTeleopSub;

public class DefaultTeleop extends Command{


    private DefaultTeleopSub s_DefaultTeleop;
    private AutoRotateUtil s_AutoRotateUtil;

    private Debouncer bButtonDebouncer = new Debouncer(0.25);

    private int translationSup;
    private int strafeSup;
    private int rotationSup;
    private int throttle;
    private boolean robotCentricSup;
    private Joystick driver;

    private PIDController rotationPidController = new PIDController(0, 0, 0);
    public boolean isAligning = false;
    private Pair<Double, Double> speakerCoordinate;
    private boolean bButtonPressed;

    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(1.8); 
    private SlewRateLimiter throttleLimiter = new SlewRateLimiter(2);

    public DefaultTeleop(Joystick controller, int translationSup, int strafeSup, int rotationSup, boolean robotCentricSup, int throttle) {
        s_DefaultTeleop = DefaultTeleopSub.getInstance();
        s_AutoRotateUtil = new AutoRotateUtil(0);
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.driver = controller;
        this.throttle = throttle;
        bButtonPressed = bButtonDebouncer.calculate(driver.getRawButtonReleased(XboxController.Button.kB.value));
        addRequirements(s_DefaultTeleop);

    }

    @Override
    public void initialize() {

        if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            speakerCoordinate = new Pair<Double, Double>(8.0, 1.5);
        } else {
            speakerCoordinate = new Pair<Double, Double>(-8.0, 1.5);
        }
            
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
        
        double xDiff = botX - speakerCoordinate.getFirst(); // gets distance of x between robot and target
        double yDiff = botY - speakerCoordinate.getSecond(); // gets distance of y between robot and target
        SmartDashboard.putNumber("Bot Pose X", botX);
        SmartDashboard.putNumber("Bot Pose Y", botY);
        double angle = Units.radiansToDegrees(Math.atan2(xDiff, yDiff));
        SmartDashboard.putNumber("Align Swerve Angle", angle);
        s_AutoRotateUtil.updateTargetAngle(angle - 90); // updates pid angle setpoint to the angle that faces towards the target by using arctangent2 (which keeps negative and junk for us so it'll be nice)
        double translationVal = MathUtil.applyDeadband(yAxis, Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(xAxis, Constants.stickDeadband);
        double rotationVal;
        
        if (driver.getRawButtonReleased(XboxController.Button.kB.value)) {
            isAligning = true;
        } else if (MathUtil.applyDeadband(rotationAxis, Constants.stickDeadband) > 0) {
            isAligning = false;
        }

        if ((MathUtil.applyDeadband(rotationAxis, Constants.stickDeadband) > 0 && isAligning) || (bButtonPressed && isAligning)) {
            isAligning = false;
        } else if (bButtonPressed && isAligning == false) {
            isAligning = true;
        }

        rotationVal = (isAligning? s_AutoRotateUtil.calculateRotationSpeed():MathUtil.applyDeadband(rotationAxis, Constants.stickDeadband));

        SmartDashboard.putBoolean("IsAligning", isAligning);
        SmartDashboard.putNumber("Translation Val", translationVal);
        SmartDashboard.putNumber("Strave Val", strafeVal);
        SmartDashboard.putNumber("Rotation Value", rotationVal);
        SmartDashboard.putNumber("Gyro", s_DefaultTeleop.s_Swerve.getGyroYaw().getDegrees());

        

        double throttleAxis = driver.getRawAxis(throttle);

        throttleAxis = (Math.abs(throttleAxis) < Constants.stickDeadband) ? .2 : throttleAxis;

        rotationVal = rotationVal * rotationLimiter.calculate(rotationAxis) * (throttleLimiter.calculate(throttleAxis));

        s_DefaultTeleop.s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed).times(throttleLimiter.calculate(throttleAxis) * (5)), rotationVal * Constants.Swerve.maxAngularVelocity, robotCentricSup, true);
    }

    @Override
    public void end(boolean isFinished) {
        s_AutoRotateUtil.end();
    }
}