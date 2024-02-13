package frc.robot.commands;

import com.ctre.phoenix6.Timestamp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoRotateUtil;
import frc.robot.Constants;
import frc.robot.subsystems.SkyLimelight;
import frc.robot.subsystems.Swerve;

/**
 * This class detects the "april tag" and positions the robot infront of the node/April Tag
 */
public class NaviToPos extends Command {

    private PIDController yTranslationPidController;
    private PIDController xTranslationPidController;
    
    private Swerve s_Swerve;
    private SkyLimelight limelight = SkyLimelight.getInstance();
    private AutoRotateUtil autoUtil;
    private double targetX;
    private double targetY;
    private double targetRot;

    private SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2);
    private SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(2);

    /**
     * Constructer for NaviToPos
     * 
     * @param s_Swerve swerve subsystem to be aligned with the april tag
     */
    public NaviToPos(  double targetX, double targetY, double targetRot) {
        
        this.s_Swerve = Swerve.getInstance();
        this.targetRot = targetRot;
        this.autoUtil = new AutoRotateUtil(targetRot);
        addRequirements(s_Swerve);
        this.targetX = targetX;
        this.targetY = targetY;
        // creating yTranslationPidController and setting the tolerance and setpoint
        yTranslationPidController = new PIDController(.7, 0, 0);
        yTranslationPidController.setTolerance(.05);
        yTranslationPidController.setSetpoint(0);
        
        // creating xTranslationPidController and setting the tolerance and setpoint
        xTranslationPidController = new PIDController(.7 , 0, 0);
        xTranslationPidController.setTolerance(.05);//5);
        xTranslationPidController.setSetpoint(0);
        
        
    

        // puts the value of P,I and D onto the SmartDashboard
        // Will remove later
        SmartDashboard.putNumber("AlignP", 0.3);
        SmartDashboard.putNumber("AlignI", 0);
        SmartDashboard.putNumber("AlignD", 0.00035);
    }



    @Override
    public void execute() {
        // gets value of P,I and D from smartdashboard
        // will be removed
        double kP = SmartDashboard.getNumber("AlignP", 0);
        double kI = SmartDashboard.getNumber("AlignI", 0);
        double kD = SmartDashboard.getNumber("AlignD", 0);

        double rotationkP = SmartDashboard.getNumber("RotationP", 0);
        double rotationkI = SmartDashboard.getNumber("RotationI", 0);
        double rotationkD = SmartDashboard.getNumber("RotationD", 0);
        
        // sets the PID values for the PIDControllers
        yTranslationPidController.setPID(kP, kI, kD);
        xTranslationPidController.setPID(kP, kI, kD);
        // xTranslationPidController.setPID(kP, kI, kD);



        double xValue = limelight.getX(); //gets the limelight X Coordinate
        double areaValue = limelight.getArea(); // gets the area percentage from the limelight
        double angularValue = limelight.getSkew();
        // sets current position
        Pose2d botPose = limelight.getBotPose(); 
        double botX = botPose.getX();
        double botY = botPose.getY();
        if (botX == 0 && botY == 0){
            botPose = s_Swerve.getEstimatedPosition();
            botX = botPose.getX();
            botY = botPose.getY();

        }
        else{
            s_Swerve.updateWithVision(botPose, Timer.getFPGATimestamp());
            SmartDashboard.putNumber("Current X", botX);
            SmartDashboard.putNumber("Current Y", botY); 
        }
        SmartDashboard.putNumber("Limelightta", areaValue);

        SmartDashboard.putNumber("Xvalue", xValue);
        SmartDashboard.putNumber("Areavalue", areaValue);
        SmartDashboard.putNumber("AngularValue", angularValue);
        SmartDashboard.putNumber("NaviToPosSetpoint", xTranslationPidController.getSetpoint());
        //SmartDashboard.putNumber("Ts0", limelight.getSkew0());
        //SmartDashboard.putNumber("Ts1", limelight.getSkew1());
        //SmartDashboard.putNumber("Ts2", limelight.getSkew2());


        // Calculates the x and y speed values for the translation movement
        double ySpeed = MathUtil.clamp(yTranslationPidController.calculate(botY - targetY), -Constants.Swerve.maxSpeed, Constants.Swerve.maxSpeed); //TODO:should be changed to max speed at some point 
        ySpeed = yTranslationPidController.atSetpoint() ? 0 : ySpeed;
        ySpeed = ySpeedLimiter.calculate(ySpeed);
        double xSpeed = MathUtil.clamp(xTranslationPidController.calculate(botX - targetX), -Constants.Swerve.maxSpeed, Constants.Swerve.maxSpeed);//TODO:should be changed to max speed at some point
        xSpeed = xTranslationPidController.atSetpoint() ? 0 : xSpeed;
        xSpeed = xSpeedLimiter.calculate(xSpeed);
        double angularSpeed = autoUtil.calculateRotationSpeed();
        angularSpeed = autoUtil.isFinished() ? 0: angularSpeed;
        
        SmartDashboard.putNumber("AlignXSpeed", xSpeed);
        SmartDashboard.putNumber("AlignYSpeed", ySpeed);

        // moves the swerve subsystem
        Translation2d translation = new Translation2d(-xSpeed, -ySpeed).times(Constants.Swerve.maxSpeed); 
        double rotation = angularSpeed * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, true, true);

    }


    @Override
    public boolean isFinished() {

        //checks if the Swerve subsystem is within the given position tolerance
        SmartDashboard.putBoolean("AtSetPoint", yTranslationPidController.atSetpoint());
        SmartDashboard.putBoolean("AtSetPoint", xTranslationPidController.atSetpoint());
        return yTranslationPidController.atSetpoint() && xTranslationPidController.atSetpoint(); // && autoUtil.isFinished();
    }

    @Override
    public void end(boolean end) {

        // tells the swerve subsystem to stop
        Translation2d translation = new Translation2d(0, 0).times(Constants.Swerve.maxSpeed);
        s_Swerve.drive(translation, 0, true, true);
        xTranslationPidController.reset();
        yTranslationPidController.reset();
    }
}