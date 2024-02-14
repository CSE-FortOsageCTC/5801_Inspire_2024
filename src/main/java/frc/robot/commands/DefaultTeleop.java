package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.AutoRotateUtil;
import frc.robot.Constants;
import frc.robot.subsystems.DefaultTeleopSub;

public class DefaultTeleop extends Command{


    private DefaultTeleopSub s_DefaultTeleop;
    private AutoRotateUtil s_AutoRotateUtil;

    private Debouncer buttonDebouncer = new Debouncer(0.25);

    private int translationSup;
    private int strafeSup;
    private int rotationSup;
    private boolean robotCentricSup;
    private Joystick driver;
    private Joystick operator;

    private PIDController rotationPidController = new PIDController(0, 0, 0);
    public boolean isAligning = false;
    private Pose2d alignPose;
    private boolean bButtonPressed;
    private boolean aButtonPressed;
    private boolean xButtonPressed;
    private boolean yButtonPressed;

    public DefaultTeleop(Joystick driver, Joystick operator) {
        s_DefaultTeleop = DefaultTeleopSub.getInstance();
        s_AutoRotateUtil = new AutoRotateUtil(0); 
        this.driver = driver;
        this.operator = operator;
        translationSup = XboxController.Axis.kLeftY.value;
        strafeSup = XboxController.Axis.kLeftX.value;
        rotationSup = XboxController.Axis.kRightX.value;
        robotCentricSup = true;
        bButtonPressed = buttonDebouncer.calculate(driver.getRawButtonReleased(XboxController.Button.kB.value));
        aButtonPressed = buttonDebouncer.calculate(driver.getRawButtonReleased(XboxController.Button.kA.value));
        xButtonPressed = buttonDebouncer.calculate(driver.getRawButtonReleased(XboxController.Button.kX.value));
        yButtonPressed = buttonDebouncer.calculate(driver.getRawButtonReleased(XboxController.Button.kY.value));
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

        double yAxis = -driver.getRawAxis(translationSup);
        double xAxis = -driver.getRawAxis(strafeSup);
        double rotationAxis = driver.getRawAxis(rotationSup);
        Pose2d botPose = s_DefaultTeleop.s_Limelight.getBotPose(); // gets botpose based on approximated position from limelight
        double xDiff = botPose.getX() - alignPose.getX(); // gets distance of x between robot and target
        double yDiff = botPose.getY() - alignPose.getY(); // gets distance of y between robot and target
        SmartDashboard.putNumber("Bot Pose X", botPose.getX());
        SmartDashboard.putNumber("Bot Pose Y", botPose.getY());
        double angle = Units.radiansToDegrees(Math.atan2(xDiff, yDiff));
        SmartDashboard.putNumber("Align Swerve Angle", angle);
        s_AutoRotateUtil.updateTargetAngle(angle - 90); // updates pid angle setpoint to the angle that faces towards the target by using arctangent2 (which keeps negative and junk for us so it'll be nice)
        double translationVal = MathUtil.applyDeadband(yAxis, Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(xAxis, Constants.stickDeadband);
        double rotationVal;

        if ((isAligning && MathUtil.applyDeadband(rotationAxis, Constants.stickDeadband) != 0) || isAligning && ((bButtonPressed || xButtonPressed || yButtonPressed || aButtonPressed))) {
            isAligning = false;
        } else if (!isAligning && (bButtonPressed || xButtonPressed || yButtonPressed || aButtonPressed)) {
            isAligning = true;
        }

        if (bButtonPressed) {
            AlignPosition.setPosition(AlignPosition.AmpPos);
        } else if (aButtonPressed) {
            AlignPosition.setPosition(AlignPosition.ClimbPos);
        } else if (xButtonPressed) {
            AlignPosition.setPosition(AlignPosition.SpeakerPos);
        } // find a way to bind each source side to a button scheme that works

        rotationVal = (isAligning? s_AutoRotateUtil.calculateRotationSpeed():MathUtil.applyDeadband(rotationAxis, Constants.stickDeadband));

        SmartDashboard.putBoolean("IsAligning", isAligning);
        SmartDashboard.putNumber("Translation Val", translationVal);
        SmartDashboard.putNumber("Strave Val", strafeVal);
        SmartDashboard.putNumber("Rotation Value", rotationVal);
        SmartDashboard.putNumber("Gyro", s_DefaultTeleop.s_Swerve.getGyroYaw().getDegrees());

        s_DefaultTeleop.s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), rotationVal * Constants.Swerve.maxAngularVelocity, robotCentricSup, true);

    }

    @Override
    public void end(boolean isFinished) {
        s_AutoRotateUtil.end();
    }
}