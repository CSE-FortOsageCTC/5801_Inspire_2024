package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.AlignPosition;
import frc.robot.AutoRotateUtil;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;



public class Swerve extends SubsystemBase{
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private SwerveDrivePoseEstimator swerveEstimator;
    private static Swerve swerve;
    public double gyroOffset;
    public PIDController rotationPidController;
    public PIDController yTranslationPidController;
    public PIDController xTranslationPidController;
    public SkyLimelight s_Limelight;
    public FloorLimelight f_Limelight;
    public AutoRotateUtil s_AutoRotateUtil;
    private Pair<Double, Double> speakerCoordinate;
    public Debouncer pieceSeenDebouncer;

    public static Swerve getInstance() {
        if (swerve == null) {
            swerve = new Swerve();
        }
        return swerve;
    }

    private Swerve() {
        pieceSeenDebouncer = new Debouncer(.1, Debouncer.DebounceType.kBoth);
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        //gyro.setYaw(0);
        s_Limelight = SkyLimelight.getInstance();
        f_Limelight = FloorLimelight.getInstance();
        s_AutoRotateUtil = new AutoRotateUtil(0);

        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        swerveEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d(0, 0, new Rotation2d()));

        swerveEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.005, 0.005, Units.degreesToRadians(.5)));

         AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(.5,0,0 ), // Translation PID constants
                        new PIDConstants(.08,0,0), // Rotation PID constants
                        Constants.Swerve.maxSpeed, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this); // Reference to this subsystem to set requirements

        } 
    public void updatePoseEstimator() {
        swerveEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());
    }

    public Pose2d getEstimatedPosition(){
        return swerveEstimator.getEstimatedPosition();
        // return swerveOdometry.getPoseMeters();
    }

    public void updateWithVision(Pose2d pose2d, double timestamp){
        Pose2d test = new Pose2d(pose2d.getTranslation(), Rotation2d.fromDegrees(180));
        swerveEstimator.addVisionMeasurement(test, timestamp);
        //setHeading(Rotation2d.fromDegrees(gyroOffset-180));
        // swerveEstimator.resetPosition(getGyroYaw(), getModulePositions(), test);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        SmartDashboard.putNumber("Estimated x Pose", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Estimated y Pose", swerveOdometry.getPoseMeters().getY());
        return swerveEstimator.getEstimatedPosition();
        // return swerveEstimator.getEstimatedPosition();
    }
    public Pose2d getLimelightBotPose(){
        if (s_Limelight.getArea() >= 0.15 || RobotState.isDisabled()) {
            Pose2d currentPose = getEstimatedPosition();
            Pose2d visionPose = s_Limelight.getBotPose();

            // if ((visionPose.getX() != 0 && visionPose.getY() != 0 && Math.abs(currentPose.getX() - visionPose.getX()) < 1 && Math.abs(currentPose.getY() - visionPose.getY()) < 1) || RobotState.isDisabled()) {
                updateWithVision(visionPose, s_Limelight.getLastBotPoseTimestamp());
                return visionPose;
            // } else {
            //     return currentPose;
            // }
        }

        return getEstimatedPosition();
        // Pose2d botPose = s_Limelight.getBotPose(); 
        // double botX = botPose.getX();
        // double botY = botPose.getY();

        // if (botX == 0 && botY == 0){
        //     botPose = getEstimatedPosition();
        //     System.out.println(botPose);
        // }
        // else{
        //     updateWithVision(botPose, s_Limelight.getLastBotPoseTimestamp());
        //     System.out.println(botPose);
        // }
        // return botPose;
    }

    public void setPose(Pose2d pose) {
        swerveEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        // swerveEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return swerveEstimator.getEstimatedPosition().getRotation();
    }

    public void setHeading(Rotation2d heading){
        Rotation2d gyroYaw = getGyroYaw();
        swerveEstimator.resetPosition(gyroYaw, getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
        gyroOffset = gyroYaw.getDegrees() - heading.getDegrees();
    }

    public void zeroHeading(){
        Rotation2d gyroYaw = getGyroYaw();
        swerveEstimator.resetPosition(gyroYaw, getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
        gyroOffset = gyroYaw.getDegrees();
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public double getGyroRoll() {
        return gyro.getRoll().getValueAsDouble();
    }
    public double correctedYaw() {
        return (((gyro.getYaw().getValue() - gyroOffset) % 360) + 360) % 360;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }
    
    public ChassisSpeeds getEstimatedFieldRelativeSpeeds(){
        ChassisSpeeds speed = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
        speed = ChassisSpeeds.fromRobotRelativeSpeeds(speed, getHeading());
        return speed;
    }
    
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){

        SmartDashboard.putNumber("omega radians per second", robotRelativeSpeeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("x speed", robotRelativeSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("y speed", robotRelativeSpeeds.vyMetersPerSecond);
        // robotRelativeSpeeds.omegaRadiansPerSecond = rotateToSpeaker();
        if (AlignPosition.getPosition() == AlignPosition.SpeakerPos){
            robotRelativeSpeeds.omegaRadiansPerSecond =  rotateToSpeaker();
        }
        
        else if (AlignPosition.getPosition() == AlignPosition.AutoPickup){
            robotRelativeSpeeds.omegaRadiansPerSecond =  rotateToNote();        
        }

        else
        {
            robotRelativeSpeeds.omegaRadiansPerSecond =  0;
        }

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, .02);
        SwerveModuleState[] setpointStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.Swerve.maxSpeed);

        for(int i = 0; i < 4; i++){
            mSwerveMods[i].setDesiredState(setpointStates[i], false);
        }
    }

    public double rotateToSpeaker(){

        Pose2d botPose = getLimelightBotPose();

        double xDiff = botPose.getX() - AlignPosition.getAlignPose().getX(); // gets distance of x between robot and target
        double yDiff = botPose.getY() - AlignPosition.getAlignPose().getY();
        
        double angle = 180 + Units.radiansToDegrees(Math.atan2(yDiff, xDiff));

        double output = (((angle - correctedYaw() )) + 360) % 360;
        SmartDashboard.putNumber("Speaker Diff Output", output);

        s_AutoRotateUtil.updateTargetAngle(output); //why are we subtracting 90? idk man it just works ¯\_(ツ)_/¯
        
        return s_AutoRotateUtil.calculateRotationSpeed();
    }

    public double rotateToAmp() {
        double headingError = correctedYaw() - AlignPosition.getAlignPose().getRotation().getDegrees();

        s_AutoRotateUtil.updateTargetAngle(headingError);
        
        return -s_AutoRotateUtil.calculateRotationSpeed();
    }

    public double rotateToSource() {
        double headingError = getGyroYaw().getDegrees() - AlignPosition.getAlignPose().getRotation().getDegrees();

        s_AutoRotateUtil.updateTargetAngle(headingError);
        
        return s_AutoRotateUtil.calculateRotationSpeed();
    }

    public double rotateToNote(){
        boolean noteInView = f_Limelight.hasTag();

        noteInView = pieceSeenDebouncer.calculate(noteInView);

        if (noteInView){
            s_AutoRotateUtil.updateTargetAngle(-f_Limelight.getX()/2);
            return s_AutoRotateUtil.calculateRotationSpeed();
        }
        
        return 0;
    }
    public Translation2d noteTranslation(){
        double xValue = f_Limelight.getX(); //gets the limelight X Coordinate
        double areaValue = f_Limelight.getArea();
         // creating yTranslationPidController and setting the toleance and setpoint
         yTranslationPidController = new PIDController(0, 0, 0);
         yTranslationPidController.setTolerance(1);
         yTranslationPidController.setSetpoint(0);
         
         // creating xTranslationPidController and setting the toleance and setpoint
         xTranslationPidController = new PIDController(0, 0, 0);
         xTranslationPidController.setTolerance(0);
         xTranslationPidController.setSetpoint(0);
 
        if (pieceSeenDebouncer.calculate(f_Limelight.hasTag())){

            // Calculates the x and y speed values for the translation movement
            double ySpeed = yTranslationPidController.calculate(xValue);
            double xSpeed = xTranslationPidController.calculate(areaValue);
            
            Translation2d translation = new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed);

            return translation;
        }
        else{
            return new Translation2d(0,0);
        }
    }

    public void resetAutoRotateUtil(){
        s_AutoRotateUtil.end();
    }
    
    @Override
    public void periodic(){
        //swerveOdometry.update(getGyroYaw(), getModulePositions());

        updatePoseEstimator();

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        
        double odometryX = swerveOdometry.getPoseMeters().getX();
        double odometryY = swerveOdometry.getPoseMeters().getY();
        SmartDashboard.putNumber("Odometry X", odometryX);
        SmartDashboard.putNumber("Odometry Y", odometryY);
    }
}