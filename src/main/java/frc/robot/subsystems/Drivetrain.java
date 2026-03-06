
package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.PathPlannerLogging;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.SwerveDriveConstants;

public class Drivetrain extends SubsystemBase{
    private static Drivetrain drivetrain;

    private SwerveModuleState[] swerveModuleStates;
    private SwerveModulePosition[] swerveModulePositions;

    private final Mk4TTBSwerve[] swerveModules;
    private final Mk4TTBSwerve frontLeftSwerveModule, backRightSwerveModule, backLeftSwerveModule, frontRightSwerveModule;

    private double lastestChassisSpeed;

    private final ADIS16470_IMU gyro;
    private double heading;
    private boolean isFlipped;

    private boolean useHeadingCorrection;
    private Rotation2d correctHeadingTargetHeading;
    private Timer correctHeadingTimer;
    private double correctHeadingPreviousTime;
    private double correctHeadingOffTime;

    private Field2d m_field = new Field2d();
    private final SwerveDrivePoseEstimator odometry;

    private final StructArrayPublisher<SwerveModuleState> publisher;
    private final StructPublisher<Pose2d> posPublisher;

    RobotConfig robotConfig;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.

    public Drivetrain(){

        try{
            robotConfig = RobotConfig.fromGUISettings();
        }catch (Exception e){
            e.printStackTrace();
        }

        SmartDashboard.putData("Field",m_field);
        posPublisher = NetworkTableInstance.getDefault().getStructTopic("Robot Current Pose", Pose2d.struct).publish();


        publisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

        frontLeftSwerveModule = new Mk4TTBSwerve(0, Swerve.Mod0.constants);
        frontRightSwerveModule = new Mk4TTBSwerve(1, Swerve.Mod1.constants);
        backLeftSwerveModule = new Mk4TTBSwerve(2, Swerve.Mod2.constants);
        backRightSwerveModule = new Mk4TTBSwerve(3 , Swerve.Mod3.constants);

        swerveModules = new Mk4TTBSwerve[] {
            frontLeftSwerveModule,
            frontRightSwerveModule,
            backRightSwerveModule,
            backLeftSwerveModule
            };

        swerveModulePositions = new SwerveModulePosition[] {
            frontLeftSwerveModule.getPosition(),
            frontRightSwerveModule.getPosition(),
            backRightSwerveModule.getPosition(),
            backLeftSwerveModule.getPosition(),
            };

        gyro = new ADIS16470_IMU();
        isFlipped = false;

        lastestChassisSpeed = 0.0;

        odometry = new SwerveDrivePoseEstimator(
        SwerveDriveConstants.kinematics, 
        getHeadingAsRotation2d(), 
            new SwerveModulePosition[]
            {
                frontLeftSwerveModule.getPosition(),
                frontRightSwerveModule.getPosition(),
                backRightSwerveModule.getPosition(),
                backLeftSwerveModule.getPosition()
                }, 
                new Pose2d(), 
                VecBuilder.fill(0.1, 0.1, 0.1), 
                VecBuilder.fill(0.9,0.9,0.9));

        useHeadingCorrection = true;
        correctHeadingTimer = new Timer();
        correctHeadingTimer.start();
        correctHeadingPreviousTime = 0.0;
        correctHeadingOffTime = 0.0;
        correctHeadingTargetHeading = getHeadingAsRotation2d();

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotChassisSpeeds,
            (speeds, feedfowards) -> autoDrive(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(0.1,0,0),
                new PIDConstants(0.4,0,0)
            ),
            robotConfig,
            ()-> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public static Drivetrain getInstance(){
        if(drivetrain == null){
            drivetrain = new Drivetrain();
        }

        return drivetrain;
    }

    public double getSpeed(){
        return lastestChassisSpeed;
    }

    public void setUseHeadingCorrection(boolean enable){
        useHeadingCorrection = enable;
    }

    public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveConstants.kRealMaxSpeedMPS); //12.5 per SDS for L1
    
        // for(int i=0; i < swerveModules.length; i++){
        //     swerveModules[i].setDesiredState(swerveModuleStates[i]);
        // }
        for(Mk4TTBSwerve module : swerveModules){
            module.setDesiredState(swerveModuleStates[module.getModuleNumber()]);
        }

    }

    public void stopSwerveModules(){
        for(Mk4TTBSwerve module : swerveModules){
            module.stop();
        }
    }

    public ChassisSpeeds getRobotChassisSpeeds(){
        return SwerveDriveConstants.kinematics.toChassisSpeeds(
            frontLeftSwerveModule.getState(),
            frontRightSwerveModule.getState(),
            backRightSwerveModule.getState(), 
            backLeftSwerveModule.getState()
            );
    }

    private ChassisSpeeds correctHeading(ChassisSpeeds desiredSpeed){
        double correctHeadingCurrentTime = correctHeadingTimer.get();
        double dt = correctHeadingCurrentTime - correctHeadingPreviousTime;

        double vr = desiredSpeed.omegaRadiansPerSecond;
        double v = Math.sqrt(Math.pow(desiredSpeed.vxMetersPerSecond, 2) + Math.pow(desiredSpeed.vyMetersPerSecond, 2));

        if(vr > 0.01 || vr < -0.01){
            correctHeadingOffTime = correctHeadingCurrentTime;
            correctHeadingTargetHeading = getHeadingAsRotation2d();
            return desiredSpeed;
        }
        if(correctHeadingCurrentTime - correctHeadingOffTime < 0.5){
            correctHeadingTargetHeading = getHeadingAsRotation2d();
            return desiredSpeed;
        }
        if (v < 0.05){
            correctHeadingTargetHeading = getHeadingAsRotation2d();
            return desiredSpeed;
        }

        correctHeadingTargetHeading = correctHeadingTargetHeading.plus(new Rotation2d(vr * dt));
        Rotation2d currentHeading = getHeadingAsRotation2d();

        Rotation2d deltaHeading = correctHeadingTargetHeading.minus(currentHeading);

        if(Math.abs(deltaHeading.getDegrees()) < SwerveDriveConstants.kHeadingCorrectionTolerance){
            return desiredSpeed;
        }

        double correctedVr = deltaHeading.getRadians() / dt * SwerveDriveConstants.kHeadingCorrectionP;

        correctHeadingPreviousTime = correctHeadingCurrentTime;
        return new ChassisSpeeds(
            desiredSpeed.vxMetersPerSecond,
            desiredSpeed.vyMetersPerSecond,
            correctedVr);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerOfRoation){
        if(DriverStation.isTeleop()){
            for (Mk4TTBSwerve module : swerveModules){
                if (module.getDriveIdleMode().equals(IdleMode.kBrake)){
                    module.toggleDriveIdleMode();
                }
            }
        }
        double adjustedRotation = Constants.SwerveDriveConstants.MAXROTATIONRATE * rotation; // Max turn rate in Radians


        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), adjustedRotation);
        ChassisSpeeds robotRelativeSpeeds;

        if(useHeadingCorrection){
            fieldRelativeSpeeds = correctHeading(fieldRelativeSpeeds);
        }

        if(fieldOriented){
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, 
                                Rotation2d.fromDegrees((isFlipped ? 180 : 0) + getHeading()));
        }else{
            robotRelativeSpeeds = fieldRelativeSpeeds;
        }


        lastestChassisSpeed = Math.sqrt(Math.pow(robotRelativeSpeeds.vxMetersPerSecond, 2) + Math.pow(robotRelativeSpeeds.vyMetersPerSecond, 2));

        swerveModuleStates = SwerveDriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds, centerOfRoation);

        setSwerveModuleStates(swerveModuleStates);
    }

    public void autoDrive(ChassisSpeeds speeds){
        if(DriverStation.isAutonomous()){
            for (Mk4TTBSwerve module : swerveModules){
                if (module.getDriveIdleMode().equals(IdleMode.kCoast)){
                    module.toggleDriveIdleMode();
                }
            }
        }

        //Pathplanner example code
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] targetStates = SwerveDriveConstants.kinematics.toSwerveModuleStates(targetSpeeds);
        setSwerveModuleStates(targetStates);
    }

    public double getHeading(){
        heading = gyro.getAngle(gyro.getYawAxis());
        return Math.IEEEremainder(heading, 360);
    }

    public Rotation2d getHeadingAsRotation2d(){
        heading = getHeading();
        return Rotation2d.fromDegrees(heading);
    }

    public void resetPose(Pose2d pose){
        resetGyro();
        odometry.resetPosition(getHeadingAsRotation2d(),swerveModulePositions, pose);
    }

    public void resetGyro(){
        gyro.reset();   
    }

    public Pose2d getPose(){
        return odometry.getEstimatedPosition();
    }

    public void updateOdometry(){
        odometry.updateWithTime(Timer.getFPGATimestamp(), getHeadingAsRotation2d(), swerveModulePositions);
        //odometry.update(correctHeadingTargetHeading, swerveModulePositions);
        m_field.setRobotPose(odometry.getEstimatedPosition());
        posPublisher.set(getPose());
    }

    public void setFlipped(){
        isFlipped = Math.abs(getPose().getRotation().getDegrees()) < 90;
    }

    public void setFlipped(boolean bool){
        isFlipped = bool;
    }

    public boolean getFlipped(){
        return isFlipped;
    }

    @Override
    public void periodic(){
        for (Mk4TTBSwerve module : swerveModules){
            module.putSmartDashboard();
            module.getPosition();
        }

        publisher.set(new SwerveModuleState[]{
            frontLeftSwerveModule.getState(),
            frontRightSwerveModule.getState(),
            backLeftSwerveModule.getState(),
            backRightSwerveModule.getState()
        });

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Gyro Heading", getHeading());
        SmartDashboard.putNumber("Gyro Pitch", gyro.getAngle(gyro.getPitchAxis()));
        SmartDashboard.putNumber("Gyro Roll", gyro.getAngle(gyro.getRollAxis()));
        updateOdometry();
    }
}
