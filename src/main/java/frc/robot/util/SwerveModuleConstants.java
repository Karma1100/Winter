package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID, turnMotorID;
    public final Rotation2d angleOffset;
    public final boolean driveInverted, turnInverted;
    
        /**
         * Swerve Module Constants to be used when creating swerve modules;
         * @param driveMotorID
         * @param turnMotorID
         * @param angleOffset
         * @param driveInverted
         * @param turnInverted
     */
    public SwerveModuleConstants(int driveMotorID, int turnMotorID, Rotation2d angleOffset, boolean driveInverted, boolean turnInverted){
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.angleOffset = angleOffset;
        this.driveInverted = driveInverted;
        this.turnInverted = turnInverted;
    }
}
