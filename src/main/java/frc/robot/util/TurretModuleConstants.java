package frc.robot.util;

public class TurretModuleConstants {
    public final int turnMotorID;
    public final int shooterMotorLeftID, shooterMotorRightID;
    public final double maxAngle, minAngle;
    public final double idleRPM, maxRPM;
    public final boolean turnMotorInverted, shooterMotorLeftInverted, shooterMotorRightInverted;

    /**
     * Turret Module Constants to be used when creating turret modules;
     * @param turnMotorID
     * @param shooterMotorLeftID
     * @param shooterMotorRightID
     * @param maxAngle
     * @param minAngle
     * @param turnMotorInverted
     * @param shooterMotorLeftInverted
     */
    
    public TurretModuleConstants(int turnMotorID, int shooterMotorLeftID, int shooterMotorRightID, double maxAngle, double minAngle, double idleRPM, double maxRPM, boolean turnMotorInverted, boolean shooterMotorLeftInverted){
        this.turnMotorID = turnMotorID;
        this.shooterMotorLeftID = shooterMotorLeftID;
        this.shooterMotorRightID = shooterMotorRightID;
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.idleRPM = idleRPM;
        this.maxRPM = maxRPM;
        this.turnMotorInverted = turnMotorInverted;
        this.shooterMotorLeftInverted = shooterMotorLeftInverted;
        this.shooterMotorRightInverted = !shooterMotorLeftInverted;
    }
    
}
