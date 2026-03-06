
package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.util.TurretModuleConstants;

public class Turret{
    private int turretID;
    private TurretModuleConstants turretConstants;

    private final SparkMax turretSpark;
    private final SparkMaxConfig turretSparkConfig;

    private final SparkFlex shooterMotorLeft, shooterMotorRight;
    private final SparkFlexConfig shooterMotorLeftConfig, shooterMotorRightConfig;

    /**
     * Create an instance of a turret module, use constants to alter motor controller properties.
     * @param turretID Turret ID Number
     * @param turretConstants Turret Constants to use
     */

  public Turret(int turretID, TurretModuleConstants turretConstants){
    this.turretID = turretID;
    this.turretConstants = turretConstants;

    turretSpark = new SparkMax(this.turretConstants.turnMotorID, MotorType.kBrushless);
    turretSparkConfig = new SparkMaxConfig();
    turretSparkConfig.smartCurrentLimit(40);
    turretSparkConfig.idleMode(IdleMode.kBrake);
    turretSpark.configure(turretSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    shooterMotorLeft = new SparkFlex(this.turretConstants.shooterMotorLeftID, MotorType.kBrushless);
    shooterMotorLeftConfig = new SparkFlexConfig();
    shooterMotorLeftConfig.smartCurrentLimit(80);
    shooterMotorLeftConfig.inverted(this.turretConstants.shooterMotorLeftInverted);
    shooterMotorLeft.configure(shooterMotorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterMotorRight = new SparkFlex(this.turretConstants.shooterMotorRightID, MotorType.kBrushless);
    shooterMotorRightConfig = new SparkFlexConfig();
    shooterMotorRightConfig.smartCurrentLimit(80);
    shooterMotorRightConfig.inverted(this.turretConstants.shooterMotorRightInverted);
    shooterMotorRight.configure(shooterMotorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public int getTurretID(){
        return this.turretID;
    }
}