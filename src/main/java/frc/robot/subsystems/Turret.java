
package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.TurretModuleConstants;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    private int turretID;
    private TurretModuleConstants turretConstants;

    private Timer timeManager;

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
    turretSparkConfig.encoder.positionConversionFactor(Constants.Turret.encoderConverionFactor);
    turretSparkConfig.closedLoop.pid(6,0,0.4);
    turretSparkConfig.idleMode(IdleMode.kBrake);
    turretSpark.configure(turretSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turretSpark.getEncoder().setPosition(Constants.Turret.turretHome);
    
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
    
    public double getAngle(){
        return (turretSpark.getEncoder().getPosition() * Math.PI);
    }

    public double getThrow(){
        return getAngle() * Constants.Turret.turretRadius;

    }
        
    //turret must be sent to center and then wait for the function be 0(0 is the center)
    public double circleSearchGenerator(double speedModifyer){
        double out = Constants.Turret.turretCircleThrow * Math.sin(timeManager.getFPGATimestamp() * speedModifyer);
        return out;
    }

    /**
     * checks to see if angle is within boarders and then sends
     * @param desiredangle this is formated with the understanding that left is negitive 180 and right is positive 180 
     * @return
     */
    public Command setPosAngle(double desiredangle){
        
        double turretPos = desiredangle / 180;

        return runOnce(()->{
            if((turretPos > Constants.Turret.leftSoftBound) && (turretPos < Constants.Turret.rightSoftBound)){
                turretSpark.getClosedLoopController().setSetpoint(turretPos, ControlType.kPosition);
            }
        });
    }

    //desired arc position must be in the total arch swing. the desired pizza crust must come from the existing pizza crust, not from the non existrant(no go zone)not form the non existant pizza.
    public Command setPosPosition(double desiredPosition){
        double turretPos = desiredPosition / (Constants.Turret.turretRadius * .5); //going from arc position to graph position
        return runOnce(()->{
            if((turretPos > Constants.Turret.leftSoftBound) && (turretPos < Constants.Turret.rightSoftBound)){
                turretSpark.getClosedLoopController().setSetpoint(desiredPosition, ControlType.kPosition);
            }
        });
    }
    //needs vision 
    public Command circleSearch(boolean targetFound){
        return run(() ->{
            if(targetFound == false)
            {
                setPosPosition(circleSearchGenerator(Constants.Turret.speedModifyer));
            }

        });
    }
    
















}