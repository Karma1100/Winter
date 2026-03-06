package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;


public class TurretBattery extends SubsystemBase{
    private static TurretBattery turt;

    Vision cameraCenter, cameraLeft, cameraRight;

    Turret turretLeft, turretRight;



    public TurretBattery(){
        cameraCenter = new Vision("AnomCenterCamera", VisionConstants.centerCamera.constants);
        cameraLeft = new Vision("AnomLeftCamera", VisionConstants.leftCamera.constants);
        cameraRight = new Vision("AnomRightCamera", VisionConstants.rightCamera.constants);

        turretLeft = new Turret(0, null);


    }
    public static TurretBattery getInstance(){
        if(turt == null){
            turt = new TurretBattery();
        }

        return turt;
    }

    






}
