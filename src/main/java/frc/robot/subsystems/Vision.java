package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.VisionModuleConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Vision extends SubsystemBase{
    
/*
 * splitting into 2 different thoughts. 
 * 
 * Vision for turret:
 *      This will be all of the things necessary for locking a turret onto a zone.
 * 
 * Vision for Robot:
 *      Grabbing the wanted targeted april tag and allowing for odometry to be calculated. math will be dont in the battery class 
 * 
 * All cameras will be looking for an inner and outter april tag.
 * 
 */

    private PhotonCamera cam;
    private double cameraPosX, cameraPosY, cameraPosZ;

    //for the of the field
    private double outisdeTargetYaw;
    //for when scoring
    private double insideTargetYaw;

    private PhotonTrackedTarget trackedTargetIn, trackedTargetOut;

    private int outsideAprilTagID, insideAprilTagID;
    private Pose3d cameraPositionOnField;
    private boolean targetInsideConformation, targetOutsdieConformation;

    private Pose3d camToRobotPosition;
    private Transform3d camOnRobot;

    public Vision(String camName, VisionModuleConstants constants)
    {
        cam = new PhotonCamera(camName);
        outisdeTargetYaw = 0.0;
        insideTargetYaw = 0.0;

        cam.setFPSLimit(30);
        outsideAprilTagID = constants.outsideID;
        insideAprilTagID = constants.insdieID;
        cameraPosX = constants.cameraPosX;
        cameraPosY = constants.cameraPosY;
        cameraPosZ = constants.cameraPosZ;

        Translation3d cameraGridPosition = new Translation3d(
            cameraPosX,
            cameraPosY,
            cameraPosZ
        );
        
        Rotation3d cameraRotationPos = new Rotation3d(0,0,0);

        camToRobotPosition = new Pose3d(cameraGridPosition, cameraRotationPos);
        camOnRobot = new Transform3d( cameraGridPosition, cameraRotationPos);
    }

    private void targetProfile(boolean targetStatus, int targetID, PhotonTrackedTarget target, double yawAngle){
        targetStatus = true;

        cameraPositionOnField = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), Constants.AprilTag.kfieldLayout.getTagPose(targetID).get(), camOnRobot);

        yawAngle = -target.getYaw();
    }

    // this will get called continuously as to update the data fields.  
    private void targeting(int IDin, int IDout)
    {
        var result = cam.getLatestResult();
        if(!result.hasTargets())
        {
            targetInsideConformation = false;
            targetOutsdieConformation = false;
        }

        for(var target : result.getTargets())
        {
            if(target.getFiducialId() == IDin)
            {
                targetProfile(targetInsideConformation, IDin, target, insideTargetYaw);
                
            }
            if(target.getFiducialId() == IDout)
            {
                targetProfile(targetOutsdieConformation, IDout, target, outisdeTargetYaw);
            }
        }

    }

    private void targeting(int ID1){
        
    }
    private void insideTarget(){

    }
    private void outsideTarget(){

    }


    //this can only be called if the target value is true
    public double distanceToTarget(){
        if(targetInsideConformation == true)
        {
            return PhotonUtils.getDistanceToPose(cameraPositionOnField.toPose2d(), Constants.AprilTag.kfieldLayout.getTagPose(insideAprilTagID).get().toPose2d());

        }
        if(targetOutsdieConformation == true)
        {
            return PhotonUtils.getDistanceToPose(cameraPositionOnField.toPose2d(), Constants.AprilTag.kfieldLayout.getTagPose(outsideAprilTagID).get().toPose2d());
        }
        return 0;
    }

    public double angleOffOftarget(){
        if(targetInsideConformation == true)
        {
            return insideTargetYaw;
        }
        if(targetOutsdieConformation == true)
        {
            return outisdeTargetYaw;
        }
        return 0;

    }

    

}
