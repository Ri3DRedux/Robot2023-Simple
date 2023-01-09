// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.camera;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/** 
 * Simple wrapper for 
 */
public final class PhotonCamWrapper {
  PhotonCamera cam;

  boolean isConnected;

  List<CameraPoseObservation> observations;

  final Pose3d fieldPose = new Pose3d(); //Field-referenced orign

  //TODO - set up actual tag locations (possibly though json?)
  final Transform3d tagLocation = new Transform3d( new Translation3d(Units.feetToMeters(54.0), Units.feetToMeters(9.8541), 1.0), new Rotation3d(0,0,0));


  final Transform3d robotToCam;


  public PhotonCamWrapper(String cameraName, Transform3d robotToCam){
      this.cam = new PhotonCamera(cameraName);
      cam.setVersionCheckEnabled(false);
      this.robotToCam = robotToCam;
      this.observations = new ArrayList<CameraPoseObservation>();
  }

  public void update(){

      var res = cam.getLatestResult();
      double observationTime = Timer.getFPGATimestamp() - res.getLatencyMillis();


      observations = new ArrayList<CameraPoseObservation>();

      if(cam.isConnected()){
        List<PhotonTrackedTarget> tgtList = res.getTargets();

        for(PhotonTrackedTarget t : tgtList){
            Transform3d camToTargetTrans = t.getBestCameraToTarget(); //TODO - better apriltag multiple pose arbitration strategy
            Pose3d targetPose = fieldPose.transformBy(tagLocation);
            Pose3d camPose = targetPose.transformBy(camToTargetTrans.inverse());
            Pose2d visionEstPose = camPose.transformBy(robotToCam.inverse()).toPose2d();   
            observations.add(new CameraPoseObservation(observationTime, visionEstPose, 1.0)); //TODO - add trustworthiness scale by distance - further targets are less accurate  
        }
      }
  }

  public List<CameraPoseObservation> getCurObservations(){
      return observations;
  }

  public int getCurTargetCount(){
      return observations.size();
  }
}
