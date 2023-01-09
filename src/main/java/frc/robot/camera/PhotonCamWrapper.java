// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.camera;

import java.util.Optional;

import org.photonvision.PhotonCamera;

/** 
 * Simple wrapper for 
 */
public final class PhotonCamWrapper {
  PhotonCamera cam;

  boolean isConnected;

  //We only attempt to align to fiducial 1
  final int TARGET_FIDUCIAL_ID = 1;

  public PhotonCamWrapper(String cameraName){
      this.cam = new PhotonCamera(cameraName);
      PhotonCamera.setVersionCheckEnabled(false);
  }

  public Optional<Double> getTgtYaw(){
    Optional<Double> retVal = Optional.empty();
      if(cam.isConnected()){
        var results = cam.getLatestResult();
        for(var tgt : results.getTargets()){
            if(tgt.getFiducialId() == TARGET_FIDUCIAL_ID){
                retVal = Optional.of(tgt.getYaw());
                break;
            }
        }
      }
      return retVal;
  }
}
