// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class PhotonVisionWrapper {
    PhotonCamera cam;
    
    public PhotonVisionWrapper(String name){
        cam = new PhotonCamera(name);
    } 
    public double getYaw(){
        var results = cam.getLatestResult();
        return (results.hasTargets())?results.getBestTarget().getYaw():0;
    } 
    public double getDistance(){
        var results = cam.getLatestResult();
        double distance = 0;
        if(results.hasTargets()){
            double x = results.getBestTarget().getCameraToTarget().getX();
            double y = results.getBestTarget().getCameraToTarget().getX();
            distance = Math.sqrt(x*x+y*y);
        }
        return (results.hasTargets())?distance:0;
    }
    public boolean hasTargets(){
        return cam.getLatestResult().hasTargets();
    }
}
