// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drives;

/** Add your docs here. */
public class PurePursuitHelper {
    final double FORWARD_ANGLE = 3;
    final double FORWARD_SPEED = 0.2;
    final double ROTATION_SPEED = 0.2;
    Drives drives;
    PhotonVisionWrapper cam;
    Rotation2d driveRotation = new Rotation2d();
    double ballRotation = 0;
    double distance = 0;
    Debouncer detectionDebounce = new Debouncer(1, DebounceType.kFalling);
    public PurePursuitHelper(Drives drives){
        this.drives = drives;
        this.cam = drives.getCamera();
    }
    public void update(){
        driveRotation = drives.getRotation();
        ballRotation = cam.getYaw();
        distance = cam.getDistance();
    }
    public double getXSpeed(){
        if(Math.abs(ballRotation) < FORWARD_ANGLE){
            // Forward relative robot
            return FORWARD_SPEED*driveRotation.getCos();
        }
        return 0;
    }
    public double getYSpeed(){
        if(Math.abs(ballRotation) < FORWARD_ANGLE){
            // Forward relative robot
            return  FORWARD_SPEED*driveRotation.getSin();
        }
        return 0;
    }
    public double getRotSpeed(){
        SmartDashboard.putNumber("ballRotation", ballRotation);
        SmartDashboard.putNumber("ROTATIONSPEED", ROTATION_SPEED);
        return -ROTATION_SPEED*(ballRotation/15);
    }
    public boolean isFinished(){
        return !detectionDebounce.calculate(cam.hasTargets());
    }
}
