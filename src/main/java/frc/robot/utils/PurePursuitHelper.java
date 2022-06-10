// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drives;

/** Add your docs here. */
public class PurePursuitHelper {
    final double FORWARD_ANGLE = 2;
    final double CHANGE_ANGLE = 0.25;
    final double FORWARD_SPEED = 0.2;
    final double DEGREE_RANGE = 25;
    Drives drives;
    PhotonVisionWrapper cam;
    Rotation2d driveRotation = new Rotation2d();
    Rotation2d ballRotation = new Rotation2d();
    Rotation2d lastBallRot = new Rotation2d();
    double distance = 0;
    Rotation2d targetRotation = new Rotation2d();
    Debouncer detectionDebounce = new Debouncer(1, DebounceType.kFalling);
    public PurePursuitHelper(Drives drives){
        this.drives = drives;
        this.cam = drives.getCamera();
    }
    public void update(){
        driveRotation = drives.getRotation();
        distance = cam.getDistance();
        double rawBall = cam.getYaw();
        ballRotation = Rotation2d.fromDegrees(rawBall);
        SmartDashboard.putNumber("br", rawBall);
        SmartDashboard.putNumber("br2", ballRotation.getDegrees());
        SmartDashboard.putNumber("tr", targetRotation.getDegrees());
        SmartDashboard.putNumber("dr", driveRotation.getDegrees());
        SmartDashboard.putNumber("tr-dr", targetRotation.getDegrees() - driveRotation.getDegrees());
        if(Math.abs(lastBallRot.minus(ballRotation).getDegrees()) > CHANGE_ANGLE){
            targetRotation = driveRotation.plus(ballRotation);
            SmartDashboard.putNumber("targetRotation", targetRotation.getDegrees());
        }
        lastBallRot = ballRotation;
    }
    public double getXSpeed(){
        if(Math.abs(ballRotation.getDegrees()) < FORWARD_ANGLE){
            // Forward relative robot
            return FORWARD_SPEED*driveRotation.getCos();
        }
        return 0;
    }
    public double getYSpeed(){
        if(Math.abs(ballRotation.getDegrees()) < FORWARD_ANGLE){
            // Forward relative robot
            return  FORWARD_SPEED*driveRotation.getSin();
        }
        return 0;
    }
    public double getRotSpeed(){
        SmartDashboard.putNumber("raw rotSpeed", (targetRotation.minus(driveRotation)).getDegrees());

        double rotSpeed = ((targetRotation.minus(driveRotation)).getDegrees()/DEGREE_RANGE) * -0.5 ; 
        return rotSpeed;
    }
    public boolean isFinished(){
        return !detectionDebounce.calculate(cam.hasTargets());
    }
}
