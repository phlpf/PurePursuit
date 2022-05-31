// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kClimb;
import frc.robot.constants.kPneumatics;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    public ClimberArm outerArm;
    public ClimberArm innerArm;
    private DoubleSolenoid lock = new DoubleSolenoid(kCANIDs.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, kPneumatics.CLIMB_BREAK_2, kPneumatics.CLIMB_BREAK_1);
    private CANSparkMax sidewaysMover;
    public Climber() {
        innerArm = new ClimberArm(kCANIDs.INNER_ANGLE,kCANIDs.INNER_REACH, 
                    kClimb.climbAngleInner, kClimb.climbReachInner, false);
        outerArm = new ClimberArm(kCANIDs.OUTER_ANGLE,kCANIDs.OUTER_REACH, 
                    kClimb.climbAngleOuter, kClimb.climbReachOuter, true);
        sidewaysMover = new CANSparkMax(kCANIDs.SIDEWAYS_MOVER, MotorType.kBrushless);
    }

    public void extendArm(ClimberArm arm, double distance){
        double rotations = distance/kClimb.CLIMB_ROTATION_TO_INCH;
        SmartDashboard.putNumber("Climb Setpoint Reach", rotations);
        arm.setReachSetpoint(rotations);
    }  
    public void rotateArmTo(ClimberArm arm, double angle){
        double rotations = angle/kClimb.CLIMB_ROTATION_TO_DEGREE;
        SmartDashboard.putNumber("Climb Setpoint Angle", rotations);
        arm.setAngleSetpoint(rotations);
    }  
    public void setToCoast(){
        outerArm.setAngleToCoast();
        innerArm.setAngleToCoast();
        outerArm.setReachToCoast();
        innerArm.setReachToCoast();
    }  
    public void setToBrake(){
        outerArm.setAngleToBrake();
        innerArm.setAngleToBrake();
        outerArm.setReachToBrake();
        innerArm.setReachToBrake();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Encoder Out Reach", outerArm.reachEncoder.getPosition());
        SmartDashboard.putNumber("Climb Encoder Out Angle", outerArm.angleEncoder.getPosition()*kClimb.CLIMB_ROTATION_TO_DEGREE);
        SmartDashboard.putNumber("Climb Encoder In Reach", innerArm.reachEncoder.getPosition());
        SmartDashboard.putNumber("Climb Encoder In Angle", innerArm.angleEncoder.getPosition()*kClimb.CLIMB_ROTATION_TO_DEGREE);
        SmartDashboard.putNumber("Climb Setpoint Out Reach", outerArm.getReachSetpoint());
        SmartDashboard.putNumber("Climb Setpoint In Reach", innerArm.getReachSetpoint());
        SmartDashboard.putBoolean("Climb lock", lock.get() == Value.kForward);

        SmartDashboard.putNumber("Sideways Mover Current", sidewaysMover.getOutputCurrent());

        outerArm.periodic();
        innerArm.periodic();
    }

    public void disableAngleSoftLimits(){
        innerArm.disableAngleSoftLimits();
        outerArm.disableAngleSoftLimits();
    }
    public void enableAngleSoftLimits(){
        innerArm.enableAngleSoftLimits();
        outerArm.enableAngleSoftLimits();
    }
    public void disableReachSoftLimits(){
        innerArm.disableReachSoftLimits();
        outerArm.disableReachSoftLimits();
    }
    public void enableReachSoftLimits(){
        innerArm.enableReachSoftLimits();
        outerArm.enableReachSoftLimits();
    }
    public void zeroAngleEncoders(){
        innerArm.zeroAngleEncoders();
        outerArm.zeroAngleEncoders();
    }
    public void startInitialize(){
        innerArm.startInitialize();
        outerArm.startInitialize();
        releaseLock();
    }
    public void endInitialize(){
        innerArm.endInitialize();
        outerArm.endInitialize();
    }
    public void releaseLock(){
        lock.set(Value.kReverse);
    }
    public void extendBreak(){
        lock.set(Value.kForward);
    }
    public void moveSidewaysPOut(double percent){
        sidewaysMover.set(percent);
    }
}
