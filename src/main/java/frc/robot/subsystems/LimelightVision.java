// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.lang.reflect.Member;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionConstants;
import frc.robot.VisionConstants.CameraConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LLPipelines.pipelines;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision. */

  private double llHeartbeatfl;
  private double llHeartbeatLastfl;
  private int samplesfl;

  public boolean limelightExistsfl;

  private double llHeartbeatfr;
  private double llHeartbeatLastfr;
  private int samplesfr;

  public boolean limelightExistsfr;

  private double llHeartbeatr;
  private double llHeartbeatLastr;
  private int samplesr;

  public boolean limelightExistsr;
  private int loopctr;

  public String flname = VisionConstants.CameraConstants.frontLeftCamera.camname;
  public String frname = VisionConstants.CameraConstants.frontRightCamera.camname;
  public String rname = VisionConstants.CameraConstants.rearCamera.camname;

  Optional<Pose3d> temp;

  final int[] autoTagFilter = new int[] { 3, 4, 7, 8 };
  final int[] teleopTagFilter = new int[] { 3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15, 16 };

  public LimelightVision() {

    if (VisionConstants.CameraConstants.rearCamera.isUsed)
      LimelightHelpers.setPipelineIndex(VisionConstants.CameraConstants.rearCamera.camname,
          pipelines.NOTEDET1.ordinal());

    if (VisionConstants.CameraConstants.frontLeftCamera.isUsed)
      setCamToRobotOffset(VisionConstants.CameraConstants.frontLeftCamera);

    if (VisionConstants.CameraConstants.frontRightCamera.isUsed)
      setCamToRobotOffset(VisionConstants.CameraConstants.frontRightCamera);

  }

  public void setAutoTagFilter(String camname) {
    LimelightHelpers.SetFiducialIDFiltersOverride(camname, autoTagFilter);
  }

  public void setTeleopTagFilter(String camname) {
    LimelightHelpers.SetFiducialIDFiltersOverride(camname, teleopTagFilter);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (loopctr > 2)
      loopctr = 0;

    if (RobotBase.isReal()) {
      if (VisionConstants.CameraConstants.frontLeftCamera.isUsed && loopctr == 0) {
        llHeartbeatfl = LimelightHelpers.getLimelightNTDouble(VisionConstants.CameraConstants.frontLeftCamera.camname,
            "hb");
        if (llHeartbeatfl == llHeartbeatLastfl) {
          samplesfl += 1;
        } else {
          samplesfl = 0;
          llHeartbeatLastfl = llHeartbeatfl;
          limelightExistsfl = true;
        }
        if (samplesfl > 5)
          limelightExistsfl = false;

        VisionConstants.CameraConstants.frontLeftCamera.isActive = limelightExistsfl;
      }
      if (VisionConstants.CameraConstants.frontRightCamera.isUsed && loopctr == 1) {
        llHeartbeatfr = LimelightHelpers.getLimelightNTDouble(VisionConstants.CameraConstants.frontRightCamera.camname,
            "hb");
        if (llHeartbeatfr == llHeartbeatLastfr) {
          samplesfr += 1;
        } else {
          samplesfr = 0;
          llHeartbeatLastfr = llHeartbeatfr;
          limelightExistsfr = true;
        }
        if (samplesfr > 5)
          limelightExistsfr = false;

        VisionConstants.CameraConstants.frontRightCamera.isActive = limelightExistsfr;
      }

      if (VisionConstants.CameraConstants.rearCamera.isUsed && loopctr == 2) {
        llHeartbeatr = LimelightHelpers.getLimelightNTDouble(VisionConstants.CameraConstants.rearCamera.camname, "hb");
        if (llHeartbeatr == llHeartbeatLastr) {
          samplesr += 1;
        } else {
          samplesr = 0;
          llHeartbeatLastr = llHeartbeatr;
          limelightExistsr = true;
        }
        if (samplesr > 5)
          limelightExistsr = false;

        VisionConstants.CameraConstants.rearCamera.isActive = limelightExistsr;
      }
    }

    loopctr++;

    SmartDashboard.putBoolean("LL//FrontLeftCamOk", limelightExistsfl);
    SmartDashboard.putBoolean("LL//FrontRightCamOk", limelightExistsfr);
    SmartDashboard.putBoolean("LL//RearCamOk", limelightExistsr);

    boolean allcamsok = VisionConstants.CameraConstants.frontLeftCamera.isUsed && limelightExistsfl
        && VisionConstants.CameraConstants.frontRightCamera.isUsed && limelightExistsfr
        && VisionConstants.CameraConstants.rearCamera.isUsed && limelightExistsr;
    SmartDashboard.putBoolean("LL//CamsOK", allcamsok);
  }

  public void setReefOffset(boolean left) {
    double xOffset = VisionConstants.centerToReefBranch.in(Meters);
    if (left)
      xOffset = -xOffset;
    LimelightHelpers.SetFidcuial3DOffset(
        CameraConstants.frontLeftCamera.camname, xOffset, 0, 0);
  }

  public void setCamToRobotOffset(VisionConstants.CameraConstants.CameraValues cam) {
    LimelightHelpers.setCameraPose_RobotSpace(cam.camname, cam.forward, cam.side, cam.up, cam.roll, cam.pitch, cam.yaw);
  }

  public double getNoteY(double distanceToNote) {
    return distanceToNote * Math.cos(Math.toRadians(90 - getTX().getDegrees()));
  }

  public Pose2d getNotePoseFromCamera(double distanceToNote) {
    double temp = getNoteY(distanceToNote);
    return new Pose2d(distanceToNote, temp, getTX());
  }

  public Transform2d getNoteTransform2dFromCamera(double distanceToNote) {
    double temp = getNoteY(distanceToNote);
    return new Transform2d(distanceToNote, temp, getTX());
  }

  // angle target is from the center
  public Rotation2d getTX() {
    double tx = LimelightHelpers.getTX(rname);
    return Rotation2d.fromDegrees(tx);
  }

  public double getBoundingHorizontalPixels() {
    return LimelightHelpers.getLimelightNTDouble(rname, "thor");
  }

  public double rearCameraTX() {
    return LimelightHelpers.getTX(rname);
  }

  public double rearCameraTY() {
    return LimelightHelpers.getTY(rname);
  }

  public double rearCameraTA() {
    return LimelightHelpers.getTA(rname);
  }

}