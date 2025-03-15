package frc.robot.subsystems.Vision;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class Limelight extends SubsystemBase {

    public Limelight(String limelightName){
    }
@Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.setSmartDashboardType("Limelight");
    builder.addBooleanProperty("Has Target", () -> LimelightHelpers.getTV("limey"), null);
      if(true){
        builder.addDoubleProperty("TX", () -> LimelightHelpers.getTX("limey"),null);
        builder.addDoubleProperty("TY", () -> LimelightHelpers.getTY("limey"), null);
      }
  }
}
