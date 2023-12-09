package frc.robot.subsystems.drive.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.sim.MAXSwerveModuleSim;
import frc.robot.sim.SimulationContext;

public class SimModuleIO implements ModuleIO {
  /** Hardware simulation for the swerve module. */
  private final MAXSwerveModuleSim sim;

  public SimModuleIO(MAXSwerveModuleSim sim) {
    this.sim = sim;
  }

  public SimModuleIO() {
    this(new MAXSwerveModuleSim());
  }

  @Override
  public double getWheelVelocity() {
    return sim.getWheelVelocity();
  }

  @Override
  public double getWheelDistance() {
    return sim.getDistanceTraveled();
  }

  @Override
  public Rotation2d getModuleRotation() {
    return sim.getModuleAngle();
  }

  @Override
  public void setDesiredState(SwerveModuleState state) {
    sim.setDesiredState(state);
  }

  @Override
  public void stop() {
    setDesiredState(new SwerveModuleState(0, getModuleRotation()));
  }

  @Override
  public void resetEncoders() {
    sim.resetDistance();
  }

  @Override
  public void close() {
    // Stop simulating the mechanism
    SimulationContext.getDefault().removeMechanism(sim);
  }
}
