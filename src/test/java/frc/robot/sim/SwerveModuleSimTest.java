package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Note: all tests run with a constants 12V battery input.
 */
class SwerveModuleSimTest {
  @BeforeAll
  static void wpilibSetup() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData(); // "enable" the robot so motor controllers will work
  }

  @Test
  void updatesPositiveVelocity() {
    var sim = new MAXSwerveModuleSim();
    sim.setDesiredState(new SwerveModuleState(5, new Rotation2d()));
    sim.update(0.02);
    assertTrue(sim.getWheelVelocity() > 0, sim.getWheelVelocity() + " should be positive");
  }

  @Test
  void updatesNegativeVelocity() {
    var sim = new MAXSwerveModuleSim();
    sim.setDesiredState(new SwerveModuleState(-5, new Rotation2d()));
    sim.update(0.02);
    assertTrue(sim.getWheelVelocity() < 0, sim.getWheelVelocity() + " should be negative");
  }

  @Test
  void convergesOnPositiveVelocity() {
    var sim = new MAXSwerveModuleSim();
    sim.setDesiredState(new SwerveModuleState(5, new Rotation2d()));
    for (int i = 0; i < 50; i++) {
      sim.update(0.02);
    }
    assertEquals(5, sim.getWheelVelocity(), 1e-3, "Wheel velocity did not converge within specified time period");
  }

  @Test
  void convergesOnNegativeVelocity() {
    var sim = new MAXSwerveModuleSim();
    sim.setDesiredState(new SwerveModuleState(-5, new Rotation2d()));
    for (int i = 0; i < 50; i++) { // 1 second of acceleration
      sim.update(0.02);
    }
    assertEquals(-5, sim.getWheelVelocity(), 1e-3, "Wheel velocity did not converge within specified time period");
  }

  @Test
  void updatesPositiveAngle() {
    var sim = new MAXSwerveModuleSim();
    sim.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90)));
    sim.update(0.02);
    double angle = sim.getModuleAngle().getDegrees();
    assertTrue(angle > 0, angle + " should be positive");
  }

  @Test
  void updatesNegativeAngle() {
    var sim = new MAXSwerveModuleSim();
    sim.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-90)));
    sim.update(0.02);
    double angle = sim.getModuleAngle().getDegrees();
    assertTrue(angle < 0, angle + " should be positive");
  }

  @Test
  void testConvergesOnPositiveAngle() {
    var sim = new MAXSwerveModuleSim();
    sim.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90)));
    for (int i = 0; i < 50; i++) {
      sim.update(0.02);
    }
    double angle = sim.getModuleAngle().getDegrees();
    assertEquals(90, angle, 1e-3, "Module angle did not converge");
  }

  @Test
  void testConvergesOnNegativeAngle() {
    var sim = new MAXSwerveModuleSim();
    sim.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-90)));
    for (int i = 0; i < 50; i++) {
      sim.update(0.02);
    }
    double angle = sim.getModuleAngle().getDegrees();
    assertEquals(-90, angle, 1e-3, "Module angle did not converge");
  }
}
