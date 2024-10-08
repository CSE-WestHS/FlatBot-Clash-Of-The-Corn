// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Indexer.Indexer;
public class ShootNote extends Command {
  Shooter shooter;
  Indexer indexer;
  int targetSpeed;
  double startTIme;
  /** Creates a new ShootNote. */
  public ShootNote(Shooter Shooter, Indexer Indexer, int TargetSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Shooter,Indexer);
    this.shooter = Shooter;
    this.indexer = Indexer;
    this.targetSpeed = TargetSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.runVelocity(targetSpeed);
    indexer.runVelocity(targetSpeed);
    startTIme = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs((shooter.getVelocityRPM() - targetSpeed)) < 3) {
      indexer.runVelocity(2000);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!indexer.getBeamState() || (Timer.getFPGATimestamp()- startTIme) > 3) {
      return true;
    }
    return false;
  }
}
