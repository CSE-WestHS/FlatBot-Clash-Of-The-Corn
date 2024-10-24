// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(10);
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);

  public static int DRIVE_FL = 6;
  public static int TURN_FL = 4;
  public static int DRIVE_FR = 9;
  public static int TURN_FR = 7;
  public static int DRIVE_BL = 3;
  public static int TURN_BL = 1;
  public static int DRIVE_BR = 12;
  public static int TURN_BR = 10;
  public static int INTAKE_MOTOR_1 = 0;
  public static int INTAKE_MOTOR_2 = 1;
  public static int SHOOTER_MOTOR_1 = 20;
  public static int SHOOTER_MOTOR_2 = 21;
  public static int INDEXER_MOTOR = 13;
}
