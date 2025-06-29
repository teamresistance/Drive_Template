package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

/** Utility class for working with Phoenix 6 CTRE devices and status codes. */
public class PhoenixUtil {

  private PhoenixUtil() {
    throw new IllegalStateException("Utility class");
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }
}
