# A message for specifying a target position, velocity, and acceleration for a servo.

float32 position        # Specifies the target position for the servo as a percentage of it's operating range.
                        # Range: -1.0 to 1.0

float32 speed           # Specifies the maximum speed for the servo to reach the target, in percentage/second.
                        # Range: 0.0 to infinity.  Driver will clip at maximum achievable speed.
                        # Setting speed to 0.0 sets the speed limit to it's maximum value.
                        # Setting speed to N/A indicates to not update the speed.

float32 acceleration    # Specifies the maximum acceleration for the servo to reach the target, in percentage/second^2.
                        # Range: 0.0 to infinity.  Driver will clip at maximum achievable acceleration.
                        # Setting acceleration to 0.0 sets the acceleration to it's maximum value.
                        # Setting acceleration to N/A indicates to not update the acceleration.
