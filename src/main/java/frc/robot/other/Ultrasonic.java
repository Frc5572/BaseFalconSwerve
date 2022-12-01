package frc.robot.other;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Ultrasonic subsystem
 */
public class Ultrasonic extends SubsystemBase {

    private AnalogInput ultrasonicCal = new AnalogInput(0);

    /**
     * Gets the distance from the target
     *
     * @return distance in centimeters
     */
    public double getDistanceValue() {
        double currentDistanceCentimeters = ultrasonicCal.getAverageValue() * 0.125;
        if (currentDistanceCentimeters < 30) {
            return 0;
        } else {
            return currentDistanceCentimeters;
        }
    }
}
