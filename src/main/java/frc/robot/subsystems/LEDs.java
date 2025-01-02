package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

    AddressableLEDBufferView view;

    public LEDs(AddressableLEDBufferView view) {
        this.view = view;
    }

    public Command runDefault(Color color) {
        LEDPattern pattern = LEDPattern.solid(color);
        return run(() -> {
            pattern.applyTo(view);
        }).ignoringDisable(true);
    }

    public Command runDefault2(Color color) {
        LEDPattern pattern = LEDPattern.solid(color).blink(Units.Seconds.of(1));
        return run(() -> {
            pattern.applyTo(view);
        }).ignoringDisable(true);
    }
}
