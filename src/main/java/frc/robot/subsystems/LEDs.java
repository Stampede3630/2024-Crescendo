// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

public class LEDs extends SubsystemBase {

    private static final LEDs instance = new LEDs();
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_LEDBuffer; //156
    private Color solidColor = new Color(0, 0, 0);

    private LEDs(int port, int length) {
        m_led = new AddressableLED(port);
        m_led.setLength(length);
        m_LEDBuffer = new AddressableLEDBuffer(length);
    }

    private LEDs() {
        this(0, 10);
    }

    public static LEDs getInstance() {
        return instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public Command off() {
        return setSolidColor(() -> new Color(0, 0, 0));
    }

    private void setEntireStrip() {
        for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_LEDBuffer.setLED(0, solidColor);
        }
        m_led.setData(m_LEDBuffer);
        m_led.start();
    }

    public Command setSolidColor(Supplier<Color> _color) {
        return runOnce(() -> {
                    solidColor = _color.get();
                    setEntireStrip();
                }
        );
    }

    public Command purple() {
        return setSolidColor(() -> new Color(162, 0, 255));
    }

    @Deprecated
    public Command blink(Supplier<Color> color, int count, double pulseDuration, double waitDuration) {
        Command[] pulses = new Command[count];
        for (int i = 0; i < count; i++) {
            pulses[i] = setSolidColor(color).andThen(Commands.waitSeconds(pulseDuration)).andThen(off()).andThen(Commands.waitSeconds(waitDuration));
        }
        return new SequentialCommandGroup(pulses);
    }

    public Command blink(Color color, int count, double pulseDuration, double waitDuration) {
        return new BlinkFiniteCommand(count, waitDuration, pulseDuration, color);
    }

    public Command blinkConstant(Color color, double pulseDuration, double waitDuration) {
        return new BlinkConstantCommand(pulseDuration, waitDuration, color);
    }

    public Command breathe(int hue, double secPerBreath) {
        return new BreatheCommand(hue, secPerBreath);
    }

    public Command chase(Color[] colors, long period) {
        return new ChaseColorsCommand(colors, period);
    }

    public Command rainbow() {
        return new RainbowCommand(60);
    }

    public class BlinkFiniteCommand extends Command {
        private static final Color BLACK = new Color();
        private final double waitDuration; // seconds
        private final double pulseDuration; // seconds
        private final Color color;
        private final int endCount;
        private long lastChange;
        private boolean on = false;
        private int cycleCount = 0;

        public BlinkFiniteCommand(int n, double waitDuration, double pulseDuration, Color color) {
            endCount = n;
            this.waitDuration = waitDuration;
            this.pulseDuration = pulseDuration;
            this.color = color;
            addRequirements(LEDs.this);
        }

        @Override
        public void initialize() {
            super.initialize();
            lastChange = System.currentTimeMillis();
        }

        @Override
        public void execute() {
            long currentTime = System.currentTimeMillis();
            double delta = (currentTime - lastChange) / 1000.0;

            cycleCount += (int) (delta / (waitDuration + pulseDuration));
            delta %= waitDuration + pulseDuration; // normalize to a single cycle
            if (!on && delta > waitDuration) {
                on = true;
                lastChange = currentTime;
            } else if (on && delta > pulseDuration && delta < waitDuration) {
                on = false;
                lastChange = currentTime;
                cycleCount++;
            }

            if (on) {
                for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
                    m_LEDBuffer.setLED(i, color);
                }
            } else {
                for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
                    m_LEDBuffer.setLED(i, BLACK);
                }
            }

            m_led.setData(m_LEDBuffer);
            m_led.start();
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            off().initialize();
        }

        @Override
        public boolean isFinished() {
            return cycleCount > endCount;
        }
    }

    public class BlinkConstantCommand extends Command {
        private static final Color BLACK = new Color();
        private final double waitDuration; // seconds
        private final double pulseDuration; // seconds
        private final Color color;
        private long lastChange;
        private boolean on = false;

        public BlinkConstantCommand(double waitDuration, double pulseDuration, Color color) {
            this.waitDuration = waitDuration;
            this.pulseDuration = pulseDuration;
            this.color = color;
            addRequirements(LEDs.this);
        }

        @Override
        public void initialize() {
            super.initialize();
            lastChange = System.currentTimeMillis();
        }

        @Override
        public void execute() {
            long currentTime = System.currentTimeMillis();
            double delta = (currentTime - lastChange) / 1000.0;

            delta %= waitDuration + pulseDuration; // normalize to a single cycle
            if (!on && delta > waitDuration) {
                on = true;
                lastChange = currentTime;
            } else if (on && delta > pulseDuration && delta < waitDuration) {
                on = false;
                lastChange = currentTime;
            }

            if (on) {
                for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
                    m_LEDBuffer.setLED(i, color);
                }
            } else {
                for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
                    m_LEDBuffer.setLED(i, BLACK);
                }
            }

            m_led.setData(m_LEDBuffer);
            m_led.start();
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            off().initialize();
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public class BreatheCommand extends Command {
        private final int hue;
        private final double secPerBreath;
        private long prevTime;
        private int value = 255;
        private boolean inhaling = false;

        public BreatheCommand(int hue, double secPerBreath) {
            this.hue = hue;
            this.secPerBreath = secPerBreath;
            addRequirements(LEDs.this);
        }

        @Override
        public void initialize() {
            super.initialize();
            prevTime = System.currentTimeMillis();
        }

        @Override
        public void execute() {
            long currentTime = System.currentTimeMillis();
            double delta = (currentTime - prevTime) / 1000.0;
            prevTime = currentTime;
            delta %= secPerBreath;

            for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
                m_LEDBuffer.setHSV(i, hue, 255, value);
            }

            int theChange = (int) (510 * delta / secPerBreath);
            if (inhaling && value + theChange > 255) { // trying to inhale too much
                value = 255 - (value + theChange - 255);
                inhaling = false; // start the exhalation next cycle
            } else if (!inhaling && value - theChange < 0) { // exhaling too much
                value = theChange - value;
                inhaling = true;
            } else { // normal stuff
                if (inhaling) { // normal inhale
                    value += theChange;
                } else { // normal exhale
                    value -= theChange;
                }
            }
            m_led.setData(m_LEDBuffer);
            m_led.start();
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            off().initialize();
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public class ChaseColorsCommand extends Command {
        private final Color[] chaseColors;
        private final int[] chaseColorDelim;
        private final long period;
        private long prevTime;

        public ChaseColorsCommand(Color[] colors, long period) {
            chaseColors = colors;
            this.period = period;
            chaseColorDelim = new int[colors.length];
            for (int i = 0; i < chaseColorDelim.length; i++) {
                chaseColorDelim[i] = i * m_LEDBuffer.getLength() / 4;
            }
            addRequirements(LEDs.this);
        }

        @Override
        public void execute() {
            long currentTime = System.currentTimeMillis();
            long delta = currentTime - prevTime;
            prevTime = currentTime;

            for (int i = 0; i < chaseColorDelim.length; i++) {
                chaseColorDelim[i] += (int) (delta / period);
                chaseColorDelim[i] %= m_LEDBuffer.getLength();
            }

            for (int i = 0; i < chaseColors.length; i++) {
                setRegionWrap(chaseColorDelim[i], chaseColorDelim[(i + 1) % chaseColorDelim.length], chaseColors[i]);
            }
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            off().initialize();
        }

        private void setRegionWrap(int a, int b, Color color) {
            // make sure we are "in bounds"
            a = a % m_LEDBuffer.getLength();
            b = b % m_LEDBuffer.getLength();
            if (a < 0) a += m_LEDBuffer.getLength();
            if (b < 0) b += m_LEDBuffer.getLength();

            if (b > a && a > 0 && b < m_LEDBuffer.getLength()) { // "normal" instance
                for (int i = a; i < b; i++) {
                    m_LEDBuffer.setLED(i, color);
                }
            } else { // "wrap around"
                for (int i = b; i < m_LEDBuffer.getLength(); i++) {
                    m_LEDBuffer.setLED(i, color);
                }
                for (int i = 0; i < a; i++) {
                    m_LEDBuffer.setLED(i, color);
                }
            }
            m_led.setData(m_LEDBuffer);
            m_led.start();
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public class RainbowCommand extends Command {
        private final int hueChangePerSec;
        private long prevTime;
        private int rainbowFirstPixelHue = 0;

        public RainbowCommand(int hueChangePerSec) {
            this.hueChangePerSec = hueChangePerSec;
            addRequirements(LEDs.this);
        }

        @Override
        public void execute() {
            long currentTime = System.currentTimeMillis();
            long delta = currentTime - prevTime;
            prevTime = currentTime;

            // For every pixel
            for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
                int hue = (rainbowFirstPixelHue + (i * 180 / m_LEDBuffer.getLength())) % 180;
                // Set the value
                m_LEDBuffer.setHSV(i, hue, 255, 30);
            }
            // Increase by to make the rainbow "move"
            rainbowFirstPixelHue += (int) (hueChangePerSec * (delta / 1000.0));
            // Check bounds
            rainbowFirstPixelHue %= 180;

            m_led.setData(m_LEDBuffer);
            m_led.start();
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            off().initialize();
        }

        @Override
        public boolean isFinished() {
            return false;
        }

    }
}