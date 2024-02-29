package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoCommands;

import java.lang.reflect.InvocationTargetException;
import java.util.Arrays;
import java.util.stream.Collectors;

public class AutoCommandFinder {
    public static void addAutos() {
        NamedCommands.registerCommands(Arrays.stream(AutoCommands.class.getMethods())
                .filter(m -> Command.class.equals(m.getReturnType()) && m.getParameterCount() == 0)
                .map(m -> {
                    try {
                       System.out.println("Added: "+m.getName()+" "+ m.invoke(null));
                        return new Pair<>(m.getName(), (Command) (m.invoke(null)));
                    } catch (IllegalAccessException | InvocationTargetException e) {
                        throw new RuntimeException(e);
                    }
                })
                .collect(Collectors.toList()));
    }
}
