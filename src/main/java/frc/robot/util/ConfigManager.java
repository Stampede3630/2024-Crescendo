package frc.robot.util;

import org.reflections.Reflections;
import org.reflections.scanners.Scanners;
import org.reflections.util.ConfigurationBuilder;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

public class ConfigManager {
    private String title;

    public void configure(Object root) {
        Reflections reflections = new Reflections(
                new ConfigurationBuilder()
                        .forPackage(".frc.robot")
                        .addScanners(
                                Scanners.MethodsAnnotated,
                                Scanners.FieldsAnnotated,
                                Scanners.SubTypes
                        ));
        Set<Class<? extends Configable>> classes = reflections.getSubTypesOf(Configable.class);

        Class<?> rootClass = root.getClass();
        Set<Field> configableFields = Arrays.stream(rootClass.getDeclaredFields()).filter(f -> f.getType().isInstance(Configable.class)).collect(Collectors.toUnmodifiableSet());
        Set<Configable> configables = new HashSet<>();
        for (Field configable : configableFields) {
            try {
                configables.add((Configable) configable.get(root));
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
        for (Configable c : configables) {
            Set<Field> fields = Arrays.stream(c.getDeclaredFields()).filter(f -> f.getAnnotation(Config.class) != null).collect(Collectors.toUnmodifiableSet());
            Set<Method> methods = Arrays.stream(c.getDeclaredMethods()).filter(m -> m.getAnnotation(Config.class) != null).collect(Collectors.toUnmodifiableSet());
            for (Field f : fields) {
                String name = f.getAnnotation(Config.class).name();
                c.
            }
//            System.out.println(c);
//            System.out.println(fields);
//            System.out.println(methods);
//            System.out.println("-------------------------");
        }
    }
}
