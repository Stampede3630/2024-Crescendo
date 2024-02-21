package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.EntryBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;
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
    private String tableRoot;

    public ConfigManager(String tableRoot) {
        this.tableRoot = tableRoot;
    }

    Set<FieldData> fields = new HashSet<>();
    public void configure(Object root) {
        Reflections reflections = new Reflections(
                new ConfigurationBuilder()
                        .forPackage(".frc.robot")
                        .addScanners(
                                Scanners.MethodsAnnotated,
                                Scanners.FieldsAnnotated,
                                Scanners.SubTypes
                        ));

        Class<?> rootClass = root.getClass();
        System.out.println(rootClass);

        Set<Field> configableFields = Arrays.stream(rootClass.getDeclaredFields())
//        configableFields.forEach(System.out::println);
                .filter(f -> f.getType().isInstance(Configable.class)).collect(Collectors.toUnmodifiableSet());

        System.out.println(configableFields);
        Set<Configable> configables = new HashSet<>();
        for (Field configable : configableFields) {
            try {
                configables.add((Configable) configable.get(root));
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
            System.out.println(configable);
        }
        for (Configable c : configables) {
            Set<FieldData> myFields = Arrays.stream(c.getClass().getDeclaredFields()).filter(f -> f.getAnnotation(Config.class) != null)
                    .map(f -> new FieldData(f, c, f.getAnnotation(Config.class).name()))
                    .collect(Collectors.toUnmodifiableSet());
            fields.addAll(myFields);
        }

        for (FieldData f : fields) {
            if (f.field.getType().isInstance(Double.class)) {
                try {
                    f.setTopic(NetworkTableInstance.getDefault().getTable(tableRoot).getDoubleTopic(f.name).getEntry(f.field.getDouble(f.object)));
                } catch (IllegalAccessException e) {
                    throw new RuntimeException(e);
                }
                System.out.println(f.name);
            }
        }

//        fields.forEach(f -> System.out.println(f.field.getName()));

    }

    public void update() {
        for (FieldData f : fields) {
            try {
                f.field.set(f.object, f.getTopic().get());
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }

    }

    private class FieldData {
        private Field field;
        private Object object;
        private String name;
        private DoubleEntry topic;

        public FieldData(Field field, Object object, String name) {
            this.field = field;
            this.object = object;
            this.name = name;
//            this.topic = topic;
        }

        public Field getField() {
            return field;
        }

        public Object getObject() {
            return object;
        }

        public String getName() {
            return name;
        }

        public DoubleEntry getTopic() {
            return topic;
        }

        public void setTopic(DoubleEntry topic) {
            this.topic = topic;
        }
    }
}
