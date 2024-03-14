package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ValueEventData;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

public class ConfigManager {
    private final String tableRoot;

    public ConfigManager(String tableRoot) {
        this.tableRoot = tableRoot;
    }

    private Set<FieldData> fields = new HashSet<>();

    public void configure(Object root) {

        Class<?> rootClass = root.getClass();

        Set<Field> configableFields = Arrays.stream(rootClass.getDeclaredFields())
            .filter(f -> Arrays.asList(f.getType().getInterfaces()).contains(Configable.class)).collect(Collectors.toUnmodifiableSet());

        Set<Configable> configables = new HashSet<>();
        for (Field configable : configableFields) {
            try {
                configable.setAccessible(true);
                configables.add((Configable) configable.get(root));
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
        for (Configable c : configables) {
            Set<FieldData> myFields = Arrays.stream(c.getClass().getDeclaredFields()).filter(f -> f.getAnnotation(Config.class) != null)
                .map(f -> new FieldData(f, c,
                        "".equals(f.getAnnotation(Config.class).name()) ? c.getClass().getSimpleName() + ":" + f.getName() : f.getAnnotation(Config.class).name()
                ))
                .collect(Collectors.toUnmodifiableSet());
            fields.addAll(myFields);
        }

        for (FieldData f : fields) {
            f.setEntry(NetworkTableInstance.getDefault().getTable(tableRoot).getEntry(f.name));
            f.entry.setDefaultValue(f.getValue());
            NetworkTableInstance.getDefault().addListener(f.entry,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> setValue(f, e.valueData));
        }
    }

    private void setValue(FieldData f, ValueEventData e) {
        System.out.println(e.value.getType());
//        f.field.getGenericType().
        switch (e.value.getType()) {
//            case kDouble -> f.setValue(e.value.getValue());
            // case kBoolean -> f.setValue(e);
            case kInteger -> f.setValue(((Number) e.value.getValue()).intValue());
            default -> f.setValue(e.value.getValue());
//            case k
        }
    }

    private static class FieldData {
        private final Field field;
        private final Object object;
        private final String name;
        private NetworkTableEntry entry;

        public FieldData(Field field, Object object, String name) {
            field.setAccessible(true);
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

        public NetworkTableEntry getEntry() {
            return entry;
        }

        public void setEntry(NetworkTableEntry entry) {
            this.entry = entry;
        }

        public Object getValue() {
            try {
                return field.get(object);
            } catch (IllegalAccessException e) {
                return null;
            }
        }

        public void setValue(Object value) {
            try {
                field.set(object, value);
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }

        @Override
        public String toString() {
            return "FieldData{" +
                "field=" + field +
                ", object=" + object +
                ", name='" + name + '\'' +
                ", entry=" + entry +
                '}';
        }
    }
}
