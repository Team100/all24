package org.team100.log_annotation;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class Registrar {
    final Map<String, DoubleSupplier> suppliers = new HashMap<>();

    public void register(Object obj) {
        Class<?> clazz = obj.getClass();

        // first look for fields
        for (Field field : clazz.getDeclaredFields()) {
            if (field.isAnnotationPresent(Log.class)) {
                field.setAccessible(true); // allow private
                Class<?> fieldType = field.getType();
                String fieldName = field.getName();
                if (fieldType.equals(double.class)) {
                    suppliers.put(fieldName, () -> {
                        try {
                            return field.getDouble(obj);
                        } catch (IllegalArgumentException | IllegalAccessException e) {
                            e.printStackTrace();
                            return 0;
                        }
                    });
                }
            }
        }

        // then look for methods
        for (Method method : clazz.getDeclaredMethods()) {
            if (method.isAnnotationPresent(Log.class)) {
                method.setAccessible(true); // allow private
                Class<?> methodType = method.getReturnType();
                String methodName = method.getName();
                if (methodType.equals(double.class)) {
                    suppliers.put(methodName, () -> {
                        try {
                            return (double) method.invoke(obj);
                        } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
                            e.printStackTrace();
                            return 0;
                        }
                    });
                }
            }
        }

    }
}
