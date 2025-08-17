# Guía de Prueba: Teleoperación Braccio con Gamepad

## Estado del Sistema
✅ **Compilación**: Exitosa  
✅ **Paquete**: braccio_gamepad_teleop instalado  
✅ **Launch files**: Disponibles  
✅ **Código**: Corregido para inicialización robusta de MoveIt  

## Instrucciones de Prueba

### 1. Preparación
Antes de empezar, asegúrate de tener:
- Un gamepad conectado (Xbox, PlayStation o compatible)
- El sistema Braccio configurado y funcionando
- ROS2 Humble instalado

### 2. Verificar el Gamepad

```bash
# Verificar que el gamepad se detecte
ls /dev/input/js*

# Debería mostrar algo como: /dev/input/js0
```

```bash
# Probar el gamepad directamente
ros2 run joy joy_node
```

En otra terminal:
```bash
ros2 topic echo /joy
```

### 3. Opción 1: Prueba Individual (Recomendado para depuración)

**Terminal 1**: Iniciar MoveIt
```bash
cd /home/ivan/Escritorio/Braccio-Tinkerkit-Arduino
source install/setup.bash
ros2 launch braccio_moveit_config demo.launch.py
```

**Terminal 2**: Iniciar driver del gamepad
```bash
source /opt/ros/humble/setup.bash
ros2 run joy joy_node
```

**Terminal 3**: Iniciar teleoperación
```bash
cd /home/ivan/Escritorio/Braccio-Tinkerkit-Arduino
source install/setup.bash
ros2 run braccio_gamepad_teleop gamepad_teleop
```

### 4. Opción 2: Launch Completo (Una vez verificado que funciona)

```bash
cd /home/ivan/Escritorio/Braccio-Tinkerkit-Arduino
source install/setup.bash
ros2 launch braccio_gamepad_teleop gamepad_control.launch.py
```

**O usando el script**:
```bash
./start_gamepad_control.sh
```

### 5. Controles del Gamepad

| Control | Función |
|---------|---------|
| Stick Izquierdo X | Movimiento en eje X |
| Stick Izquierdo Y | Movimiento en eje Y |
| Stick Derecho Y | Movimiento en eje Z (arriba/abajo) |
| Botón A | Rotación positiva |
| Botón B | Rotación negativa |

### 6. Verificación

**Monitorizar logs**:
```bash
# En el terminal donde ejecutaste gamepad_teleop, deberías ver:
[gamepad_teleop]: Gamepad teleop iniciando...
[gamepad_teleop]: MoveGroup inicializado. Gamepad teleop listo
```

**Verificar tópicos**:
```bash
ros2 topic list | grep joy
ros2 topic echo /joy --once
```

### 7. Solución de Problemas

#### Problema: "MoveGroup aún no inicializado"
- **Causa**: MoveIt no está ejecutándose
- **Solución**: Verificar que `braccio_moveit_config demo.launch.py` esté corriendo

#### Problema: No hay datos en `/joy`
- **Causa**: Gamepad no detectado o joy_node no funcionando
- **Solución**: Verificar conexión del gamepad y reiniciar joy_node

#### Problema: El brazo no se mueve
- **Causa**: MoveIt puede estar en modo de simulación
- **Solución**: Verificar configuración de MoveIt y conexión del hardware

### 8. Seguridad

⚠️ **IMPORTANTE**: 
- Mantener supervisión visual constante del brazo
- Movimientos son incrementales y graduales
- Velocidad y aceleración limitadas al 20%
- En caso de emergencia: Ctrl+C en todas las terminales

### 9. Archivos Importantes

```
braccio_gamepad_teleop/
├── src/gamepad_teleop.cpp          # Código principal
├── launch/gamepad_control.launch.py # Launch completo
├── CMakeLists.txt                   # Configuración de build
├── package.xml                      # Dependencias
└── README.md                        # Documentación detallada
```

### 10. Próximos Pasos

Una vez verificado el funcionamiento básico:
1. Ajustar parámetros de velocidad si es necesario
2. Personalizar mapeo de botones según preferencias
3. Agregar control de la pinza (gripper)
4. Implementar modos de control adicionales

## Log de Desarrollo

- ✅ Estructura del paquete creada
- ✅ Dependencias de MoveIt configuradas
- ✅ Headers de MoveIt localizados y corregidos
- ✅ Inicialización robusta de MoveGroupInterface implementada
- ✅ Sistema de teleoperación básico funcionando
- ✅ Launch files configurados
- ✅ Documentación completa
