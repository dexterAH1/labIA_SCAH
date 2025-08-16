
from controller import Robot
import time
import os

# --- PARÁMETROS DEL CONTROLADOR PID ---
# Se han cambiado los nombres de las variables
PROPORTIONAL_GAIN = 35.0  # Ganancia Proporcional
INTEGRAL_GAIN = 0.5       # Ganancia Integral
DERIVATIVE_GAIN = 8.0     # Ganancia Derivativa

# --- INICIALIZACIÓN DE COMPONENTES ---
# Variables y objetos del robot
robot_instance = Robot()
time_step_ms = int(robot_instance.getBasicTimeStep())
time_step_sec = time_step_ms / 1000.0

# Dispositivos y sensores
cart_actuator = robot_instance.getDevice("cart motor")
pendulum_sensor = robot_instance.getDevice("pole position sensor")
pendulum_sensor.enable(time_step_ms)

# Archivo para guardar el mejor tiempo
record_file_path = "best_time.txt"

# --- FUNCIONES DE GESTIÓN DE TIEMPO ---
def load_current_record():
    """Carga el mejor tiempo guardado o devuelve 0.0 si no existe."""
    if os.path.exists(record_file_path):
        try:
            with open(record_file_path, "r") as f:
                return float(f.read().strip())
        except (ValueError, IOError):
            return 0.0
    return 0.0

def save_new_record(time_value):
    """Guarda un nuevo récord de tiempo en el archivo."""
    with open(record_file_path, "w") as f:
        f.write(f"{time_value:.2f}")

# Inicialización de las variables de control PID
integral_sum = 0.0
last_error = 0.0

# Carga el mejor tiempo al inicio de la simulación
current_best_time = load_current_record()
print(f"Mejor tiempo registrado: {current_best_time:.2f} s")

# --- BUCLE PRINCIPAL DE SIMULACIÓN ---
start_timestamp = time.time()
while robot_instance.step(time_step_ms) != -1:
    # Obtener el tiempo transcurrido desde el inicio
    elapsed_time_sec = time.time() - start_timestamp
    print(f"Tiempo transcurrido: {elapsed_time_sec:.2f} s", end="\r")

    # --- LÓGICA DE CONTROL PID ---
    
    # 1. Leer la medición del sensor
    pendulum_angle_rad = pendulum_sensor.getValue()
    
    # 2. Calcular el error actual
    current_error = pendulum_angle_rad

    # 3. Calcular los términos del PID
    proportional_term = PROPORTIONAL_GAIN * current_error
    integral_sum += current_error * time_step_sec
    derivative_term = (current_error - last_error) / time_step_sec
    last_error = current_error

    # 4. Sumar los términos para obtener la fuerza total
    force_output = proportional_term + (INTEGRAL_GAIN * integral_sum) + (DERIVATIVE_GAIN * derivative_term)

    # 5. Aplicar la fuerza al motor del carro
    cart_actuator.setForce(force_output)

    # --- VERIFICACIÓN DE RÉCORD ---
    if elapsed_time_sec > current_best_time:
        print(f"\n¡Nuevo récord! {elapsed_time_sec:.2f} s (El anterior fue {current_best_time:.2f} s)")
        save_new_record(elapsed_time_sec)
        current_best_time = elapsed_time_sec