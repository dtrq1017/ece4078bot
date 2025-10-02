import socket
import struct
import io
import threading
import time
from time import monotonic
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# Network Configuration
HOST = '0.0.0.0'
WHEEL_PORT = 8000
CAMERA_PORT = 8001
PID_CONFIG_PORT = 8002

# Pins
RIGHT_MOTOR_ENA = 18
RIGHT_MOTOR_IN1 = 17
RIGHT_MOTOR_IN2 = 27
LEFT_MOTOR_ENB = 25
LEFT_MOTOR_IN3 = 23
LEFT_MOTOR_IN4 = 24
LEFT_ENCODER = 26
RIGHT_ENCODER = 16

# PID Constants (default values, will be overridden by client)
use_PID = 0
KP, KI, KD = 0, 0, 0
MAX_CORRECTION = 30  # Maximum PWM correction value

# Global variables
running = True
left_pwm, right_pwm = 0, 0
left_count, right_count = 0, 0
prev_left_state, prev_right_state = None, None
use_ramping = True
RAMP_RATE = 250  # PWM units per second (adjust this value to tune ramp speed)
MIN_RAMP_THRESHOLD = 15  # Only ramp if change is greater than this
MIN_PWM_THRESHOLD = 15
PWM_ZERO_TOLERANCE = 1e-2  # Treat very small values as zero for movement classification
encoder_reset_requested = False
current_movement, prev_movement = 'stop', 'stop'
pid_integral, pid_last_error = 0, 0

state_lock = threading.Lock()

def setup_gpio():
    global prev_left_state, prev_right_state
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Motor
    GPIO.setup(RIGHT_MOTOR_ENA, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_ENB, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN3, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN4, GPIO.OUT)
    
    # This prevents slight motor jerk when connection is established
    GPIO.output(RIGHT_MOTOR_ENA, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_ENB, GPIO.LOW)
    
    
    # Encoder setup and interrupt (both activated and deactivated)
    GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Seed the encoder history so that the very first transition is counted.
    # Without this, the callbacks would ignore the first edge (``prev_*_state`` is
    # ``None`` on boot) and the two wheels would accumulate a one-tick mismatch
    # that immediately triggers PID corrections during the first movement.
    with state_lock:
        prev_left_state = GPIO.input(LEFT_ENCODER)
        prev_right_state = GPIO.input(RIGHT_ENCODER)

    GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=right_encoder_callback)
    
    # Initialize PWM (frequency: 100Hz)
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENB, 100)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 100)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)

def left_encoder_callback(channel):
    global left_count, prev_left_state
    current_state = GPIO.input(LEFT_ENCODER)
    
    with state_lock:
        # Check for actual state change. Without this, false positive happens due to electrical noise
        # After testing, debouncing not needed
        if (prev_left_state is not None and current_state != prev_left_state):
            left_count += 1
            prev_left_state = current_state

        elif prev_left_state is None:
            # First reading
            prev_left_state = current_state

def right_encoder_callback(channel):
    global right_count, prev_right_state, prev_right_time
    current_state = GPIO.input(RIGHT_ENCODER)
    
    with state_lock:
        if (prev_right_state is not None and current_state != prev_right_state):
            right_count += 1
            prev_right_state = current_state

        elif prev_right_state is None:
            prev_right_state = current_state
    
def reset_encoder():
    global left_count, right_count, encoder_reset_requested, prev_left_state, prev_right_state
    with state_lock:
        left_count, right_count = 0, 0
        # Re-sample the hardware state so that the next edge is captured.
        prev_left_state = GPIO.input(LEFT_ENCODER)
        prev_right_state = GPIO.input(RIGHT_ENCODER)
        encoder_reset_requested = True
def reset_pid_state():
    global pid_integral, pid_last_error
    with state_lock:
        pid_integral, pid_last_error = 0, 0

def emergency_stop(reason=None):
    global left_pwm, right_pwm, use_PID
    with state_lock:
        left_pwm, right_pwm = 0, 0
        pid_enabled = bool(use_PID)
    set_motors(0, 0)
    if pid_enabled:
        reset_encoder()
        reset_pid_state()
    if reason:
        print(f"Emergency stop: {reason}")

def set_motors(left, right):
    global prev_movement, current_movement
    
    with state_lock:
        previous_movement = prev_movement
        current_state = current_movement

    # Pre-Start Kick (Motor Priming), to reduce initial jerk and slight orientation change
    if previous_movement == 'stop' and current_state in ['forward', 'backward']:
        if current_state  == 'forward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        elif current_state == 'backward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
        right_motor_pwm.ChangeDutyCycle(100)
        time.sleep(0.05)

    if right > 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
        right_motor_pwm.ChangeDutyCycle(min(right, 100))
    elif right < 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(min(abs(right), 100))
    else:
        # when pwm = 0, implement Active Braking mode, better than putting duty cycle to 0 which may cause uneven stopping
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(100)
    
    if left > 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        left_motor_pwm.ChangeDutyCycle(min(left, 100))
    elif left < 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(min(abs(left), 100))
    else:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
    
    
def apply_min_threshold(pwm_value, min_threshold):
    if pwm_value == 0:
        return 0  # Zero means stop
    elif abs(pwm_value) < min_threshold:
        # Boost small values to minimum threshold, preserving direction
        return min_threshold if pwm_value > 0 else -min_threshold
    else:
        return pwm_value

def classify_movement(left_value, right_value):
    """Classify robot movement based on left/right PWM values.

    The previous implementation only accepted perfectly matched PWM magnitudes
    for turns and rejected common skid-steer manoeuvres. This version tolerates
    unequal magnitudes for gentle curves and treats pivots (one wheel stopped)
    as turns so that legitimate commands are not downgraded to an error state.
    """

    left_is_zero = abs(left_value) <= PWM_ZERO_TOLERANCE
    right_is_zero = abs(right_value) <= PWM_ZERO_TOLERANCE

    if left_is_zero and right_is_zero:
        return 'stop'

    left_sign = 0 if left_is_zero else (1 if left_value > 0 else -1)
    right_sign = 0 if right_is_zero else (1 if right_value > 0 else -1)

    if left_sign == right_sign:
        if left_sign > 0:
            return 'forward'
        if left_sign < 0:
            return 'backward'

    if left_sign <= 0 and right_sign >= 0:
        return 'turn_left'

    if left_sign >= 0 and right_sign <= 0:
        return 'turn_right'

    return 'error'

def pid_control():
    # Only applies for forward/backward, not turning or error states
    global left_pwm, right_pwm, left_count, right_count, use_PID, KP, KI, KD, prev_movement, current_movement, pid_integral, pid_last_error, encoder_reset_requested

    reset_pid_state()
    last_time = monotonic()
    prev_left_sample = 0
    prev_right_sample = 0
    
    # Ramping variables & params
    ramp_left_pwm = 0
    ramp_right_pwm = 0
    previous_left_target = 0
    previous_right_target = 0
    
    while running:
        current_time = monotonic()
        dt = current_time - last_time
        last_time = current_time
        reset_samples = False
        with state_lock:
            previous_movement = current_movement
            current_state = classify_movement(left_pwm, right_pwm)
            prev_movement = previous_movement
            current_movement = current_state

            local_left_pwm = left_pwm
            local_right_pwm = right_pwm
            local_left_count = left_count
            local_right_count = right_count
            local_use_pid = use_PID
            local_kp, local_ki, local_kd = KP, KI, KD
            local_pid_integral = pid_integral
            local_pid_last_error = pid_last_error
            reset_samples = encoder_reset_requested
            if encoder_reset_requested:
                encoder_reset_requested = False

        if reset_samples:
            prev_left_sample = local_left_count
            prev_right_sample = local_right_count
        if not local_use_pid:
            target_left_pwm = local_left_pwm
            target_right_pwm = local_right_pwm

        else:
            delta_left = local_left_count - prev_left_sample
            delta_right = local_right_count - prev_right_sample
            if current_state in ('forward', 'backward'):
                error = local_left_count - local_right_count
                proportional = local_kp * error
                updated_integral = local_pid_integral + local_ki * error * dt
                updated_integral = max(-MAX_CORRECTION, min(updated_integral, MAX_CORRECTION))  # Anti-windup
                derivative = local_kd * (error - local_pid_last_error) / dt if dt > 0 else 0
                correction = proportional + updated_integral + derivative
                correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
                            

                target_left_pwm = local_left_pwm - correction
                target_right_pwm = local_right_pwm + correction

                with state_lock:
                    pid_integral = updated_integral
                    pid_last_error = error
            elif current_state in ('turn_left', 'turn_right'):
                abs_left_pwm = abs(local_left_pwm)
                abs_right_pwm = abs(local_right_pwm)
                abs_delta_left = abs(delta_left)
                abs_delta_right = abs(delta_right)

                if abs_delta_left + abs_delta_right == 0:
                    error = 0
                elif abs_left_pwm > PWM_ZERO_TOLERANCE and abs_right_pwm > PWM_ZERO_TOLERANCE:
                    expected_right = (abs_right_pwm / abs_left_pwm) * abs_delta_left
                    error = expected_right - abs_delta_right
                else:
                    error = abs_delta_left - abs_delta_right

                proportional = local_kp * error
                updated_integral = local_pid_integral + local_ki * error * dt
                updated_integral = max(-MAX_CORRECTION, min(updated_integral, MAX_CORRECTION))
                derivative = local_kd * (error - local_pid_last_error) / dt if dt > 0 else 0
                correction = proportional + updated_integral + derivative
                correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))

                left_direction = 1 if local_left_pwm >= 0 else -1
                right_direction = 1 if local_right_pwm >= 0 else -1

                adjusted_left = max(abs_left_pwm - correction, 0)
                adjusted_right = max(abs_right_pwm + correction, 0)

                target_left_pwm = left_direction * adjusted_left
                target_right_pwm = right_direction * adjusted_right

                with state_lock:
                    pid_integral = updated_integral
                    pid_last_error = error              
            else:
                if current_movement == 'stop':
                    # Reset when stopped to clear any accumulated correction
                    reset_pid_state()
                    reset_encoder()
                target_left_pwm = local_left_pwm
                target_right_pwm = local_right_pwm

        if current_movement == 'error':
            # Ignore invalid PWM commands and actively stop the robot
            target_left_pwm = 0
            target_right_pwm = 0
            if local_use_pid:
                reset_pid_state()
                reset_encoder()
        
        if current_state == 'error':
            # Apply zero output immediately when ignoring invalid commands
            ramp_left_pwm = target_left_pwm
            ramp_right_pwm = target_right_pwm
            previous_left_target = target_left_pwm
            previous_right_target = target_right_pwm
        elif use_ramping and local_use_pid:
            # PWM Ramping Logic
            max_change_per_cycle = RAMP_RATE * dt
            
            # Calculate differences for both motors
            left_diff = target_left_pwm - ramp_left_pwm
            right_diff = target_right_pwm - ramp_right_pwm
            
            # Determine if either motor needs ramping
            left_needs_ramp = abs(left_diff) > MIN_RAMP_THRESHOLD
            right_needs_ramp = abs(right_diff) > MIN_RAMP_THRESHOLD
            
            # Check for direction change conditions (but not stops)
            left_direction_change = (target_left_pwm * previous_left_target < 0) and target_left_pwm != 0 and previous_left_target != 0
            right_direction_change = (target_right_pwm * previous_right_target < 0) and target_right_pwm != 0 and previous_right_target != 0
            
            # Apply immediate changes for direction changes only (for safety)
            if left_direction_change:
                ramp_left_pwm = target_left_pwm
            if right_direction_change:
                ramp_right_pwm = target_right_pwm
            
            # Synchronized ramping - both motors ramp together or not at all
            if not left_direction_change and not right_direction_change:
                if left_needs_ramp or right_needs_ramp:
                    
                    # Left motor ramping (including ramp-down to zero)
                    if abs(left_diff) <= max_change_per_cycle:
                        ramp_left_pwm = target_left_pwm  # Close enough, set to target
                    else:
                        # Ramp towards target (up or down)
                        if left_diff > 0:
                            ramp_left_pwm += max_change_per_cycle
                        else:
                            ramp_left_pwm -= max_change_per_cycle
                    
                    # Right motor ramping (including ramp-down to zero)
                    if abs(right_diff) <= max_change_per_cycle:
                        ramp_right_pwm = target_right_pwm  # Close enough, set to target
                    else:
                        # Ramp towards target (up or down)
                        if right_diff > 0:
                            ramp_right_pwm += max_change_per_cycle
                        else:
                            ramp_right_pwm -= max_change_per_cycle
                else:
                    # Neither motor needs ramping - apply targets directly
                    ramp_left_pwm = target_left_pwm
                    ramp_right_pwm = target_right_pwm
            
            # Store previous targets for next iteration
            previous_left_target = target_left_pwm
            previous_right_target = target_right_pwm
        
        else:
            # Ramping disabled - apply target values directly
            ramp_left_pwm = target_left_pwm
            ramp_right_pwm = target_right_pwm
            
        final_left_pwm = apply_min_threshold(ramp_left_pwm, MIN_PWM_THRESHOLD)
        final_right_pwm = apply_min_threshold(ramp_right_pwm, MIN_PWM_THRESHOLD)
        set_motors(final_left_pwm, final_right_pwm)
        
        if ramp_left_pwm != 0: # print for debugging purpose
            with state_lock:
                debug_left = left_count
                debug_right = right_count
            print(f"(Left PWM, Right PWM)=({ramp_left_pwm:.2f},{ramp_right_pwm:.2f}), (Left Enc, Right Enc)=({debug_left}, {debug_right})")

        prev_left_sample = local_left_count
        prev_right_sample = local_right_count
        time.sleep(0.01)


def camera_stream_server():
    # Initialize camera
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(lores={"size": (640,480)})
    picam2.configure(camera_config)
    picam2.start()
    
    # Create socket for streaming
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, CAMERA_PORT))
    server_socket.listen(1)
    server_socket.settimeout(1.0)
    print(f"Camera stream server started on port {CAMERA_PORT}")
    client_socket = None
    while running:
        try:
            client_socket, _ = server_socket.accept()
            client_socket.settimeout(1.0)
            print(f"Camera stream client connected")
            
            while running:
                # Capture frame and convert to bytes
                stream = io.BytesIO()
                picam2.capture_file(stream, format='jpeg')
                stream.seek(0)
                jpeg_data = stream.getvalue()
                jpeg_size = len(jpeg_data)
                
                try:
                    client_socket.sendall(struct.pack("!I", jpeg_size))
                    client_socket.sendall(jpeg_data)
                except socket.timeout:
                    continue
                except Exception:
                    print("Camera stream client disconnected")
                    break
                
                # Small delay to avoid hogging CPU
                time.sleep(0.01)

                if not running:
                    break

        except socket.timeout:
            continue
                
        except Exception as e:
            print(f"Camera stream server error: {str(e)}")
        
        if 'client_socket' in locals() and client_socket:
            client_socket.close()
            client_socket = None

    server_socket.close()
    picam2.stop()

def recv_exact(sock, size):
    """Receive exactly ``size`` bytes from ``sock`` or return None if disconnected."""
    chunks = bytearray()
    while len(chunks) < size:
        try:
            chunk = sock.recv(size - len(chunks))
        except Exception:
            return None
        if not chunk:
            return None
        chunks.extend(chunk)
    return bytes(chunks)

def pid_config_server():
    global use_PID, KP, KI, KD
    
    # Create socket for receiving PID configuration
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PID_CONFIG_PORT))
    server_socket.listen(1)
    print(f"PID config server started on port {PID_CONFIG_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"PID config client connected")
            
            try:
                # Receive PID constants (4 floats)
                data = recv_exact(client_socket, 16)
                if data is not None:
                    new_use_pid, new_kp, new_ki, new_kd = struct.unpack("!ffff", data)
                    with state_lock:
                        use_PID = new_use_pid
                        KP, KI, KD = new_kp, new_ki, new_kd
                        pid_enabled = bool(use_PID)
                    if pid_enabled:
                        print(f"Updated PID constants: KP={KP}, KI={KI}, KD={KD}")
                    else:
                        print("The robot is not using PID.")
                    
                    # Send acknowledgment (1 for success)
                    response = struct.pack("!i", 1)
                else:
                    print("PID config client disconnected")
                    # Send failure response
                    response = struct.pack("!i", 0)
                
                client_socket.sendall(response)
                    
            except Exception as e:
                print(f"PID config socket error: {str(e)}")
                try:
                    response = struct.pack("!i", 0)
                    client_socket.sendall(response)
                except:
                    pass
                    
            client_socket.close()
                    
        except Exception as e:
            print(f"PID config server error: {str(e)}")
    
    server_socket.close()
    

def wheel_server():
    global left_pwm, right_pwm, running, left_count, right_count
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, WHEEL_PORT))
    server_socket.listen(1)
    print(f"Wheel server started on port {WHEEL_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Wheel client connected")
            
            while running:
                try:
                    # Receive speed (4 bytes for each value)
                    data = recv_exact(client_socket, 8)
                    if data is None:
                        print("Wheel client sending speed error")
                        emergency_stop("Invalid wheel speed payload received")
                        break
                    
                    # Unpack speed values and convert to PWM
                    left_speed, right_speed = struct.unpack("!ff", data)
                    # print(f"Received wheel: left_speed={left_speed:.4f}, right_speed={right_speed:.4f}")
                    with state_lock:
                        left_pwm = left_speed*100
                        right_pwm = right_speed*100
                        current_left_count = left_count
                        current_right_count = right_count
                    
                    # Send encoder counts back
                    response = struct.pack("!ii", current_left_count, current_right_count)
                    client_socket.sendall(response)
                    
                except Exception as e:
                    print(f"Wheel client disconnected")
                    emergency_stop("Wheel client connection lost")
                    break
                    
        except Exception as e:
            print(f"Wheel server error: {str(e)}")
            emergency_stop("Wheel server encountered an error")
        
        if 'client_socket' in locals() and client_socket:
            client_socket.close()
    
    server_socket.close()


def main():
    try:
        setup_gpio()
        
        # Start PID control thread
        pid_thread = threading.Thread(target=pid_control)
        pid_thread.daemon = True
        pid_thread.start()
        
        # Start camera streaming thread
        camera_thread = threading.Thread(target=camera_stream_server)
        camera_thread.daemon = True
        camera_thread.start()
        
        # Start PID configuration server thread
        pid_config_thread = threading.Thread(target=pid_config_server)
        pid_config_thread.daemon = True
        pid_config_thread.start()
        
        # Start wheel server (main thread)
        wheel_server()
        
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        global running
        running = False
        GPIO.cleanup()
        print("Cleanup complete")


if __name__ == "__main__":
    main()