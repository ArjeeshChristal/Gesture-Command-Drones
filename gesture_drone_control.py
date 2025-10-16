import time
import cv2
import mediapipe as mp
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# ------------------- DRONE CONNECTION -------------------

connection_string = '127.0.0.1:14550' #this is where to specify the SITL default : 127.0.0.1:14550
baud_rate = 57600

print(f"🔗 Connecting to vehicle on {connection_string} at {baud_rate} baud...")

print(f"Connecting to vehicle on: {connection_string}")
vehicle = connect(
    connection_string,
    baud=baud_rate,
    wait_ready=False,
    heartbeat_timeout=60,
    timeout=12000
)


print("✅ Connected! Waiting for readiness...")
vehicle.wait_ready(True, timeout=12000)
print("✅ Vehicle ready.")

# ------------------- DRONE FUNCTIONS -------------------
def arm_and_takeoff(target_altitude=5):
    """Arm and take off to initial altitude."""
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {target_altitude} m")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.2f}")
        if alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def land():
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")

def hover():
    send_ned_velocity(0, 0, 0)
    print("Hovering")

# --- MAVLink velocity control ---
def send_ned_velocity(vx, vy, vz, duration=1):
    """Move vehicle in body-frame velocity (m/s)."""
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    for _ in range(duration * 2):  # send for given seconds
        vehicle.send_mavlink(msg)
        time.sleep(0.5)

def move_left():
    send_ned_velocity(0, -0.5, 0)
    print("Move Left")

def move_right():
    send_ned_velocity(0, 0.5, 0)
    print("Move Right")

# --- Adjust altitude smoothly by ±delta (m) ---
def adjust_altitude(delta):
    current_alt = vehicle.location.global_relative_frame.alt
    target_alt = current_alt + delta
    target_alt = max(1, min(target_alt, 20))  # clamp between 1m and 20m

    print(f"Changing altitude from {current_alt:.1f} → {target_alt:.1f}")
    point = LocationGlobalRelative(vehicle.location.global_frame.lat,
                                   vehicle.location.global_frame.lon,
                                   target_alt)
    vehicle.simple_goto(point)

# ------------------- HAND DETECTION SETUP -------------------
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)

gesture = "HOVER"
last_gesture = None
print("Starting webcam... Press 'q' to quit.")

# ------------------- MAIN LOOP -------------------
try:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            continue

        height, width, _ = image.shape
        image = cv2.flip(image, 1)
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb_image)

        # Hover zone box
        dz_x1, dz_x2 = int(width * 0.35), int(width * 0.65)
        dz_y1, dz_y2 = int(height * 0.35), int(height * 0.65)
        cv2.rectangle(image, (dz_x1, dz_y1), (dz_x2, dz_y2), (0, 255, 0), 2)

        new_gesture = "HOVER"

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            index_tip = hand_landmarks.landmark[8]
            index_mcp = hand_landmarks.landmark[5]
            palm = hand_landmarks.landmark[9]
            cx, cy = int(palm.x * width), int(palm.y * height)
            cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1)

            # Fist → Land
            if index_tip.y > index_mcp.y:
                new_gesture = "LAND"
            else:
                if cx < dz_x1:
                    new_gesture = "MOVE LEFT"
                elif cx > dz_x2:
                    new_gesture = "MOVE RIGHT"
                elif cy < dz_y1:
                    new_gesture = "MOVE UP"
                elif cy > dz_y2:
                    new_gesture = "MOVE DOWN"
                else:
                    new_gesture = "HOVER"

        else:
            new_gesture = "HOVER"

        # Only act when gesture changes
        if new_gesture != last_gesture:
            gesture = new_gesture
            print(f"Gesture changed to: {gesture}")

            if gesture == "LAND":
                land()
            elif gesture == "MOVE LEFT":
                move_left()
            elif gesture == "MOVE RIGHT":
                move_right()
            elif gesture == "MOVE UP":
                if not vehicle.armed:
                    arm_and_takeoff(5)  # initial takeoff
                else:
                    adjust_altitude(+0.5)  # 0.5 m step
            elif gesture == "MOVE DOWN":
                adjust_altitude(-0.5)  # 0.5 m step
            elif gesture == "HOVER":
                hover()

            last_gesture = gesture

        # Display gesture text
        cv2.putText(image, f"Gesture: {gesture}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow('Gesture Control', image)

        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"Error: {e}")

finally:
    print("Landing and closing connection...")
    land()
    cap.release()
    cv2.destroyAllWindows()
    vehicle.close()
