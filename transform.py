from ugot import ugot
import time
import cv2
import numpy as np

#Some constants that control the robot functionalities
LEFT = 2 
RIGHT = 3
FORWARD = 0
BACKWARD = 1
BASE_HEIGHT = 5 # cm
LOWER_HEIGHT = 3 # cm
UPPER_HEIGHT = 7 # cm

got = ugot.UGOT()
got.initialize('192.168.196.130')
got.open_camera()
got.load_models(["line_recognition", "apriltag_qrcode"])
got.set_track_recognition_line(0)  # 0: mono line, 1: double line

performing_action = False
current_action = None
last_change_time = 0  # Timestamp of the last track_type change
previous_track_type = 1  # Initialize with a default value
verification_failed = False  # Flag to indicate a failed verification
cooldown = False  # Flag to indicate if cooldown is active
cooldown_duration = 5  # Time in seconds to wait after a failed verification
cooldown_start_time = 0  # Timestamp when the cooldown started

def line_follow(offset):
    if offset > 30:
        got.transform_turn_speed(LEFT, 10)  # 2: Left turn
    elif offset < -31:
        got.transform_turn_speed(RIGHT, 10)  # 3: Right turn
    else:
        got.transform_move_speed(FORWARD, 5)  # 0: Forward

def verify_track_type_change():
    """
    Verify the track_type change by chainging the height
    """    
    got.transform_set_chassis_height(LOWER_HEIGHT)
    lower_track_type = got.get_single_track_total_info()[1]

    got.transform_set_chassis_height(UPPER_HEIGHT)
    upper_track_type = got.get_single_track_total_info()[1]

    got.transform_set_chassis_height(BASE_HEIGHT)
    # Check if both recorded track_types match the original track_type change
    print(lower_track_type, upper_track_type)
    if lower_track_type != 1 and upper_track_type != 1:
        return True
    return False


action_queue = ["forward_short", "forward_long", "forward_short", "turn_right_small"]

action_dict = {
  "forward_short"     : lambda: got.transform_move_speed_times(FORWARD, 10, 15, 1),
  "forward_long"      : lambda: got.transform_move_speed_times(FORWARD, 30, 70, 1),
  "backward_short"    : lambda: got.transform_move_speed_times(BACKWARD, 10, 15, 1),
  "backward_long"     : lambda: got.transform_move_speed_times(BACKWARD, 30, 70, 1),
  "turn_right_small"  : lambda: got.transform_turn_speed_times(RIGHT, 10, 3, 0),
  "turn_right_big"    : lambda: got.transform_turn_speed_times(RIGHT, 20, 3, 0),
  "turn_left_small"   : lambda: got.transform_turn_speed_times(LEFT, 10, 3, 0),
  "turn_right_big"    : lambda: got.transform_turn_speed_times(LEFT, 20, 3, 0),
}

qr_code_list = []


while True:
    qr_code_info = got.get_qrcode_total_info()
    if len(qr_code_info) != 0 and qr_code_info not in qr_code_list:
      qr_code_list.append(qr_code_info)


    got.transform_set_chassis_height(BASE_HEIGHT)
    track_info = got.get_single_track_total_info()
    offset = track_info[0]  # -320 -> 320 (positive = right)
    track_type = track_info[1]  # 1:|, 2:Y, 3:X, 0:No line
    x_center, y_center = track_info[2:]  # Intersection coordinates
    print(action_queue, current_action, verification_failed)
    current_time = time.time()

    # Handle cooldown logic
    if cooldown:
        if current_time - cooldown_start_time >= cooldown_duration:
            cooldown = False  # Deactivate cooldown after the duration
        else:
            line_follow(offset)  # Continue following the line during cooldown
            continue  # Skip the rest of the loop during cooldown

    # Reset the failed verification flag after cooldown
    if verification_failed and current_time - cooldown_start_time >= cooldown_duration:
        verification_failed = False

    # Normal line following when track_type is 1
    if track_type == 1:
        line_follow(offset)

    elif track_type != 1:
        # Track_type changed, start verification
        if verify_track_type_change():
            print("Verification successful")
            # Perform the next action if verification is successful
            if not performing_action and action_queue:
                current_action = action_queue.pop(0)
                performing_action = True
        else:
            # Verification failed, activate cooldown
            print("Verification Failed")
            verification_failed = True
            cooldown = True
            cooldown_start_time = current_time  # Record when cooldown starts

    # Execute the current action
    if performing_action:
        action_dict[current_action]
        performing_action = False


    previous_track_type = track_type

    # Display camera feed with text overlay
    frame = got.read_camera_data()
    if frame is not None:
        nparr = np.frombuffer(frame, np.uint8)
        data = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        text = f"{offset} {x_center} {y_center}"
        position = (50, 50)  # Adjust position if needed
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (255, 255, 255)  # White color in BGR
        thickness = 2

        # Put the text on the image
        cv2.putText(data, text, position, font, font_scale, color, thickness)

        cv2.imshow("frame", data)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit on 'q' key
            break

# Release resources
got.close_camera()
cv2.destroyAllWindows()
