import cv2
import mediapipe as mp

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils


def is_finger_open(hand_landmarks, finger_tip, finger_base, image_height):
    """
    Determine if a finger is open based on the Y-axis distance
    between the fingertip and its base joint.
    """
    tip_y = hand_landmarks.landmark[finger_tip].y * image_height
    base_y = hand_landmarks.landmark[finger_base].y * image_height
    return tip_y < base_y  # Finger is open if the tip is above the base


def is_hand_open(hand_landmarks, image_height):
    """
    Determine if a hand is open or closed.
    """
    # Thumb (Tip: 4, Base: 3)
    thumb_open = is_finger_open(hand_landmarks, 4, 3, image_height)

    # Index finger (Tip: 8, Base: 6)
    index_open = is_finger_open(hand_landmarks, 8, 6, image_height)

    # Middle finger (Tip: 12, Base: 10)
    middle_open = is_finger_open(hand_landmarks, 12, 10, image_height)

    # Ring finger (Tip: 16, Base: 14)
    ring_open = is_finger_open(hand_landmarks, 16, 14, image_height)

    # Pinky finger (Tip: 20, Base: 18)
    pinky_open = is_finger_open(hand_landmarks, 20, 18, image_height)

    # Count open fingers
    open_fingers = [thumb_open, index_open, middle_open, ring_open, pinky_open]
    return sum(open_fingers) >= 4  # Hand is open if at least 4 fingers are open


cap = cv2.VideoCapture(0)
with mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7) as hands:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Flip for a mirror view
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb_frame)

        # Initialize a list to hold the states of both hands
        hand_states = []

        if results.multi_hand_landmarks:
            height, _, _ = frame.shape
            for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                # Determine handedness (left or right)
                handedness = results.multi_handedness[idx].classification[0].label  # "Left" or "Right"

                # Determine open/closed state
                if is_hand_open(hand_landmarks, height):
                    hand_state = f"open {handedness.lower()} hand"
                else:
                    hand_state = f"closed {handedness.lower()} hand"

                # Add to hand states
                hand_states.append(hand_state)

                # Draw landmarks
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        # Clear the console line and print updated hand states
        print("\r" + " " * 80, end="")  # Clear the current line
        print("\r" + " | ".join(hand_states), end="")

        cv2.imshow("Hand Open/Closed Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
