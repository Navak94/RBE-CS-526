import cv2
import mediapipe as mp

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils


def is_thumb_up(hand_landmarks, image_height, image_width):
    """
    Determine if the gesture is a thumbs-up.
    """
    thumb_tip = hand_landmarks.landmark[4]
    thumb_base = hand_landmarks.landmark[2]

    # Calculate distances
    thumb_tip_y = thumb_tip.y * image_height
    thumb_base_y = thumb_base.y * image_height

    # Check if thumb is extended and other fingers are curled
    thumb_extended = thumb_tip_y < thumb_base_y  # Thumb tip is above the base
    fingers_curled = all(
        hand_landmarks.landmark[tip].y * image_height > hand_landmarks.landmark[base].y * image_height
        for tip, base in [(8, 6), (12, 10), (16, 14), (20, 18)]
    )

    return thumb_extended and fingers_curled


def is_fist(hand_landmarks, image_height):
    """
    Determine if the gesture is a fist.
    """
    return all(
        hand_landmarks.landmark[tip].y * image_height > hand_landmarks.landmark[base].y * image_height
        for tip, base in [(4, 3), (8, 6), (12, 10), (16, 14), (20, 18)]
    )


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

        if results.multi_hand_landmarks:
            height, width, _ = frame.shape
            for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                # Detect gestures
                if is_thumb_up(hand_landmarks, height, width):
                    gesture = "Thumbs Up"
                elif is_fist(hand_landmarks, height):
                    gesture = "Fist"
                else:
                    gesture = "Unknown Gesture"

                # Print gesture in the console
                handedness = results.multi_handedness[idx].classification[0].label  # "Left" or "Right"
                print(f"{handedness} hand: {gesture}")

                # Draw landmarks
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        cv2.imshow("Gesture Recognition", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
