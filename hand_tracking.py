import cv2
import mediapipe as mp

# Initialize video capture
cap = cv2.VideoCapture(0)

# Initialize Mediapipe hands solution
mpHands = mp.solutions.hands
hands = mpHands.Hands(min_detection_confidence=0.75, min_tracking_confidence=0.75)
mpDraw = mp.solutions.drawing_utils

# Finger tip landmarks (index of each finger's tip in Mediapipe Hands model)
tipIds = [4, 8, 12, 16, 20]

while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)  # Flip horizontally for a mirror view
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
            lmList = []
            h, w, c = img.shape
            for id, lm in enumerate(handLms.landmark):
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmList.append((id, cx, cy))

            if lmList:
                fingers = []

                # Thumb detection logic
                # Check if thumb tip is higher than its lower joint (relative to horizontal flipping)
                # if lmList[tipIds[0]][1] > lmList[tipIds[0] - 1][1]:
                #     fingers.append(1)  # Thumb is up
                # else:
                #     fingers.append(0)

                # Fingers: Check if tip is above the lower joint
                for id in range(1, 5):
                    if lmList[tipIds[id]][2] < lmList[tipIds[id] - 2][2]:
                        fingers.append(1)  # Finger is up
                    else:
                        fingers.append(0)

                totalFingers = fingers.count(1)
                print(f"Fingers up: {totalFingers}")

            mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

    # Display the image
    cv2.imshow("Image", img)

    # Exit on pressing 'q'
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
