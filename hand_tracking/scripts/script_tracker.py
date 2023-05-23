#!/usr/bin/env python3
import mediapipe as mp
import numpy as np
import cv2

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

cap = cv2.VideoCapture(0)
ret, test = cap.read()
image_height, image_width, c = test.shape 
coordinates_hands = np.ndarray([],[])

with mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5) as hands: 
    while cap.isOpened():
        ret, frame = cap.read()
        
        # BGR 2 RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Flip on horizontal
        image = cv2.flip(image, 1)
        # Set flag
        image.flags.writeable = False
        # Detections
        results = hands.process(image)
        # Set flag to true
        image.flags.writeable = True
        # RGB 2 BGR
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # Get image shape

        
        # Rendering results
        if results.multi_hand_landmarks:
            for num, hand in enumerate(results.multi_hand_landmarks):
                mp_drawing.draw_landmarks(image, hand, mp_hands.HAND_CONNECTIONS, 
                                        mp_drawing.DrawingSpec(color=(121, 22, 76), thickness=2, circle_radius=4),
                                        mp_drawing.DrawingSpec(color=(250, 44, 250), thickness=2, circle_radius=2),
                                         )
                
            #
            
            c = 30
            i = 0
            for hand_landmarks in results.multi_hand_landmarks:
                coordinates_hands = np.append(hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x * image_width, hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y * image_height)
                text = (coordinates_hands[0], coordinates_hands[1])
                text = str(text)
                coordinates = (30,c)
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 0.5
                color = (0,0,0)
                thickness = 1
                image = cv2.putText(image, text, coordinates, font, fontScale, color, thickness, cv2.LINE_AA)
                c = c+20
                i = i+1              

        # Show image    
        cv2.imshow('Hand Tracking', image)
        
        if cv2.waitKey(10) & 0xFF == ord(' '):
            # print(results.multi_hand_landmarks)
            if results.multi_hand_landmarks is not None:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Position of landmark in middle finger MCP
                    
                    print( coordinates_hands[0], coordinates_hands[1])
                    #("Dimensions: ", image_width, image_height)
        elif cv2.waitKey(10) & 0xFF == ord('q'):
            break
        
        print(coordinates_hands)  
        coordinates_hands = np.empty

cap.release()
cv2.destroyAllWindows()