import tensorflow 
import cv2
import numpy as np
import os

confidence_threshold = 0.80
piece = 0
model = tensorflow.keras.models.load_model("model_class_object.h5", compile=False)
camera = cv2.VideoCapture(0)

def image_process():
    ret, image = camera.read()
    # Resize the raw image into (224-height,224-width) pixels
    image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)
    # Change brightness and contrast of image
    image = cv2.addWeighted(image, 3., image, 0., 1.)      
    cv2.imshow("Object classification", image)  
    # Make the image a numpy array and reshape it to the models input shape.
    image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
    # Normalize the image array
    image = (image / 127.5) - 1
    return(image)

def object_class(image):
    # Predict class
    prediction = model.predict(image, verbose=0)
    # Obtain value with higher confidence score
    index = np.argmax(prediction)
    confidence_score = prediction[0][index]
    # Predict class
    if confidence_score >= confidence_threshold:
        if index == 0:
            piece = 1
        if index == 1:
            piece = 2
        if index == 2:
            piece = 3
        if index == 3:
            piece = 4
        if index == 4:
            piece = 5
        if index == 5:
            piece = 6
        if index == 6:
            piece = 7
    else:
        piece = 0
    return(piece, confidence_score)        

while (True):
        
    image = image_process()
    piece, confidence_score = object_class(image)

    ## Delete for node

    os.system("cls")
    print("Piece: ", piece, " Confidence score: ", confidence_score)
    keyboard_input = cv2.waitKey(10)
    if keyboard_input == 27 :
        break

    ##

camera.release()
cv2.destroyAllWindows()