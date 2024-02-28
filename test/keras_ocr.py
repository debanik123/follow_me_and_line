import matplotlib.pyplot as plt

import keras_ocr

# keras-ocr will automatically download pretrained
# weights for the detector and recognizer.
pipeline = keras_ocr.pipeline.Pipeline()

# # Load an image
# image_path = 'one.png'
# image = plt.imread(image_path)

# # Detect text regions in the image
# boxes = detector.detect(images=[image])[0]

# # Recognize text within the detected regions
# prediction_groups = recognizer.recognize(images=[image], detection_boxes=[boxes])

# # Display the results
# recognizer.drawAnnotations(image=image, predictions=prediction_groups[0])
# plt.imshow(image)
# plt.show()