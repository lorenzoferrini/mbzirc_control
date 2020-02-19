import cv2
import os
import sys
from datetime import datetime

if __name__ == "__main__":
    fps = 30
    if len(sys.argv) != 2:
        print 'Error'
        print 'usage:'
        print 'img_to_video "path to image folder"'
    else:
        image_folder=sys.argv[1]
        images = [img for img in os.listdir(image_folder) if img.endswith(".jpg")]
        frame = cv2.imread(os.path.join(image_folder, images[0]))
        height, width, layers = frame.shape
        video = cv2.VideoWriter(os.path.join(os.getcwd(),sys.argv[1], sys.argv[1].strip('/')+'.avi'), cv2.VideoWriter_fourcc(*'XVID'), fps, (width, height))

        for image in sorted(images):
            #frame = cv2.cvtColor(cv2.imread(os.path.join(image_folder, image)),cv2.COLOR_BGR2RGB)
            frame = cv2.imread(os.path.join(image_folder, image))
            video.write(frame)
        video.release()
