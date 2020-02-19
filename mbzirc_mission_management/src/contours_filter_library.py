import cv2
import math
import numpy as np

# Filtro per i contours, restituisce solo i contours che hanno almeno una determinata area ed un rapporto area perimetro
# compatibile con un palloncino
#
# parametri
def contours_area_shape_filter(frame, contours, min_area=500, percentage=0.15, min_index_of_circularity=0.95):
    list_result = []
    frame_height, frame_width = frame.shape[:2]
    for cnt in contours:
        area = cv2.contourArea(cnt)
        r = math.sqrt(area / math.pi)
        circonferenza = 2 * math.pi * r
        circonferenza_cnt = cv2.arcLength(cnt, True)
        if area > min_area and circonferenza <= circonferenza_cnt <= (circonferenza + (circonferenza * percentage)):
            black = np.zeros((frame_height, frame_width, 1), dtype="uint8")
            cv2.drawContours(black, [cnt], 0, 255, -1)
            momenti = cv2.moments(black, True)
            c = ((momenti['m00']) ** 2) / (2 * math.pi * (momenti['mu20'] + momenti['mu02']))

            if c >= min_index_of_circularity:
                list_result.append(cnt)
    return list_result


def contours_watershed_filter(frame, contours):
    list_final_Contourn = []
    frame_height, frame_width = frame.shape[:2]
    i=0
    for cnt in contours:
        black_frame = np.zeros((frame_height, frame_width, 1), dtype="uint8")
        cv2.drawContours(black_frame, [cnt], 0, 255, -1)
        x_rect, y_rect, w, h = cv2.boundingRect(cnt)
        x_prec = int(round(x_rect - (w / 2)))
        x2_prec = x_prec + (2 * w)
        y_prec = int(round(y_rect - (h / 2)))
        y2_prec = y_prec + (2 * h)

        height, width = frame.shape[:2]

        x = 0 if x_prec < 0 else x_prec
        y = 0 if y_prec < 0 else y_prec
        x2 = width - 1 if x2_prec > width - 1 else x2_prec
        y2 = height - 1 if y2_prec > height - 1 else y2_prec

        imCrop = frame[y:y2, x:x2]

        cropCopy=imCrop.copy()
        mask_crop = black_frame[y:y2, x:x2]


        kernel = np.ones((3, 3), np.uint8)
        opening = cv2.morphologyEx(mask_crop, cv2.MORPH_OPEN, kernel, iterations=1)

        sure_bg = cv2.dilate(opening, kernel, iterations=5)
        sure_fg = np.uint8(opening)
        unknown = cv2.subtract(sure_bg, sure_fg)
        ret, markers = cv2.connectedComponents(sure_fg)
        markers = markers + 1
        markers[unknown == 255] = 0
        markers = cv2.watershed(cropCopy, markers)
        markers[:, 0] = 1
        markers[0, :] = 1
        markers[-1, :] = 1
        markers[:, -1] = 1
        cropCopy[markers == -1] = [255, 0, 0]

        black_little_frame = np.zeros((y2-y, x2-x), np.uint8)
        black_little_frame[markers == -1] = 255

        contours_little, hieracy = cv2.findContours(black_little_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        img_contours_little = np.zeros((black_little_frame.shape[0], black_little_frame.shape[1], 1), dtype="uint8")
        cv2.drawContours(img_contours_little, contours_little, -1, 255, -1)

        momenti = cv2.moments(img_contours_little, True)
        c = ((momenti['m00']) ** 2) / (2 * math.pi * (momenti['mu20'] + momenti['mu02']))

        if c>=0.95:
            hu_moments = cv2.HuMoments(momenti).flatten()
            list_final_Contourn.append(cnt)
        i += 1
    return list_final_Contourn


def reference_contour(path):
    kernelOpen = np.ones((5, 5))
    kernelClose = np.ones((49, 49))

    lowerBound = np.array([114, 140, 100])
    upperBound = np.array([128, 255, 255])
    sagoma = cv2.imread(path)
    blurred = cv2.GaussianBlur(sagoma, (7, 7), 0)
    sagomaHSV = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(sagomaHSV, lowerBound, upperBound)
    maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
    maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)
    maskFinal = maskClose

    contours_sagoma, hieracy = cv2.findContours(maskFinal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contours_sagoma[0]


def contours_matchShape_nearest_balloon(contours,reference_contour):

    list_wright_shape=[]
    for cnt in contours:
        nearest=cv2.matchShapes(cnt,reference_contour,1,0)
        if nearest < 0.15:
            list_wright_shape.append(cnt)
    if list_wright_shape:
        return max(list_wright_shape, key=lambda r: cv2.contourArea(r))
    else:
        return []



