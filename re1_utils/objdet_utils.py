import random
import cv2
import numpy as np
import yaml
from pprint import pprint

LABEL_PATH = './yolov7/data/coco.yaml'

def load_config(config_path):
    with open(config_path, "r") as stream:
        try:
            config_dict = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    return config_dict

def load_label():
    labels = load_config(LABEL_PATH)['names']
    label_dict = {}
    for index, label in enumerate(labels):
        label_dict[index] = {}
        label_dict[index]['label'] = label
        label_dict[index]['color'] = [random.randint(0, 255) for _ in range(3)] 
    # pprint(label_dict)
    return label_dict

def plot_one_box(x, img, color=None, label=None, line_thickness=3):
    # Plots one bounding box on image img
    tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))

    cv2.rectangle(img = img, pt1 = c1, pt2 = c2, color = color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img = img, pt1 = c1, pt2 = c2, color = color, thickness=-1, lineType =cv2.LINE_AA)  # filled
        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

def plot_all_boxes(preds, img, label_dict = None):
    #default is coco labels
    label_dict = label_dict or load_label()
    
    for pred in preds:
        plot_one_box(pred, img, label=label_dict[pred[-1]]['label'], color=label_dict[pred[-1]]['color'], line_thickness=1)