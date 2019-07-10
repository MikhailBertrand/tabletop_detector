#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import cv2

# All the 6 methods for comparison in a list
# METHODS = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
#            'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
METHODS = ['cv2.TM_SQDIFF_NORMED']


def template_match_test_all(target_image_directory):
    template_red = cv2.imread('/home/gisen/ros/data/template_image.jpg', 0)
    template_green = cv2.imread('/home/gisen/ros/data/template_image_green.jpg', 0)
    temp = os.listdir(target_image_directory)
    target_images = []
    for item in temp:
        if item[-4:] == '.jpg':
            target_images.append(target_image_directory + item)
    image_id = 0
    for target in target_images:
        image_id += 1
        img = cv2.imread(target, 0)
        img_color = cv2.imread(target, 1)
        img2 = img.copy()
        for item in METHODS:
            img = img2.copy()
            meth = eval(item)
            top_left_r, bottom_right_r = return_matching_result(img, template_red, meth, threshold=None)
            top_left_g, bottom_right_g = return_matching_result(img, template_green, meth)
            if top_left_r:
                cv2.rectangle(img_color, top_left_r, bottom_right_r, (0, 0, 255), 2)
            if top_left_g:
                cv2.rectangle(img_color, top_left_g, bottom_right_g, (0, 255, 0), 2)
            save_file = '{}{}_result/{}_{}.jpg'.format(target_image_directory, item[4:], str(image_id).zfill(5), item[4:])
            print 'Saving image at {}'.format(save_file)
            cv2.imwrite(save_file, img_color)
            # cv2.imwrite(save_file, img)


def return_matching_result(img, temp_img, meth, threshold=None):
    width, height = temp_img.shape[::-1]
    res = cv2.matchTemplate(img, temp_img, meth)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    if meth in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
        if threshold and min_val <= threshold:
            pass
        else:
            top_left = min_loc
            bottom_right = (top_left[0] + width, top_left[1] + height)
            return top_left, bottom_right
    else:
        if threshold and max_val >= threshold:
            pass
        else:
            top_left = max_loc
            bottom_right = (top_left[0] + width, top_left[1] + height)
            return top_left, bottom_right
    return None, None


if __name__ == '__main__':
    directory = '/home/gisen/ros/data/saved_image/1525/'
    for method in METHODS:
        try:
            os.mkdir(directory + method[4:] + '_result')
        except OSError:
            pass
    template_match_test_all(directory)
