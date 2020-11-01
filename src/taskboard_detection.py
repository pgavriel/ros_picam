#!/usr/bin/env python

import os, sys
import rospy
import math
import numpy as np
import apriltag
import imutils
import cv2
from PIL import Image


# Custom drawing function, can highlight a particular line and draw a centerpoint
def draw_poly(image, pts, highlight=1, c1=(0,255,0), c2=(0,0,255), center=True):
    for i in range(0,len(pts)):
        if i == highlight:
            color = c2
        else:
            color = c1
        image = cv2.line(image,(pts[i][0],pts[i][1]),(pts[i-1][0],pts[i-1][1]),color,2)
    if center:
        ctr_pt = [int(sum(x)/len(x)) for x in zip(*pts)]
        image = cv2.circle(image,(ctr_pt[0],ctr_pt[1]),2,c1,-1)
    return image


# Custom drawing function for drawing coordinates points as small circles
def draw_points(image, pts, color=(0,255,0), radius=2, thickness=-1):
    for p in pts:
        image = cv2.circle(image,(p[0],p[1]),radius,color,thickness)
    return image


# Scale image size by given percent
def scale_image(image, percent):
    width = int(image.shape[1] * percent / 100)
    height = int(image.shape[0] * percent / 100)
    dim = (width,height)
    resized = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
    return resized


# Returns board points re-ordered to align with the apriltag corners
def sortpoints(board_corners,tag_corners):
    rospy.loginfo("SORTING POINTS:")
    tag = tag_corners.copy()
    board = board_corners.copy()
    ordered = np.zeros((4,2))
    rospy.loginfo("Tag Corners: \n{}".format(tag_corners))
    rospy.loginfo("Unordered Board Corners: \n{}".format(board_corners))
    for i in range(0,len(board)):
        dists = []
        for p in tag:
            dists.append(math.sqrt( ((board[i][0]-p[0])**2)+((board[i][1]-p[1])**2) ))
        minpos = dists.index(min(dists))
        ordered[minpos] = board[i]
        rospy.loginfo("{} - Dists: {} - MinPos: {}".format(i,dists,minpos))

    rospy.loginfo("Ordered Board Corners: \n{}".format(ordered))
    ordered = np.array(ordered, dtype=np.uint32)
    return ordered


# Takes a greyscale image and returns found apriltags
def find_apriltag(gray_img):
    detector = apriltag.Detector()
    result = detector.detect(gray_img)
    return result


# Finds the largest contour and it's centerpoint from an input image
def getLargestContourCenter(mat):
    gray = cv2.cvtColor(mat,cv2.COLOR_BGR2GRAY)
    if imutils.is_cv2() or imutils.is_cv4():
        (contours, _) = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    elif imutils.is_cv3():
        (_, contours, _) = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 0:
        c = max(contours, key = cv2.contourArea)
        area = cv2.contourArea(c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(mat, [box], 0,(0,255,0),1)
        center = [int(sum(x)/len(x)) for x in zip(*box)]
        mat[center[0],center[1]] = (255,0,0)
        rospy.loginfo("BOX:\n{}".format(box))
        rospy.loginfo("CENTER: {}".format(center))

    else:
        return mat, None

    return mat, center

# Big mess of a function, most of the messiness comes from trying to create a
# useful debug image to reveal whats going on.
# Looks at square regions around the rough corners, and uses the intersections of
# detected HoughLines (the edges of the board hopefully) to return more precise
# positions of the boards corners.
def refine_corners(image,corners,box_size=50,line_thresh=110):
    rospy.loginfo("REFINING CORNERS:")
    # Mask the image, removing details around the taskboard.
    kernel = np.ones((box_size,box_size),np.uint8)
    mask = np.zeros((image.shape[0],image.shape[1]),np.uint8)
    c = corners.copy()
    c = c.astype(np.int32)
    c = cv2.convexHull(c)
    cv2.fillConvexPoly(mask, c, color=(255,255,255))
    mask = cv2.dilate(mask,kernel,iterations=1)
    image = cv2.bitwise_and(image,image,mask = mask)
    # img = Image.fromarray(image)
    # img.save(os.path.join('/home/ubuntu/catkin_ws/src/ros_picam/captures/debug/',"masked.png"))

    image_bgr = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
    edges = cv2.Canny(image,100,200)
    stage2_img = edges.copy()
    stage2_img = cv2.cvtColor(stage2_img,cv2.COLOR_GRAY2BGR)
    lines = cv2.HoughLines(edges, 1, np.pi / 180, line_thresh, None, 0, 0)
    line_mat = edges.copy()
    line_mat[:,:] = 0
    # Draw any lines that are found
    if lines is not None:
        rospy.loginfo("Lines found: {}".format(len(lines)))
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(line_mat, pt1, pt2, 255, 1, cv2.LINE_AA)
            cv2.line(stage2_img, pt1, pt2, (0,0,255), 1, cv2.LINE_AA)
    harris = cv2.cornerHarris(line_mat,2,3,0.04)
    harris = cv2.dilate(harris,None)
    ret, harris = cv2.threshold(harris,0.01*harris.max(),255,0)
    harris = np.uint8(harris)
    harris_bgr = cv2.cvtColor(harris,cv2.COLOR_GRAY2BGR)
    line_mat_bgr = cv2.cvtColor(line_mat,cv2.COLOR_GRAY2BGR)
    # These lists are for storing small images of each corner at different stages (combined later)
    c_gray = []
    c_line = []
    c_line_mat = []
    c_harris = []
    rospy.loginfo("CORNERS {}:".format(corners.shape))
    for c in corners:
        x, y = int(c[0]-(box_size/2)), int(c[1]-(box_size/2))
        rospy.loginfo("X:{}\tY:{}".format(x,y))
        # Draw bounding boxes around examined corner regions
        stage2_img = cv2.rectangle(stage2_img,(x,y),(x+box_size,y+box_size),(0,255,255),2)
        # Copy those regions from each stage and save them for debug image
        found_corner = image_bgr[y:y+box_size,x:x+box_size]
        c_gray.append(found_corner)
        found_corner = stage2_img[y:y+box_size,x:x+box_size]
        c_line.append(found_corner)
        found_corner = line_mat_bgr[y:y+box_size,x:x+box_size]
        c_line_mat.append(found_corner)
        found_corner = harris_bgr[y:y+box_size,x:x+box_size]
        c_harris.append(found_corner)

    # Attempt to assemble new corner coordinates
    c_subpix = []
    for c, b in zip(c_harris,corners):
        xpos, ypos = int(b[0]-(c.shape[0]/2)), int(b[1]-(c.shape[1]/2))

        # If corner detection image is empty, just use the original corner (something is probably wrong)
        if not np.any(c):
            c_subpix.append((b[0],b[1]))
            rospy.loginfo("Corner {} region appears empty, defaulting to rough corner.".format(b))
            continue

        # Find the center of the largest contour
        c, ctr_pt = getLargestContourCenter(c)
        if ctr_pt is None:
            c_subpix.append([b[0],b[1]])
            rospy.logwarn("No center point returned.")
            continue

        xpos = int(xpos + ctr_pt[0])
        ypos = int(ypos + ctr_pt[1])
        c_subpix.append([xpos,ypos])


    stage2_img = draw_points(stage2_img,c_subpix,color=(255,0,0))
    #Attempt to combine all the corner images at different stages. Can throw an error
    # if the images are different sizes (i.e. corner was too close to the edge of the input image)
    try:
        stack1 = np.hstack(c_gray)
        stack2 = np.hstack(c_line)
        stack3 = np.hstack(c_line_mat)
        stack4 = np.hstack(c_harris)
        vstack = np.vstack((stack1,stack2,stack3,stack4))

        return stage2_img, c_subpix, vstack
    except Exception as e:
        rospy.logerror("Error with corner mats: {}".format(e))
        return stage2_img, c_subpix, None


# Tries to return coordinates for a rough bounding rectangle around the taskboard
def isolate_board(image,thresh=120,area_lb=0,area_ub=100000):
    rospy.loginfo("ISOLATING TASKBOARD:")
    area = 0
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    smooth = cv2.bilateralFilter(gray,9,75,75)
    ret,th1 = cv2.threshold(smooth,thresh,255,cv2.THRESH_BINARY)
    stage1_img = cv2.cvtColor(th1,cv2.COLOR_GRAY2BGR)
    if imutils.is_cv2() or imutils.is_cv4():
        (contours, _) = cv2.findContours(th1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    elif imutils.is_cv3():
        (_, contours, _) = cv2.findContours(th1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 0:
        c = max(contours, key = cv2.contourArea)
        area = cv2.contourArea(c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(stage1_img, [box], 0,(0,255,0),2)
        if area < area_lb or area > area_ub:
            print("Found contour was not within specified bounds, not returning corners.")
            print("Contour Area:",area,"\tLBound:",area_lb," UBound:",area_ub)
            box = None

    else:
        box = None

    rospy.loginfo("Bounding Box Found:\n{}".format(box))
    rospy.loginfo("Region Area: {}".format(area))
    return stage1_img, smooth, box


# Takes an image and four corners, outputs square warped image. Very costly.
def find_homography(image,corners,size=1000):
    rospy.loginfo("WARPING PERSPECTIVE: ")
    rospy.loginfo("Output Size: {}".format(size))
    pts1 = np.float32(corners)
    rospy.loginfo("PTS1: \n{}".format(pts1))
    pts2 = np.float32([[0,0],[size,0],[size,size],[0,size]])
    rospy.loginfo("PTS2: \n{}".format(pts2))
    M = cv2.getPerspectiveTransform(pts1,pts2)
    warped = cv2.warpPerspective(image,M,(size,size))
    return warped


# PRIMARY FUNCTION:
# Takes an image, finds the corners of the taskboard, and returns a square
# warped image of only the taskboard. Saves debug images to help understand the process.
def process_taskboard(image,thresh=80,scale_down_factor=4):
    path = '/home/ubuntu/catkin_ws/src/ros_picam/captures/debug/'
    # Scale image down for processing
    scaled = scale_image(image,(100/scale_down_factor))
    gray = cv2.cvtColor(scaled,cv2.COLOR_BGR2GRAY)
    # Attempt to detect apriltag
    tag = find_apriltag(gray)
    if tag == None:
        print("No apriltag found, unable to process.")
    else:
        print("Apriltag found.")
        # Apriltag corners
        tag_corners = tag[0].corners.astype('int32')
        # Find rough bounding box of taskboard based on largest contour
        stage1, smooth, box = isolate_board(scaled,thresh=thresh)
        # Sort box points to align with apriltag corners, this keeps ordering consistant
        box = sortpoints(box,tag_corners)
        # Refine the corner positions
        stage2, final_box, corner_debug = refine_corners(gray,box)
        scaled = draw_points(scaled,final_box,color=(255,0,0))
        # Scale corner coordinates back to original image size
        final_box_scaled = []
        for i in range(0,len(final_box)):
            final_box_scaled.append( (final_box[i][0] * scale_down_factor, final_box[i][1] * scale_down_factor) )
        # Warp original image based on refined corners
        warped = find_homography(image,final_box_scaled)

        # Save debugging images
        img = Image.fromarray(stage1)
        img.save(os.path.join(path,"1-stage1.png"))
        img = Image.fromarray(stage2)
        img.save(os.path.join(path,"2-stage2.png"))
        img = Image.fromarray(corner_debug)
        img.save(os.path.join(path,"3-corner_debug.png"))
        img = Image.fromarray(scaled)
        img.save(os.path.join(path,"4-final_corners.png"))

        return warped
