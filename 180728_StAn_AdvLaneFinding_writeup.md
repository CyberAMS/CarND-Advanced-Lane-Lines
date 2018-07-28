# Project: Advanced Lane Finding

This project has been prepared by Andre Strobel.

The goal of this project is to find a vehicle's lane in a video stream. The video stream has been recorded with a forward facing camera that is mounted to the vehicle. The detected lane is then used to estimate the lateral curvature of the road and the lateral off center position of the vehicle.

Everything has been programmed in Python 3.

---

## Content

1. Camera Calibration
    1. Distortion correction based on several test images
1. Pipeline (test images)
    1. Distortion correction of actual image
    1. Creation of thresholded binary image
    1. Identification and use of perspective transform
    1. Identification of lane line polynomials
    1. Calculation of curvature and off center position
1. Pipeline (video)
1. Discussion

[//]: # (Image References)

[image1]: ./output_images/01_distortion.png "Distortion"
[image2]: ./output_images/02a_undistorted_road.png  "Undistorted Road"
[image3]: ./output_images/02b_binary.png "Binary Example"
[image4]: ./output_images/02c_perspective.png "Warp Example"
[image5]: ./output_images/02d_line_detection.png "Fit Visual"
[image6]: ./output_images/02d_poly_result.png "Output"
[video1]: ./output_images/output.mp4 "Video"

---

## 1. Camera Calibration

### i. Distortion correction based on several test images

The camera calibration was done using the function `camera_calibration` in the notebook `180728d_StAn_AdvLaneFinding`.

I used the `cv2.findChessboardCorners` function with every test picture that showed the full chessboard pattern. This gave me the position of the chessboard corners in every test picture. I used `cv2.calibrateCamera` to get the distortion matrix and coefficients based on the detected chessboard corners of every test picture.

An example of the output is shown in the following picture: 

![alt text][image1]

## 2. Pipeline (single images)

### i. Distortion correction of actual image

The distortion correction of the actual images is done within the function `process_video_frame` in the notebook `180728d_StAn_AdvLaneFinding`.

An example of an undistorted image is shown in the following picture:

![alt text][image2]

### ii. Creation of thresholded binary image

The creation of a thresholded binary images is done within the function `create_binary_image` in the notebook `180728d_StAn_AdvLaneFinding`. All other functions mentioned in this section belong to the same notebook.

This function is very flexible. It can apply thresholds on color, magnitude, sobel x, sobel y and the direction of the gradient of a picture. It can operate on gray scale or every color channel of the hls color space. The output of the individual threshold channels can be combined using `and` or `or`.

It is used within the function `process_video_frame` to identify white and yellow lane lines:

```python
binary_white = create_binary_image(warpedimage, bOR = False, 
    bhls_l = True, hls_l_thresh = (180, 255),
    bhls_l_sx = True, hls_l_sobel_kernel = 31, hls_l_sx_thresh = (20, 255),  
    bdisplay = bdisplaydetails)
    
binary_yellow = create_binary_image(warpedimage, bOR = False, 
    bhls_h = True, hls_h_thresh = (15, 70),
    bhls_s_sx = True, hls_s_sobel_kernel = 31, hls_s_sx_thresh = (15, 255), 
    bdisplay = bdisplaydetails)

binary = cbi((binary_white, binary_yellow), bOR = True, bdisplay = bdisplay)
```

The function `cbi` (combine binary image) allows to further combine the output of the `create_binary_image` function using `and` or `or`. It is used in the function `process_video_frame` to combine the thresholded binary images for white and yellow lane lines into a single thresholded binary image using `or`.

An example of a binary image for identifying white lane lines is shown in the following picture:

![alt text][image3]

### iii. Identification and use of perspective transform

The perspective transform to birdseye view is determined in the function `camera2birdseyeview` in the notebook `180728d_StAn_AdvLaneFinding`.

The necessary transformation is determined by a test picture with straight lane lines as shown in the following picture: 

![alt text][image4]

The corners of the green polygon were picked manually and used as source points. The destination points `dst` were defined as follows where `xsize` and `ysize` define the width and height of the picture in pixels and `offsetxpercent` defines how many pixels should be drawn around the transformed corners based on the original picture size:

```python
dstxmin = 0 + (xsize * offsetxpercent / 100)
dstxmax = xsize - 1 - (xsize * offsetxpercent / 100)
dstymin = 0 + (ysize * offsetypercent / 100)
dstymax = ysize - 1 - (ysize * offsetypercent / 100)
dst = np.float32([[dstxmin, dstymax], [dstxmin, dstymin], [dstxmax, dstymin], [dstxmax, dstymax]])
```

The perspective transform is used within the function `process_video_frame` in the notebook `180728d_StAn_AdvLaneFinding`. It generates a birdseye view of the road which is then used to create a thresholded binary image of the lane lines.

### iv. Identification of lane line polynomials

The function `sliding_window_detect` in the notebook `180728d_StAn_AdvLaneFinding` looks for the start of lane lines in the lower left and right areas of the thresholded binary image of white and yellow lane lines. After this it follows the lane lines using a sliding window algorithm and stores the main line point for each window as shown in the following picture:

![alt text][image5]

At the end it estimates the shape of the lane lines by fitting a polynomial through the detected line points. The resulting polynomials for left, right and all lane lines are then used in a mask that is transferred back to the original perspective and blended into the original image as shown in the following picture:

![alt text][image5]

This function is used within the function `process_video_frame` in the notebook `180728d_StAn_AdvLaneFinding` and can either be called to reuse a previous polynomial to define the location of the sliding windows, to reuse the last starting positions of the lowest windows or to start without any previous information.

### v. Calculation of curvature and off center position

The calculation of lateral curvature of the lane `NewAllLines.radius_of_curvature` and off center position of the vehicle within the lane `potentialoffcenter` are done within the `sliding_window_detect` function in the notebook `180728d_StAn_AdvLaneFinding`.

The curvature `NewAllLines.radius_of_curvature` is calculated using the following code where `all_curverad` is a vector containing the curvature at different vertical positions, `all_line_poly` is the polynomial of the average lane line, `line_y` are the vertical positions considered and `ym_per_pix` is the factor between pixels and meters:

```python
all_curverad = ((1 + (2 * all_line_poly[0] * line_y * ym_per_pix + all_line_poly[1])**2)**1.5) / np.absolute(2 * all_line_poly[0])

NewAllLines.radius_of_curvature = np.average(all_curverad)
```

The off center position of the vehicle within the lane `potentialoffcenter` is calculated using the following code where `xsize` is the width of the image and `xpeak_right` and `xpeak_left` mark the starting positions of the right and left lane lines at the bottom.

```python
potentialoffcenter = ((xsize - xpeak_right) - (xpeak_left - 0))
```

The value of `potentialoffcenter` is stored in `offcenter` and `NewAllLines.line_base_pos` if the algorithm successfully detected lane lines. The following code is used to plot the string `text` containing the off center position in meters using the conversion `xm_per_pix`:

```python
text = 'Lateral offset in m: ' + str((np.int(offcenter * xm_per_pix * 100) / 100))
```

The horizontal and vertical ratios between pixels and meters `xm_per_pix` and `ym_per_pix` are manually determined in the function `pixels2realworld` in the notebook `180728d_StAn_AdvLaneFinding`.

---

## 3. Pipeline (video)

Here is the final video output which has been generated using my code: 

![alt text][video1]

Here's a [link to my video result](./output_images/output.mp4)

---

## 4. Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  
