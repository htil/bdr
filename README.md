# Brain-Drone Race Line Detection

The Brain-Drone Race is a competition featuring users' cognitive ability and mental endurance. During this event competitors are required to out-focus opponents in a drone drag race influenced by electrical signals emitted from the brain.

This python class is designed to facilitate the detection of the line which the drones will follow. It allows for the analysis of an image, returning an error to represent the drone's deviation from the line. This system uses methodologies such as Gaussian Blur, Color-Based Filtering, Canny Edge Detection, and Hough Line Transform.

This BDR Line Detection class is developed by the Human Technology Interaction Lab at the University of Alabama.

### Methodology
The following image will be used as a sample test image to illustrate the workflow of the algorithm.

<img src="https://github.com/htil/bdr-line-detection/blob/master/images/straight_center.jpg" width="200">

**1. Gaussian Blur**

Since edge detection results are easily affected by image noise, it is essential to filter out image noise to prevent false edge detection. To smooth the image, a Gaussian filter is applied to the image, which slightly smooths the image to reduce the effects of obvious noise on the edge detector.

<img src="https://github.com/htil/bdr-line-detection/blob/master/images/sample_workflow/blur.jpg" width="200">

**2. HLS Conversion**

The line for the Brain-Drone Race is red. In order to isolate this line, it is necessary to choose the most suitable color space that clearly highlights the line. This allows the image to retain as much of the red line as possible while blacking out other noise in the image.

HLS is an alternative color space representation to the RGB color space. Based on testing of various color spaces, the HLS representation produced the clearest red line of all color spaces. 

<img src="https://github.com/htil/bdr-line-detection/blob/master/images/sample_workflow/hls.jpg" width="200">

**3. Color-Based Filtering**

Once the image has been converted to HLS, a mask is then applied to the image that filters based on a range of HLS values. The values for this mask were selected heuristically based on the general HLS range of red. When this mask is applied to the blurred image, the result is an image that consists of the isolated red line.

<img src="https://github.com/htil/bdr-line-detection/blob/master/images/sample_workflow/mask.jpg" width="200">

**4. Canny Edge Detection**

The Canny Edge Detection algorithm detects a wide range of edges in images by measuring the intensity gradients of each pixel. Following this process, an image is produced in which each side of the red line is clearly marked as an edge.

<img src="https://github.com/htil/bdr-line-detection/blob/master/images/sample_workflow/edges.jpg" width="200">

**5. Hough Line Transform**

The Hough Transform is a technique which can be used to isolate features of a particular shape within an image. It is used here to detect the edges of the red line in the image. These lines can then be visualized by overlaying them on the original image.

<img src="https://github.com/htil/bdr-line-detection/blob/master/images/sample_workflow/result.jpg" width="200">

**6. Error Calculation**

The error for the detected line is represented by two values δ and θ, where δ = the perpendicular distance from the line to the center of the image in pixels, and θ = the angle of the line in the image in radians.

In general, the line should be essentially vertical, assuming that the initial orientation of the drone is parallel to the line; however, the two-value error calculation allows for more robustness that would enable the drone to adjust its rotation in the case of misalignment due to inconsistencies in flight behavior. These error values are calculated as follows:

------

#### δ Calculation

If a line is defined by two points <img src="https://tex.s2cms.ru/svg/P_1%3D(x_1%2Cy_1)"/> 
and <img src="https://tex.s2cms.ru/svg/P_2%3D(x_2%2Cy_2)" /> then the distance from 
<img src="https://tex.s2cms.ru/svg/(x_0%2Cy_0)"/> to the line is given by

<img src="https://tex.s2cms.ru/svg/%5Cdelta%20%3D%20distance(P_1%2CP_2%2C(x_0%2Cy_0))%3D%5Cfrac%7B%7C(y_2-y_1)x_0-(x_2-x_1)y_0%2Bx_2y_1-y_2x_1%7C%7D%7B%5Csqrt%7B(y_2-y_1)%5E2%2B(x_2-x_1)%5E2%7D"/>

---

#### θ Calculation

Likewise, a mapping is defined from two points on the line to an angle in the range [0 rads, π rads] such that

<img src="https://tex.s2cms.ru/svg/%5Ctheta%20%3D%20%5C%5B%5Cleft%5C%7B%0A%5Cbegin%7Barray%7D%7Bll%7D%0A%20%20%20%20%20%20%5Cpi%2F2%20%26%20x_2-x_1%3D0%20%5C%5C%0A%20%20%20%20%20%20%5Carctan%7B(m)%7D%20%26%20x_2-x_1%5Cneq0%20%5C%5C%0A%5Cend%7Barray%7D%20%0A%5Cright.%20%5C%5D"/>

where <img src="https://tex.s2cms.ru/svg/m"/> is defined to be the slope of the line and <img src="https://tex.s2cms.ru/svg/range(%5Carctan(m))"/> = [0 rads, π rads]

---

