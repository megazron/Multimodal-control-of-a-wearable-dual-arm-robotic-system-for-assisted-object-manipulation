# 20260830_073141 -- right_gripper

Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0

29 of 33 stages produced a result. [Back to the index](GALLERY.md)

### 01  The frame, as the camera delivered it  [ok]

![raw_colour](preview/right_gripper/01_raw_colour.jpg)

**Formula** `none -- this is the input every later stage is a function of`

**Measured** 1280 x 720 px | focus (variance of Laplacian) 98 | 4.04% of pixels clipped white, 0.14% crushed black

### 02  Intrinsics and the ray each pixel stands for  [ok]

![intrinsics](preview/right_gripper/02_intrinsics.jpg)

**Formula** `bearing = ((u - cx)/fx, (v - cy)/fy);  X = bearing * Z`

**Measured** fx 1297.67  fy 1298.63  cx 620.91  cy 238.28 | field of view 52.5 x 31.0 deg | principal point is 123.2 px from the image centre, which is 95 mm of sideways error at 1 m if you assume the centre

### 03  BGR to HSV, the space the colour tests live in  [ok]

![hsv](preview/right_gripper/03_hsv.jpg)

**Formula** `V = max(B,G,R);  S = (V - min(B,G,R))/V;  H = the sector of the RGB hexagon, 0..179 in OpenCV (not 0..359)`

**Measured** median H 117  S 30  V 139 | 70.7% of pixels are below S=80 and cannot satisfy any colour term in this repository

### 04  The pick path's own green threshold  [ok]

![green_threshold](preview/right_gripper/04_green_threshold.jpg)

**Formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`

**Measured** 12357 px selected (1.341% of the frame) | of those, median S 139 and median V 116

### 05  What the 9x9 opening removes  [ok]

![morphology](preview/right_gripper/05_morphology.jpg)

**Formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`

**Measured** 12357 px in, 12206 px out, 151 px removed (1.2%) | 8 separate blobs before, 1 after

### 06  Connected components, and their statistics  [ok]

![components](preview/right_gripper/06_components.jpg)

**Formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`

**Measured** 1 components over 40 px | largest 12206 px | this is where a MASK becomes a set of candidate OBJECTS

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](preview/right_gripper/07_vocabulary.jpg)

**Formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`

**Measured** white 248739 | black 74880 | orange 49207 | blue 38351 | cyan 30069 | red 25576

### 08  The region-MEAN ratio test, and its near-black guard  [ok]

![region_mean](preview/right_gripper/08_region_mean.jpg)

**Formula** `green  <=>  mean_G > 1.25 * mean_B  and  mean_G > 1.8 * mean_R,  refused outright when max(B,G,R) < 25`

**Measured** 1 of 1 regions pass

### 09  Rotated box, in-image yaw, and the degeneracy gate  [ok]

![minarearect](preview/right_gripper/09_minarearect.jpg)

**Formula** `minAreaRect -> ((cx,cy), (w,h), angle);  yaw is published as q = (0, 0, sin(angle/2), cos(angle/2)) ONLY when max(w,h)/min(w,h) >= 1.15`

**Measured** 125 x 114 px, 11.0 deg, aspect 1.09, identity (not measured)

### 10  The size-at-range gate  [ok]

![size_at_range](preview/right_gripper/10_size_at_range.jpg)

**Formula** `z = fx * object_width / px_across;  accept iff 0.25 <= z <= 1.50 m.  With depth the honest form is used instead: compare px against fx * size / measured_depth, tolerance 0.55 to 1.80`

**Measured** 125 px -> 0.417 m accept

### 11  FastSAM: every region in the picture, no prompt  [ok]

![fastsam](preview/right_gripper/11_fastsam.jpg)

**Formula** `object-agnostic instance masks; no class, no prompt, no scene knowledge -- 'what are the regions', not 'where is the green cube'`

**Measured** 39 regions | largest 265965 px (28.9% of frame), smallest 308 px

### 12  The area filter: noise below, the table above  [ok]

![area_filter](preview/right_gripper/12_area_filter.jpg)

**Formula** `keep iff  60 px <= area <= 0.35 * W * H  (= 322559 px here)`

**Measured** 39 in, 39 kept, 0 too small, 0 too large

### 13  Segment everything, then pick the one that matches  [ok]

![segment_match](preview/right_gripper/13_segment_match.jpg)

**Formula** `for each region: mean BGR over the MASK, then the ratio test of stage 08; score = min(1, area/2000)`

**Measured** 13034 px at (703, 343), mean BGR [94.5, 137.5, 68.6]

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](preview/right_gripper/14_yoloworld.jpg)

**refused** YOLO-World ran in 11.7 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

### 15  The depth frame, and where it has no answer  [ok]

![raw_depth](preview/right_gripper/15_raw_depth.jpg)

**Formula** `one 16-bit value per pixel, in MILLIMETRES on the wire, divided by 1000 here. 0 means 'no return' and is NOT a distance of zero.`

**Measured** 480 x 270 | 80.5% of pixels have a return | median 1.838 m, 5th-95th 0.339-5.105 m | 42779 px between the 0.08 and 1.50 m gate the pick path uses

### 16  Depth put into the colour frame  [ok]

![alignment](preview/right_gripper/16_alignment.jpg)

**Formula** `P = ((u-cx_d)z/fx_d, (v-cy_d)z/fy_d, z);  P' = P + t;  u' = fx_c X'/Z' + cx_c,  v' = fy_c Y'/Z' + cy_c`

**Measured** 60377 points projected, 43864 fell outside the colour frame, 58.8% of colour pixels got a depth

### 17  Pixels and depth become a cloud  [ok]

![deprojection](preview/right_gripper/17_deprojection.jpg)

**Formula** `X = (u - cx) Z / fx,   Y = (v - cy) Z / fy,   Z = depth`

**Measured** 42779 points in the 0.08-1.50 m band | extent X -0.284 to 0.442, Y -0.182 to 0.180, Z 0.299 to 0.927 m

### 18  RANSAC: the support plane  [ok]

![ransac](preview/right_gripper/18_ransac.jpg)

**Formula** `draw 3 points -> n = (b-a) x (c-a) / |...|,  d = -n.a;  score = #{ |n.P + d| < 6 mm };  keep the best of 400 draws`

**Measured** normal (0.1263, -0.9409, -0.3143), offset 0.2981 m | 22584 of 42779 points are inliers (52.8%) | RMS 1.59 mm | 71.7 deg between the plane normal and the camera's optical axis

### 19  The PCA refinement, and the NaN it once produced  [ok]

![pca_refine](preview/right_gripper/19_pca_refine.jpg)

**Formula** `C = cov(Q - mean(Q)) for inliers Q;  n = eigvec(C)[:, argmin];  d = -n . mean(Q)   <-- BOTH halves, together`

**Measured** the normal moved 0.000 deg | RMS 1.59 mm -> 1.59 mm | mixing the refined normal with the RANSAC offset selects 22584 inliers instead of 22584

### 20  Height above the plane  [ok]

![height_map](preview/right_gripper/20_height_map.jpg)

**Formula** `h(P) = n . P + d,  signed;  positive is off the surface, toward the camera`

**Measured** 20193 of 42779 points stand more than 5 mm off the plane (47.20%) | tallest 357.1 mm

### 21  The surface height as a MODE, not a mean  [ok]

![mode_surface](preview/right_gripper/21_mode_surface.jpg)

**Formula** `bin at 2 mm; take the fullest bin; require it to hold >= 20% of the points; refine as the mean of everything within +/-6 mm of it`

**Measured** modal bin holds 24.6% of 42779 points (floor 20%) | refined -0.2981 m, spread 1.65 mm | a plain MEAN would say -0.2512 m, which is 46.9 mm out | RANSAC's own offset is -0.2981 m, 0.07 mm from this

### 22  What is standing on the surface  [ok]

![on_surface](preview/right_gripper/22_on_surface.jpg)

**Formula** `{ pixels whose lifted point has h > 5 mm }, 8-connected`

**Measured** 152263 px, 103448 pts, top 64 mm | 38812 px, 26656 pts, top 224 mm | 13640 px, 9771 pts, top 228 mm | 727 px, 538 pts, top 12 mm | 68 px, 58 pts, top 7 mm | 62 px, 54 pts, top 9 mm

### 23  The measurement the pick actually acts on  [ok]

![cube_measurement](preview/right_gripper/23_cube_measurement.jpg)

**Formula** `cube = colour(u,v) AND h > 5 mm;  foot = mean(Pk) - n (n.mean(Pk) + d);  centre = foot + n * max(h)/2`

**Measured** 160 object points | top 55.0 mm above the plane | centre (0.0511, 0.0519, 0.7260) m in the camera frame | plane RMS 1.59 mm, inlier fraction 0.53

### 24  Splitting a cube from the pad it stands on  [ok]

![layers](preview/right_gripper/24_layers.jpg)

**Formula** `find the MODAL height bin (5 mm bins); cut 12 mm above it; recurse on what is left, twice`

**Measured** layer 1: 4-41 mm, 46552 points | layer 2: 41-53 mm, 14079 points | layer 3: 53-64 mm, 8239 points

### 25  Rotating calipers, against the two controls  [ok]

![min_width](preview/right_gripper/25_min_width.jpg)

**Formula** `width = min over theta of [ max(p.u(theta)) - min(p.u(theta)) ],  u(theta) = (cos, sin) in the support plane, swept at 0.25 deg`

**Measured** calipers 192.0 mm | PCA's short axis 221.3 mm | axis-aligned box 241.7 mm | long axis 390.4 mm, height 60.2 mm | jaws close on 85 mm, so this is TOO WIDE

### 26  Dropping masks that are unions of finer ones  [ok]

![nested](preview/right_gripper/26_nested.jpg)

**Formula** `voxelise each instance at 5 mm; accept smallest first; reject one whose claimed-voxel overlap exceeds 0.55`

**Measured** 25 instances in, 20 kept, 5 dropped | overlaps: 0.90, 0.96, 1.00, 1.00, 0.76

### 27  The parallel-jaw grasp  [refused]

![grasp](preview/right_gripper/27_grasp.jpg)

**refused** the grasp planner refused, which is an ANSWER: object is 110 mm across its narrowest axis; the Robotiq 85 opens 75 mm usable. It cannot be grasped as it lies.

### 28  Fiducials: the primary detector  [refused]

![apriltag](preview/right_gripper/28_apriltag.jpg)

**refused** no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That is an answer, not a failure: a tag either decodes or it does not, which is exactly why this is the PRIMARY detector and colour is the capped fallback. Detector used: cv2.aruco.ArucoDetector (OpenCV 4.11.0).

### 29  The support surface, as an object  [ok]

![table](preview/right_gripper/29_table.jpg)

**Formula** `fit the plane, keep its inliers, span them in the plane's own basis; extent taken at the 2nd and 98th percentile so one stray return cannot stretch it`

**Measured** 0.428 x 0.467 m of surface, 0.290 m away, flat to 1.69 mm

### 30  Every object, as an oriented 3-D box  [ok]

![boxes3d](preview/right_gripper/30_boxes3d.jpg)

**Formula** `centre = mid-extent ACROSS the surface + (foot + top/2) ALONG the normal; axes = (minimum-width direction, its perpendicular, the plane normal); size = the spans along them`

**Measured** 12 objects, 0.387 to 0.897 m away

### 31  How far apart everything is  [ok]

![distances](preview/right_gripper/31_distances.jpg)

**Formula** `range = |centre| (the camera is the origin); height = n.c + d; separation = |c_i - c_j|; free gap subtracts each object's half-width`

**Measured** 10 objects, nearest 0.394 m, closest pair 0.006 m

### 32  People in the picture  [refused]

![people](preview/right_gripper/32_people.jpg)

**refused** no person is in this frame. MediaPipe ran in 5.5 s and found nobody, which is an answer -- the wearer-tracking safety case only ever makes the modelled body BIGGER, so 'nobody seen' falls back to the mannequin rather than to an empty scene.

### 33  The robot in the picture, and what is NOT detected  [ok]

![self_view](preview/right_gripper/33_self_view.jpg)

**Formula** `{ pixels with 0 < depth < 0.08 m } -- nearer than anything this camera works on`

**Measured** 0 px nearer than 0.08 m

