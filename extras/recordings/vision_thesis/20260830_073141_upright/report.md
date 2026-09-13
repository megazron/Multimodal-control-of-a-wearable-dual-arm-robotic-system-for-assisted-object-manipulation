# Computer vision stages, 2026-08-30T23:15:31

Written by `scripts/cv_pickpose_visuals.py`. The arms were NOT commanded; this program has no publisher, no service client and no Kortex session.

## The arms

**The pose was not measured.** replay of saved frames
## left_gripper

REPLAY of Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0

- intrinsics read from the published camera_info, not assumed: colour cx=620.9 cy=238.3 on a 1280x720 image -- the principal point is not the image centre and assuming it is throws every ray

Contact sheet: `sheet_left_gripper.png`

### 01  The frame, as the camera delivered it  [ok]

![raw_colour](stages/left_gripper/01_raw_colour.png)

- **formula** `none -- this is the input every later stage is a function of`
- **source** REPLAY of Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0
- **summary** 1280 x 720 px | focus (variance of Laplacian) 38 | 0.01% of pixels clipped white, 0.11% crushed black

| quantity | value | unit |
| --- | ---: | --- |
| frame width | 1280 | px |
| frame height | 720 | px |
| focus, variance of Laplacian | 38.2 | - |
| pixels clipped white | 0.013 | % |
| pixels crushed black | 0.111 | % |
| mean blue | 147.52 | 0-255 |
| mean green | 145.33 | 0-255 |
| mean red | 151.81 | 0-255 |
| median grey | 154 | 0-255 |
| grey std | 56.3 | 0-255 |

- **note** intrinsics read from the published camera_info, not assumed: colour cx=620.9 cy=238.3 on a 1280x720 image -- the principal point is not the image centre and assuming it is throws every ray

### 02  Intrinsics and the ray each pixel stands for  [ok]

![intrinsics](stages/left_gripper/02_intrinsics.png)

- **formula** `bearing = ((u - cx)/fx, (v - cy)/fy);  X = bearing * Z`
- **source** REPLAY of Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0
- **summary** fx 1297.67  fy 1298.63  cx 620.91  cy 238.28 | field of view 52.5 x 31.0 deg | principal point is 123.2 px from the image centre, which is 95 mm of sideways error at 1 m if you assume the centre

| quantity | value | unit |
| --- | ---: | --- |
| fx | 1298 | px |
| fy | 1299 | px |
| cx | 620.914 | px |
| cy | 238.2803 | px |
| image centre u | 640 | px |
| image centre v | 360 | px |
| principal point offset from centre | 123.21 | px |
| that offset at 1 m range | 94.9 | mm |
| horizontal field of view | 52.504 | deg |
| vertical field of view | 30.988 | deg |
| ground sample distance at 1 m | 0.7706 | mm/px |


### 03  BGR to HSV, the space the colour tests live in  [ok]

![hsv](stages/left_gripper/03_hsv.png)

- **formula** `V = max(B,G,R);  S = (V - min(B,G,R))/V;  H = the sector of the RGB hexagon, 0..179 in OpenCV (not 0..359)`
- **source** REPLAY of Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0
- **summary** median H 133  S 16  V 163 | 85.8% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 133 | 0-179 |
| median saturation | 16 | 0-255 |
| median value | 163 | 0-255 |
| pixels below S=80 | 85.812 | % |
| pixels below V=40 | 0.803 | % |
| pixels above V=200 | 25.277 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/left_gripper/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 31333 px selected (3.400% of the frame) | of those, median S 108 and median V 142

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 31333 | px |
| fraction of the frame | 3.3998 | % |
| median S of the selected | 108 | 0-255 |
| median V of the selected | 142 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/left_gripper/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 31333 px in, 30550 px out, 783 px removed (2.5%) | 26 separate blobs before, 2 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 31333 | px |
| pixels after | 30550 | px |
| pixels removed | 783 | px |
| removed | 2.499 | % |
| blobs before | 26 | count |
| blobs after | 2 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/left_gripper/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** REPLAY of Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0
- **summary** 2 components over 40 px | largest 30450 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 2 | count |
| largest area | 30450 | px |
| total area kept | 30550 | px |
| median area | 1.528e+04 | px |


*every component* -- 2 rows, full data in `data/left_gripper/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/left_gripper/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** white 234609 | orange 42731 | green 31055 | black 9772 | red 8181 | purple 431

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 7 | count |
| largest term | white | - |
| its pixel count | 234609 | px |


*pixels per colour term* -- 11 rows, full data in `data/left_gripper/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/left_gripper/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 30550 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 31055 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 31055 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

### 08  The region-MEAN ratio test, and its near-black guard  [ok]

![region_mean](stages/left_gripper/08_region_mean.png)

- **formula** `green  <=>  mean_G > 1.25 * mean_B  and  mean_G > 1.8 * mean_R,  refused outright when max(B,G,R) < 25`
- **source** srl_perception/prompt_detector.py:_colour_matches
- **summary** 0 of 2 regions pass

| quantity | value | unit |
| --- | ---: | --- |
| regions tested | 2 | count |
| regions passing | 0 | count |
| regions rejected as near-black | 0 | count |
| blue ratio threshold | 1.25 | - |
| red ratio threshold | 1.8 | - |
| near-black floor | 25 | 0-255 |


*region means and verdicts* -- 2 rows, full data in `data/left_gripper/08_region_mean__region_means_and_verdicts.csv`

- **note** Per-pixel hue picked the room's teal robots three times: their shadowed hue is 76 against the target cube's 74, two apart and inside noise. Region MEANS separate them, because teal carries far more blue. The near-black guard exists because a region measuring (4.1, 8.8, 3.9) -- visually black -- satisfies the ratios on sensor noise alone and was reported as the cube.

### 09  Rotated box, in-image yaw, and the degeneracy gate  [ok]

![minarearect](stages/left_gripper/09_minarearect.png)

- **formula** `minAreaRect -> ((cx,cy), (w,h), angle);  yaw is published as q = (0, 0, sin(angle/2), cos(angle/2)) ONLY when max(w,h)/min(w,h) >= 1.15`
- **source** srl_perception/colour_shape_detector.py
- **summary** 191 x 199 px, 77.6 deg, aspect 1.04, identity (not measured)

| quantity | value | unit |
| --- | ---: | --- |
| contours measured | 1 | count |
| aspect gate for publishing yaw | 1.15 | - |
| yaws published | 0 | count |
| yaws withheld as degenerate | 1 | count |


*rotated boxes* -- 1 rows, full data in `data/left_gripper/09_minarearect__rotated_boxes.csv`

- **note** Rotation about the OPTICAL AXIS is exactly what this measures; out-of-plane tilt is what it cannot see. The detector published identity for months while measuring the angle into a debug string, so an object at 30 deg got a square grasp and no check could tell that from a square object. A near-square blob still gets identity, because minAreaRect's angle is degenerate there and publishing it would be inventing a direction.

### 10  The size-at-range gate  [ok]

![size_at_range](stages/left_gripper/10_size_at_range.png)

- **formula** `z = fx * object_width / px_across;  accept iff 0.25 <= z <= 1.50 m.  With depth the honest form is used instead: compare px against fx * size / measured_depth, tolerance 0.55 to 1.80`
- **source** srl_perception/colour_shape_detector.py -- expected_px() and size_at_range_ok()
- **summary** 199 px -> 0.261 m accept

| quantity | value | unit |
| --- | ---: | --- |
| fx used | 1298 | px |
| assumed object width | 40 | mm |
| near limit of the window | 0.25 | m |
| far limit of the window | 1.5 | m |
| blobs tested | 1 | count |
| blobs accepted | 1 | count |
| blobs rejected | 0 | count |


*implied range per blob* -- 1 rows, full data in `data/left_gripper/10_size_at_range__implied_range_per_blob.csv`

- **note** This gate was added after a measured failure: T1's coloured pads are 210 x 130 mm in exactly the cube colours, and with a minimum-area floor and no upper bound the detector locked onto the pads and scored 0 of 4 cubes. A 210 mm pad read as a 40 mm cube implies z = 0.134 m -- 'a cube 134 mm from the lens' -- which is outside anything the arm works in, so the pad disappears while a real cube at 0.5 m passes untouched.

### 11  FastSAM: every region in the picture, no prompt  [ok]

![fastsam](stages/left_gripper/11_fastsam.png)

- **formula** `object-agnostic instance masks; no class, no prompt, no scene knowledge -- 'what are the regions', not 'where is the green cube'`
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 3.6 s for this frame
- **summary** 32 regions | largest 464645 px (50.4% of frame), smallest 283 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 32 | count |
| largest region | 464645 | px |
| largest as a fraction of the frame | 50.42 | % |
| smallest region | 283 | px |
| median region | 9224 | px |
| inference time | 3.57 | s |
| input size | 1024 | px |
| confidence threshold | 0.25 | - |
| NMS IoU | 0.7 | - |
| device | cpu | - |


*the twenty largest regions* -- 20 rows, full data in `data/left_gripper/11_fastsam__the_twenty_largest_regions.csv`

- **note** This is what replaced clustering. Two 40 mm cubes on a 60 mm pitch are 20 mm apart, which is the cluster distance itself, so depth-clustering fused them into one object 140 mm across the jaws and refused it. In the PICTURE they are not remotely ambiguous. Masks decide WHAT is an object; depth decides WHERE it is.

### 12  The area filter: noise below, the table above  [ok]

![area_filter](stages/left_gripper/12_area_filter.png)

- **formula** `keep iff  60 px <= area <= 0.35 * W * H  (= 322559 px here)`
- **source** srl_perception/segment_lift.py: MIN_AREA_PX, MAX_AREA_FRAC
- **summary** 32 in, 30 kept, 0 too small, 2 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 32 | count |
| kept | 30 | count |
| rejected as noise | 0 | count |
| rejected as too large | 2 | count |
| lower bound | 60 | px |
| upper bound | 322560 | px |
| upper bound as a fraction | 0.35 | - |

- **note** The upper bound is not tidiness. A segmenter returns the table and the room as legitimate regions, and a region covering a third of the frame is never a thing to be picked up.

### 13  Segment everything, then pick the one that matches  [refused]

![segment_match](stages/left_gripper/13_segment_match.png)

- **refused** FastSAM returned 32 regions and NONE of them is green-dominant by the region-mean test. That is a real 'not found' for this prompt in this frame -- the segmenter ran, the gate ran, and nothing matched.

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](stages/left_gripper/14_yoloworld.png)

- **refused** YOLO-World ran in 4.4 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

### 15  The depth frame, and where it has no answer  [ok]

![raw_depth](stages/left_gripper/15_raw_depth.png)

- **formula** `one 16-bit value per pixel, in MILLIMETRES on the wire, divided by 1000 here. 0 means 'no return' and is NOT a distance of zero.`
- **source** REPLAY of Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0
- **summary** 480 x 270 | 89.4% of pixels have a return | median 2.222 m, 5th-95th 0.426-5.474 m | 54789 px between the 0.08 and 1.50 m gate the pick path uses

| quantity | value | unit |
| --- | ---: | --- |
| depth width | 480 | px |
| depth height | 270 | px |
| pixels with a return | 115923 | px |
| that fraction | 89.45 | % |
| median range | 2.222 | m |
| 5th percentile | 0.426 | m |
| 95th percentile | 5.474 | m |
| nearest return | 0.362 | m |
| furthest return | 10.565 | m |
| returns inside this camera's working band | 54789 | px |
| band near | 0.08 | m |
| band far | 1.5 | m |

- **note** A hole is not a far surface. Averaging a hole in as 0 drags every edge pixel toward the camera, which is why the RealSense average is taken over VALID pixels only.

### 16  Depth put into the colour frame  [ok]

![alignment](stages/left_gripper/16_alignment.png)

- **formula** `P = ((u-cx_d)z/fx_d, (v-cy_d)z/fy_d, z);  P' = P + t;  u' = fx_c X'/Z' + cx_c,  v' = fy_c Y'/Z' + cy_c`
- **source** forward-projected here: deproject with the depth intrinsics, translate by the recorded -27.1, -10.0, -4.7 mm baseline, project with the colour intrinsics, nearest point wins
- **summary** 66603 points projected, 48825 fell outside the colour frame, 64.4% of colour pixels got a depth

| quantity | value | unit |
| --- | ---: | --- |
| points projected | 66603 | count |
| points falling outside the colour frame | 48825 | count |
| colour pixels given a depth | 64.43 | % |
| splat radius | 1 | px |
| baseline x | -27.06 | mm |
| baseline y | -9.97 | mm |
| baseline z | -4.71 | mm |
| rotation assumed | identity | - |

- **note** The rotation between the two sensors is taken as IDENTITY here, because a recorded translation is all this program has. The live pick path uses the URDF FK between camera_depth_frame and camera_color_frame instead -- and that extrinsic is known to be ~8.5 deg wrong and to rotate with the wrist, which is why the pick servos in the camera frame rather than planning through it.

### 17  Pixels and depth become a cloud  [ok]

![deprojection](stages/left_gripper/17_deprojection.png)

- **formula** `X = (u - cx) Z / fx,   Y = (v - cy) Z / fy,   Z = depth`
- **source** rgbd_grasp.deproject / servo_pick_left.measure, with fx 360.01 fy 360.01 cx 243.87 cy 137.92
- **summary** 54789 points in the 0.08-1.50 m band | extent X -0.417 to 0.691, Y -0.067 to 0.389, Z 0.362 to 1.085 m

| quantity | value | unit |
| --- | ---: | --- |
| points lifted | 54789 | count |
| fx | 360.013 | px |
| fy | 360.013 | px |
| cx | 243.872 | px |
| cy | 137.922 | px |
| X minimum | -0.4166 | m |
| X maximum | 0.6905 | m |
| Y minimum | -0.0666 | m |
| Y maximum | 0.3889 | m |
| Z minimum | 0.362 | m |
| Z maximum | 1.085 | m |
| centroid X | 0.0155 | m |
| centroid Y | 0.0825 | m |
| centroid Z | 0.5687 | m |

- **note** This is the whole of 'depth decides WHERE'. Everything after it is geometry on these points, in the CAMERA's frame -- no robot transform is involved yet, which is exactly why the pick measures its error here.

### 18  RANSAC: the support plane  [ok]

![ransac](stages/left_gripper/18_ransac.png)

- **formula** `draw 3 points -> n = (b-a) x (c-a) / |...|,  d = -n.a;  score = #{ |n.P + d| < 6 mm };  keep the best of 400 draws`
- **source** scripts/servo_pick_left.py:fit_plane, imported; fitted on 54789 of 54789 points
- **summary** normal (-0.2488, -0.8831, -0.3977), offset 0.2950 m | 40517 of 54789 points are inliers (74.0%) | RMS 1.43 mm | 66.6 deg between the plane normal and the camera's optical axis

| quantity | value | unit |
| --- | ---: | --- |
| normal x | -0.2488 | - |
| normal y | -0.8831 | - |
| normal z | -0.3977 | - |
| offset d | 0.295 | m |
| points fitted | 54789 | count |
| points scored | 54789 | count |
| inliers | 40517 | count |
| inlier fraction | 73.95 | % |
| residual RMS | 1.428 | mm |
| inlier tolerance | 6 | mm |
| RANSAC iterations | 400 | count |
| angle to the optical axis | 66.56 | deg |

- **note** A plane with too few inliers is not a plane. The pick refuses below 200 inliers rather than returning a geometry whose RMS is NaN -- everything downstream would treat that as a fix and the arm would move on it.

### 19  The PCA refinement, and the NaN it once produced  [ok]

![pca_refine](stages/left_gripper/19_pca_refine.png)

- **formula** `C = cov(Q - mean(Q)) for inliers Q;  n = eigvec(C)[:, argmin];  d = -n . mean(Q)   <-- BOTH halves, together`
- **source** scripts/servo_pick_left.py:fit_plane, imported
- **summary** the normal moved 0.000 deg | RMS 1.43 mm -> 1.43 mm | mixing the refined normal with the RANSAC offset selects 40517 inliers instead of 40517

| quantity | value | unit |
| --- | ---: | --- |
| normal moved | 0 | deg |
| RMS before refinement | 1.428 | mm |
| RMS after refinement | 1.428 | mm |
| inliers with the matched pair | 40517 | count |
| inliers with a mixed normal and offset | 40517 | count |
| inliers lost by mixing | 0 | count |

- **note** That last number is the bug this stage exists to show. The function once returned the REFINED normal with the OLD offset; those describe different planes, the inlier mask could select ZERO points, and mean() of an empty slice is NaN. The pick printed 'plane RMS nan mm' and 'FOUND' in the same breath, then closed 540 mm blind on a centre computed against that plane.

### 20  Height above the plane  [ok]

![height_map](stages/left_gripper/20_height_map.png)

- **formula** `h(P) = n . P + d,  signed;  positive is off the surface, toward the camera`
- **source** servo_pick_left.measure -- the same expression the live pick uses to separate the cube from the table
- **summary** 9568 of 54789 points stand more than 5 mm off the plane (17.46%) | tallest 91.0 mm

| quantity | value | unit |
| --- | ---: | --- |
| height threshold | 5 | mm |
| points above it | 9568 | count |
| that fraction | 17.463 | % |
| tallest point | 91.05 | mm |
| lowest point | -644.91 | mm |
| median height | 0.134 | mm |

- **note** The floor is 5 mm because the plane RMS is under 1 mm on this sensor. Set it below the depth noise and the table itself becomes an object; set it far above and a flat item is missed.

### 21  The surface height as a MODE, not a mean  [ok]

![mode_surface](stages/left_gripper/21_mode_surface.png)

- **formula** `bin at 2 mm; take the fullest bin; require it to hold >= 20% of the points; refine as the mean of everything within +/-6 mm of it`
- **source** srl_perception/surface_from_depth.py, applied along the fitted plane normal because this program has no camera-to-world transform; the node applies it to world z
- **summary** modal bin holds 45.4% of 54789 points (floor 20%) | refined -0.2950 m, spread 1.43 mm | a plain MEAN would say -0.3029 m, which is 7.9 mm out | RANSAC's own offset is -0.2950 m, 0.02 mm from this

| quantity | value | unit |
| --- | ---: | --- |
| bin width | 2 | mm |
| points binned | 54789 | count |
| share in the fullest bin | 45.35 | % |
| share required | 20 | % |
| modal bin centre | -0.2949 | m |
| refined estimate | -0.295 | m |
| spread of the inliers | 1.428 | mm |
| inlier band | 6 | mm |
| a plain mean would say | -0.3029 | m |
| that mean's error | 7.9 | mm |
| RANSAC's own offset | -0.295 | m |
| the two estimators differ by | 0.018 | mm |

- **note** Two independent estimators of one surface, so they can be compared: RANSAC votes on inliers, this votes on the fullest histogram bin. A mean is dragged up by everything standing on the table and down by every hole, and it MOVES as the objects move -- the 'measurement' would change between trials on a motionless table.

### 22  What is standing on the surface  [ok]

![on_surface](stages/left_gripper/22_on_surface.png)

- **formula** `{ pixels whose lifted point has h > 5 mm }, 8-connected`
- **source** the plane of stage 18 fitted in the colour frame; heights from the aligned depth of stage 16
- **summary** 90395 px, 61806 pts, top 71 mm | 31799 px, 21830 pts, top 63 mm | 576 px, 432 pts, top 10 mm | 194 px, 170 pts, top 7 mm

| quantity | value | unit |
| --- | ---: | --- |
| pieces standing on the surface | 4 | count |
| height threshold | 5 | mm |
| largest piece | 90395 | px |
| tallest piece | 70.6 | mm |


*pieces above the plane* -- 4 rows, full data in `data/left_gripper/22_on_surface__pieces_above_the_plane.csv`

- **note** This is the pre-2020 method on its own -- threshold above a plane, then cluster what is left. It is exactly what fails when two 40 mm cubes sit 20 mm apart: connectivity fuses them into one object 140 mm across. Compare this figure with stage 11.

### 23  The measurement the pick actually acts on  [ok]

![cube_measurement](stages/left_gripper/23_cube_measurement.png)

- **formula** `cube = colour(u,v) AND h > 5 mm;  foot = mean(Pk) - n (n.mean(Pk) + d);  centre = foot + n * max(h)/2`
- **source** scripts/servo_pick_left.py:measure -- this is the measurement the servo loop nulls against, in the CAMERA's frame, where the data is good
- **summary** 1049 object points | top 57.3 mm above the plane | centre (0.0707, 0.0482, 0.5185) m in the camera frame | plane RMS 1.43 mm, inlier fraction 0.74

| quantity | value | unit |
| --- | ---: | --- |
| object points | 1049 | count |
| points that are the colour but ON the plane | 1215 | count |
| points off the plane but not the colour | 8519 | count |
| top above the plane | 57.26 | mm |
| median height | 32.15 | mm |
| centre x | 0.0707 | m |
| centre y | 0.0482 | m |
| centre z | 0.5185 | m |
| range to the centre | 0.5255 | m |
| plane RMS | 1.428 | mm |
| plane inlier fraction | 0.7395 | - |
| minimum points the pick requires | 60 | count |

- **note** Two independent gates, and that is the point: colour alone picks up the green pad the cube stands on and anything green in the room; height alone picks up everything on the table. The centre uses the foot point rather than the cloud's centroid because a depth camera sees a SHELL -- the front surface only -- and a shell's centroid is biased toward the camera, measured at 10.5 mm in the error budget against a 30 mm capture gate.

### 24  Splitting a cube from the pad it stands on  [ok]

![layers](stages/left_gripper/24_layers.png)

- **formula** `find the MODAL height bin (5 mm bins); cut 12 mm above it; recurse on what is left, twice`
- **source** srl_perception/segment_lift.py:_layers
- **summary** layer 1: 4-16 mm, 12667 points | layer 2: 16-64 mm, 38757 points

| quantity | value | unit |
| --- | ---: | --- |
| region area | 104550 | px |
| points lifted from it | 66260 | count |
| points above the plane | 51424 | count |
| pixels removed by the 1 px erosion | 1770 | px |
| layers found | 2 | count |
| lowest point | 4 | mm |
| highest point | 63.87 | mm |


*height layers* -- 2 rows, full data in `data/left_gripper/24_layers__height_layers.csv`

- **note** Appearance decides what is SIDE BY SIDE; height decides what is STACKED. A blue cube on a blue pad is correctly ONE region -- same colour, touching, no edge between them. And the split cannot look for an empty band, because there is none: the cube RESTS on the pad, so its sides fill every bin. What is there is a MODE -- the support's top face is a large flat area and dominates the histogram.

### 25  Rotating calipers, against the two controls  [ok]

![min_width](stages/left_gripper/25_min_width.png)

- **formula** `width = min over theta of [ max(p.u(theta)) - min(p.u(theta)) ],  u(theta) = (cos, sin) in the support plane, swept at 0.25 deg`
- **source** srl_perception/segment_lift.py:_min_width and rgbd_grasp.min_width_frame
- **summary** calipers 277.7 mm | PCA's short axis 293.8 mm | axis-aligned box 277.7 mm | long axis 341.0 mm, height 59.9 mm | jaws close on 85 mm, so this is TOO WIDE

| quantity | value | unit |
| --- | ---: | --- |
| instance points | 51424 | count |
| minimum width, rotating calipers | 277.69 | mm |
| at this angle in the plane | 0 | deg |
| PCA short axis, THE CONTROL | 293.83 | mm |
| axis-aligned box, THE CONTROL | 277.69 | mm |
| PCA overstates by | 16.14 | mm |
| long axis | 341.04 | mm |
| height | 59.87 | mm |
| jaw span | 85 | mm |
| graspable as it lies | no | - |
| sweep step | 0.5 | deg |

- **note** The two controls are here because both were used and both were wrong. The principal axes of a SQUARE are degenerate -- equal variance in both directions -- so SVD returns the diagonal and a 40 mm cube measures 40*sqrt(2) = 53 mm; against an 85 mm jaw that refuses a 60 mm object as ungraspable. The error that matters is not the 13 mm, it is the DIRECTION: a jaw told to close across a diagonal approaches the corner and the object rolls out.

### 26  Dropping masks that are unions of finer ones  [ok]

![nested](stages/left_gripper/26_nested.png)

- **formula** `voxelise each instance at 5 mm; accept smallest first; reject one whose claimed-voxel overlap exceeds 0.55`
- **source** srl_perception/segment_lift.py:_suppress_nested
- **summary** 5 instances in, 5 kept, 0 dropped | overlaps: none

| quantity | value | unit |
| --- | ---: | --- |
| instances in | 5 | count |
| kept | 5 | count |
| dropped as already explained | 0 | count |
| voxel size | 5 | mm |
| overlap fraction that rejects | 0.55 | - |


*per instance* -- 5 rows, full data in `data/left_gripper/26_nested__per_instance.csv`

- **note** FastSAM returns a HIERARCHY, not a partition: for one picture it hands back the cube, the pad, and a region covering both. All three are legitimate. Lifted, they became three objects in the same space and the merge absorbed a 40 mm cube -- measured EXACT at 40.3 mm against a 40 mm truth -- into a 148 mm blob that was then refused as ungraspable. Volume, not pixels: two masks that overlap in the image can be a metre apart in depth.

### 27  The parallel-jaw grasp  [refused]

![grasp](stages/left_gripper/27_grasp.png)

- **refused** the grasp planner refused, which is an ANSWER: object is 144 mm across its narrowest axis; the Robotiq 85 opens 75 mm usable. It cannot be grasped as it lies.

### 28  Fiducials: the primary detector  [refused]

![apriltag](stages/left_gripper/28_apriltag.png)

- **refused** no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That is an answer, not a failure: a tag either decodes or it does not, which is exactly why this is the PRIMARY detector and colour is the capped fallback. Detector used: cv2.aruco.ArucoDetector (OpenCV 4.11.0).

### 29  The support surface, as an object  [ok]

![table](stages/left_gripper/29_table.png)

- **formula** `fit the plane, keep its inliers, span them in the plane's own basis; extent taken at the 2nd and 98th percentile so one stray return cannot stretch it`
- **source** the plane of stage 18, expressed as a rectangle in the camera frame
- **summary** 0.579 x 0.423 m of surface, 0.278 m away, flat to 1.42 mm

| quantity | value | unit |
| --- | ---: | --- |
| points on the surface | 339810 | count |
| surface extent along its first axis | 0.5785 | m |
| surface extent along its second axis | 0.4232 | m |
| area spanned | 0.2449 | m2 |
| perpendicular distance from the camera | 0.2775 | m |
| distance to the middle of that extent | 0.5916 | m |
| nearest point on the surface | 0.3978 | m |
| furthest point on the surface | 0.9276 | m |
| tilt from the optical axis | 66.54 | deg |
| flatness, RMS of the inliers | 1.423 | mm |

- **note** This is a MEASURED surface, not the declared one. srl_experiments/work_surface.py has had set_measured() since it was written and nothing ever called it, so a table 20 mm out would have been absorbed silently into every grasp.

### 30  Every object, as an oriented 3-D box  [ok]

![boxes3d](stages/left_gripper/30_boxes3d.png)

- **formula** `centre = mid-extent ACROSS the surface + (foot + top/2) ALONG the normal; axes = (minimum-width direction, its perpendicular, the plane normal); size = the spans along them`
- **source** segment_lift._measure for the horizontal centre and rgbd_grasp.shell_corrected_centre's reasoning for the vertical -- the combination is checked to 2 mm against a constructed cube in --self-test
- **summary** 5 objects, 0.539 to 0.843 m away

| quantity | value | unit |
| --- | ---: | --- |
| objects posed | 5 | count |
| nearest object | 0.5389 | m |
| furthest object | 0.8427 | m |
| largest footprint | 341 | mm |
| tallest object | 63.9 | mm |
| graspable within the 85 mm jaw | 4 | count |


*every object, in the camera frame* -- 5 rows, full data in `data/left_gripper/30_boxes3d__every_object__in_the_camera_frame.csv`

- **note** These coordinates are in the CAMERA's frame. Putting them in the robot's frame needs the camera extrinsic, which on the wrist is measured at ~8.5 deg wrong and rotates with the joint -- 34 mm of error at 0.23 m. That is why the pick servos on an error measured HERE rather than planning through that transform.

### 31  How far apart everything is  [ok]

![distances](stages/left_gripper/31_distances.png)

- **formula** `range = |centre| (the camera is the origin); height = n.c + d; separation = |c_i - c_j|; free gap subtracts each object's half-width`
- **source** the poses of stage 30, all in the camera's own frame
- **summary** 5 objects, nearest 0.539 m, closest pair 0.093 m

| quantity | value | unit |
| --- | ---: | --- |
| objects measured | 5 | count |
| nearest object | 0.5389 | m |
| furthest object | 0.8427 | m |
| closest pair | 0.0929 | m |
| tightest free gap | 0.0087 | m |
| what the origin IS | the wrist camera, so range is distance from the HAND | - |


*each object* -- 5 rows, full data in `data/left_gripper/31_distances__each_object.csv`


*between objects* -- 10 rows, full data in `data/left_gripper/31_distances__between_objects.csv`

- **note** On a WRIST camera the origin is the hand, so 'range' is already the distance the arm has to close -- that is the quantity the servo loop nulls. On the scene camera it is the distance from the camera, and turning that into a distance from the ARM needs the scene-camera-to-robot calibration, which this repository does not have measured.

### 32  People in the picture  [refused]

![people](stages/left_gripper/32_people.png)

- **refused** no person is in this frame. MediaPipe ran in 2.8 s and found nobody, which is an answer -- the wearer-tracking safety case only ever makes the modelled body BIGGER, so 'nobody seen' falls back to the mannequin rather than to an empty scene.

### 33  The robot in the picture, and what is NOT detected  [ok]

![self_view](stages/left_gripper/33_self_view.png)

- **formula** `{ pixels with 0 < depth < 0.08 m } -- nearer than anything this camera works on`
- **source** REPLAY of Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0
- **summary** 0 px nearer than 0.08 m

| quantity | value | unit |
| --- | ---: | --- |
| pixels nearer than the gate | 0 | px |
| that fraction | 0 | % |
| nearest return of all | 0.362 | m |
| blobs of near return | 0 | count |
| this camera is on the hand | yes | - |
| near edge of this camera's band | 0.08 | m |


*near-field blobs* -- 0 rows, full data in `data/left_gripper/33_self_view__near_field_blobs.csv`

- **note** THE ROBOT ITSELF IS NOT DETECTED IN THE IMAGE, and no stage in this program claims to. Nothing in this repository finds an arm in a picture: the robot's pose comes from its own encoders through the URDF, and drawing it into a camera image needs the camera-to-robot extrinsic -- measured at ~8.5 deg wrong on the wrist, and never measured at all for the scene cameras. On a wrist camera the near-field blobs above are usually the gripper's own fingers entering the bottom of the frame, which is also why the last ~80 mm of a grasp is completed from the last good fix rather than from live vision.

## right_gripper

REPLAY of Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0

- intrinsics read from the published camera_info, not assumed: colour cx=620.9 cy=238.3 on a 1280x720 image -- the principal point is not the image centre and assuming it is throws every ray

Contact sheet: `sheet_right_gripper.png`

### 01  The frame, as the camera delivered it  [ok]

![raw_colour](stages/right_gripper/01_raw_colour.png)

- **formula** `none -- this is the input every later stage is a function of`
- **source** REPLAY of Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0
- **summary** 1280 x 720 px | focus (variance of Laplacian) 98 | 4.04% of pixels clipped white, 0.14% crushed black

| quantity | value | unit |
| --- | ---: | --- |
| frame width | 1280 | px |
| frame height | 720 | px |
| focus, variance of Laplacian | 97.9 | - |
| pixels clipped white | 4.043 | % |
| pixels crushed black | 0.143 | % |
| mean blue | 133.23 | 0-255 |
| mean green | 130.95 | 0-255 |
| mean red | 133.22 | 0-255 |
| median grey | 123 | 0-255 |
| grey std | 70.11 | 0-255 |

- **note** intrinsics read from the published camera_info, not assumed: colour cx=620.9 cy=238.3 on a 1280x720 image -- the principal point is not the image centre and assuming it is throws every ray

### 02  Intrinsics and the ray each pixel stands for  [ok]

![intrinsics](stages/right_gripper/02_intrinsics.png)

- **formula** `bearing = ((u - cx)/fx, (v - cy)/fy);  X = bearing * Z`
- **source** REPLAY of Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0
- **summary** fx 1297.67  fy 1298.63  cx 620.91  cy 238.28 | field of view 52.5 x 31.0 deg | principal point is 123.2 px from the image centre, which is 95 mm of sideways error at 1 m if you assume the centre

| quantity | value | unit |
| --- | ---: | --- |
| fx | 1298 | px |
| fy | 1299 | px |
| cx | 620.914 | px |
| cy | 238.2803 | px |
| image centre u | 640 | px |
| image centre v | 360 | px |
| principal point offset from centre | 123.21 | px |
| that offset at 1 m range | 94.9 | mm |
| horizontal field of view | 52.504 | deg |
| vertical field of view | 30.988 | deg |
| ground sample distance at 1 m | 0.7706 | mm/px |


### 03  BGR to HSV, the space the colour tests live in  [ok]

![hsv](stages/right_gripper/03_hsv.png)

- **formula** `V = max(B,G,R);  S = (V - min(B,G,R))/V;  H = the sector of the RGB hexagon, 0..179 in OpenCV (not 0..359)`
- **source** REPLAY of Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0
- **summary** median H 117  S 30  V 139 | 70.7% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 117 | 0-179 |
| median saturation | 30 | 0-255 |
| median value | 139 | 0-255 |
| pixels below S=80 | 70.72 | % |
| pixels below V=40 | 6.992 | % |
| pixels above V=200 | 27.115 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/right_gripper/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 12357 px selected (1.341% of the frame) | of those, median S 139 and median V 116

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 12357 | px |
| fraction of the frame | 1.3408 | % |
| median S of the selected | 139 | 0-255 |
| median V of the selected | 116 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/right_gripper/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 12357 px in, 12206 px out, 151 px removed (1.2%) | 8 separate blobs before, 1 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 12357 | px |
| pixels after | 12206 | px |
| pixels removed | 151 | px |
| removed | 1.222 | % |
| blobs before | 8 | count |
| blobs after | 1 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/right_gripper/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** REPLAY of Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0
- **summary** 1 components over 40 px | largest 12206 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 1 | count |
| largest area | 12206 | px |
| total area kept | 12206 | px |
| median area | 1.221e+04 | px |


*every component* -- 1 rows, full data in `data/right_gripper/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/right_gripper/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** white 248739 | black 74880 | orange 49207 | blue 38351 | cyan 30069 | red 25576

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 8 | count |
| largest term | white | - |
| its pixel count | 248739 | px |


*pixels per colour term* -- 11 rows, full data in `data/right_gripper/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/right_gripper/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 12206 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 12308 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 12281 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

### 08  The region-MEAN ratio test, and its near-black guard  [ok]

![region_mean](stages/right_gripper/08_region_mean.png)

- **formula** `green  <=>  mean_G > 1.25 * mean_B  and  mean_G > 1.8 * mean_R,  refused outright when max(B,G,R) < 25`
- **source** srl_perception/prompt_detector.py:_colour_matches
- **summary** 1 of 1 regions pass

| quantity | value | unit |
| --- | ---: | --- |
| regions tested | 1 | count |
| regions passing | 1 | count |
| regions rejected as near-black | 0 | count |
| blue ratio threshold | 1.25 | - |
| red ratio threshold | 1.8 | - |
| near-black floor | 25 | 0-255 |


*region means and verdicts* -- 1 rows, full data in `data/right_gripper/08_region_mean__region_means_and_verdicts.csv`

- **note** Per-pixel hue picked the room's teal robots three times: their shadowed hue is 76 against the target cube's 74, two apart and inside noise. Region MEANS separate them, because teal carries far more blue. The near-black guard exists because a region measuring (4.1, 8.8, 3.9) -- visually black -- satisfies the ratios on sensor noise alone and was reported as the cube.

### 09  Rotated box, in-image yaw, and the degeneracy gate  [ok]

![minarearect](stages/right_gripper/09_minarearect.png)

- **formula** `minAreaRect -> ((cx,cy), (w,h), angle);  yaw is published as q = (0, 0, sin(angle/2), cos(angle/2)) ONLY when max(w,h)/min(w,h) >= 1.15`
- **source** srl_perception/colour_shape_detector.py
- **summary** 125 x 114 px, 11.0 deg, aspect 1.09, identity (not measured)

| quantity | value | unit |
| --- | ---: | --- |
| contours measured | 1 | count |
| aspect gate for publishing yaw | 1.15 | - |
| yaws published | 0 | count |
| yaws withheld as degenerate | 1 | count |


*rotated boxes* -- 1 rows, full data in `data/right_gripper/09_minarearect__rotated_boxes.csv`

- **note** Rotation about the OPTICAL AXIS is exactly what this measures; out-of-plane tilt is what it cannot see. The detector published identity for months while measuring the angle into a debug string, so an object at 30 deg got a square grasp and no check could tell that from a square object. A near-square blob still gets identity, because minAreaRect's angle is degenerate there and publishing it would be inventing a direction.

### 10  The size-at-range gate  [ok]

![size_at_range](stages/right_gripper/10_size_at_range.png)

- **formula** `z = fx * object_width / px_across;  accept iff 0.25 <= z <= 1.50 m.  With depth the honest form is used instead: compare px against fx * size / measured_depth, tolerance 0.55 to 1.80`
- **source** srl_perception/colour_shape_detector.py -- expected_px() and size_at_range_ok()
- **summary** 125 px -> 0.417 m accept

| quantity | value | unit |
| --- | ---: | --- |
| fx used | 1298 | px |
| assumed object width | 40 | mm |
| near limit of the window | 0.25 | m |
| far limit of the window | 1.5 | m |
| blobs tested | 1 | count |
| blobs accepted | 1 | count |
| blobs rejected | 0 | count |


*implied range per blob* -- 1 rows, full data in `data/right_gripper/10_size_at_range__implied_range_per_blob.csv`

- **note** This gate was added after a measured failure: T1's coloured pads are 210 x 130 mm in exactly the cube colours, and with a minimum-area floor and no upper bound the detector locked onto the pads and scored 0 of 4 cubes. A 210 mm pad read as a 40 mm cube implies z = 0.134 m -- 'a cube 134 mm from the lens' -- which is outside anything the arm works in, so the pad disappears while a real cube at 0.5 m passes untouched.

### 11  FastSAM: every region in the picture, no prompt  [ok]

![fastsam](stages/right_gripper/11_fastsam.png)

- **formula** `object-agnostic instance masks; no class, no prompt, no scene knowledge -- 'what are the regions', not 'where is the green cube'`
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 0.8 s for this frame
- **summary** 39 regions | largest 265965 px (28.9% of frame), smallest 308 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 39 | count |
| largest region | 265965 | px |
| largest as a fraction of the frame | 28.86 | % |
| smallest region | 308 | px |
| median region | 8370 | px |
| inference time | 0.75 | s |
| input size | 1024 | px |
| confidence threshold | 0.25 | - |
| NMS IoU | 0.7 | - |
| device | cpu | - |


*the twenty largest regions* -- 20 rows, full data in `data/right_gripper/11_fastsam__the_twenty_largest_regions.csv`

- **note** This is what replaced clustering. Two 40 mm cubes on a 60 mm pitch are 20 mm apart, which is the cluster distance itself, so depth-clustering fused them into one object 140 mm across the jaws and refused it. In the PICTURE they are not remotely ambiguous. Masks decide WHAT is an object; depth decides WHERE it is.

### 12  The area filter: noise below, the table above  [ok]

![area_filter](stages/right_gripper/12_area_filter.png)

- **formula** `keep iff  60 px <= area <= 0.35 * W * H  (= 322559 px here)`
- **source** srl_perception/segment_lift.py: MIN_AREA_PX, MAX_AREA_FRAC
- **summary** 39 in, 39 kept, 0 too small, 0 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 39 | count |
| kept | 39 | count |
| rejected as noise | 0 | count |
| rejected as too large | 0 | count |
| lower bound | 60 | px |
| upper bound | 322560 | px |
| upper bound as a fraction | 0.35 | - |

- **note** The upper bound is not tidiness. A segmenter returns the table and the room as legitimate regions, and a region covering a third of the frame is never a thing to be picked up.

### 13  Segment everything, then pick the one that matches  [ok]

![segment_match](stages/right_gripper/13_segment_match.png)

- **formula** `for each region: mean BGR over the MASK, then the ratio test of stage 08; score = min(1, area/2000)`
- **source** srl_perception/prompt_detector.py:_segment -- the backend that actually found the target on this rig
- **summary** 13034 px at (703, 343), mean BGR [94.5, 137.5, 68.6]

| quantity | value | unit |
| --- | ---: | --- |
| colour word parsed | green | - |
| regions considered | 39 | count |
| regions matched | 1 | count |
| regions rejected on colour | 38 | count |
| best score | 1 | 0-1 |


*matched regions* -- 1 rows, full data in `data/right_gripper/13_segment_match__matched_regions.csv`

- **note** Naming the cube failed at every confidence -- it is ~19 px and dark, and an open-vocabulary detector cannot recognise it as anything. Segmenting it is trivial: 89 regions, exactly one green-dominant, no false positives. The MASK is the other win: a colour threshold gives whatever pixels passed, a segment gives the object's actual extent, which is what a width measurement needs.

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](stages/right_gripper/14_yoloworld.png)

- **refused** YOLO-World ran in 3.7 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

### 15  The depth frame, and where it has no answer  [ok]

![raw_depth](stages/right_gripper/15_raw_depth.png)

- **formula** `one 16-bit value per pixel, in MILLIMETRES on the wire, divided by 1000 here. 0 means 'no return' and is NOT a distance of zero.`
- **source** REPLAY of Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0
- **summary** 480 x 270 | 80.5% of pixels have a return | median 1.838 m, 5th-95th 0.339-5.105 m | 42779 px between the 0.08 and 1.50 m gate the pick path uses

| quantity | value | unit |
| --- | ---: | --- |
| depth width | 480 | px |
| depth height | 270 | px |
| pixels with a return | 104280 | px |
| that fraction | 80.46 | % |
| median range | 1.838 | m |
| 5th percentile | 0.339 | m |
| 95th percentile | 5.105 | m |
| nearest return | 0.299 | m |
| furthest return | 6.636 | m |
| returns inside this camera's working band | 42779 | px |
| band near | 0.08 | m |
| band far | 1.5 | m |

- **note** A hole is not a far surface. Averaging a hole in as 0 drags every edge pixel toward the camera, which is why the RealSense average is taken over VALID pixels only.

### 16  Depth put into the colour frame  [ok]

![alignment](stages/right_gripper/16_alignment.png)

- **formula** `P = ((u-cx_d)z/fx_d, (v-cy_d)z/fy_d, z);  P' = P + t;  u' = fx_c X'/Z' + cx_c,  v' = fy_c Y'/Z' + cy_c`
- **source** forward-projected here: deproject with the depth intrinsics, translate by the recorded -27.1, -10.0, -4.7 mm baseline, project with the colour intrinsics, nearest point wins
- **summary** 60377 points projected, 43864 fell outside the colour frame, 58.8% of colour pixels got a depth

| quantity | value | unit |
| --- | ---: | --- |
| points projected | 60377 | count |
| points falling outside the colour frame | 43864 | count |
| colour pixels given a depth | 58.79 | % |
| splat radius | 1 | px |
| baseline x | -27.06 | mm |
| baseline y | -9.97 | mm |
| baseline z | -4.71 | mm |
| rotation assumed | identity | - |

- **note** The rotation between the two sensors is taken as IDENTITY here, because a recorded translation is all this program has. The live pick path uses the URDF FK between camera_depth_frame and camera_color_frame instead -- and that extrinsic is known to be ~8.5 deg wrong and to rotate with the wrist, which is why the pick servos in the camera frame rather than planning through it.

### 17  Pixels and depth become a cloud  [ok]

![deprojection](stages/right_gripper/17_deprojection.png)

- **formula** `X = (u - cx) Z / fx,   Y = (v - cy) Z / fy,   Z = depth`
- **source** rgbd_grasp.deproject / servo_pick_left.measure, with fx 360.01 fy 360.01 cx 243.87 cy 137.92
- **summary** 42779 points in the 0.08-1.50 m band | extent X -0.284 to 0.442, Y -0.182 to 0.180, Z 0.299 to 0.927 m

| quantity | value | unit |
| --- | ---: | --- |
| points lifted | 42779 | count |
| fx | 360.013 | px |
| fy | 360.013 | px |
| cx | 243.872 | px |
| cy | 137.922 | px |
| X minimum | -0.2839 | m |
| X maximum | 0.4424 | m |
| Y minimum | -0.1816 | m |
| Y maximum | 0.1799 | m |
| Z minimum | 0.299 | m |
| Z maximum | 0.927 | m |
| centroid X | 0.0297 | m |
| centroid Y | 0.0903 | m |
| centroid Z | 0.5407 | m |

- **note** This is the whole of 'depth decides WHERE'. Everything after it is geometry on these points, in the CAMERA's frame -- no robot transform is involved yet, which is exactly why the pick measures its error here.

### 18  RANSAC: the support plane  [ok]

![ransac](stages/right_gripper/18_ransac.png)

- **formula** `draw 3 points -> n = (b-a) x (c-a) / |...|,  d = -n.a;  score = #{ |n.P + d| < 6 mm };  keep the best of 400 draws`
- **source** scripts/servo_pick_left.py:fit_plane, imported; fitted on 42779 of 42779 points
- **summary** normal (0.1263, -0.9409, -0.3143), offset 0.2981 m | 22584 of 42779 points are inliers (52.8%) | RMS 1.59 mm | 71.7 deg between the plane normal and the camera's optical axis

| quantity | value | unit |
| --- | ---: | --- |
| normal x | 0.1263 | - |
| normal y | -0.9409 | - |
| normal z | -0.3143 | - |
| offset d | 0.2981 | m |
| points fitted | 42779 | count |
| points scored | 42779 | count |
| inliers | 22584 | count |
| inlier fraction | 52.79 | % |
| residual RMS | 1.593 | mm |
| inlier tolerance | 6 | mm |
| RANSAC iterations | 400 | count |
| angle to the optical axis | 71.68 | deg |

- **note** A plane with too few inliers is not a plane. The pick refuses below 200 inliers rather than returning a geometry whose RMS is NaN -- everything downstream would treat that as a fix and the arm would move on it.

### 19  The PCA refinement, and the NaN it once produced  [ok]

![pca_refine](stages/right_gripper/19_pca_refine.png)

- **formula** `C = cov(Q - mean(Q)) for inliers Q;  n = eigvec(C)[:, argmin];  d = -n . mean(Q)   <-- BOTH halves, together`
- **source** scripts/servo_pick_left.py:fit_plane, imported
- **summary** the normal moved 0.000 deg | RMS 1.59 mm -> 1.59 mm | mixing the refined normal with the RANSAC offset selects 22584 inliers instead of 22584

| quantity | value | unit |
| --- | ---: | --- |
| normal moved | 0 | deg |
| RMS before refinement | 1.593 | mm |
| RMS after refinement | 1.593 | mm |
| inliers with the matched pair | 22584 | count |
| inliers with a mixed normal and offset | 22584 | count |
| inliers lost by mixing | 0 | count |

- **note** That last number is the bug this stage exists to show. The function once returned the REFINED normal with the OLD offset; those describe different planes, the inlier mask could select ZERO points, and mean() of an empty slice is NaN. The pick printed 'plane RMS nan mm' and 'FOUND' in the same breath, then closed 540 mm blind on a centre computed against that plane.

### 20  Height above the plane  [ok]

![height_map](stages/right_gripper/20_height_map.png)

- **formula** `h(P) = n . P + d,  signed;  positive is off the surface, toward the camera`
- **source** servo_pick_left.measure -- the same expression the live pick uses to separate the cube from the table
- **summary** 20193 of 42779 points stand more than 5 mm off the plane (47.20%) | tallest 357.1 mm

| quantity | value | unit |
| --- | ---: | --- |
| height threshold | 5 | mm |
| points above it | 20193 | count |
| that fraction | 47.203 | % |
| tallest point | 357.11 | mm |
| lowest point | -30.29 | mm |
| median height | 2.195 | mm |

- **note** The floor is 5 mm because the plane RMS is under 1 mm on this sensor. Set it below the depth noise and the table itself becomes an object; set it far above and a flat item is missed.

### 21  The surface height as a MODE, not a mean  [ok]

![mode_surface](stages/right_gripper/21_mode_surface.png)

- **formula** `bin at 2 mm; take the fullest bin; require it to hold >= 20% of the points; refine as the mean of everything within +/-6 mm of it`
- **source** srl_perception/surface_from_depth.py, applied along the fitted plane normal because this program has no camera-to-world transform; the node applies it to world z
- **summary** modal bin holds 24.6% of 42779 points (floor 20%) | refined -0.2981 m, spread 1.65 mm | a plain MEAN would say -0.2512 m, which is 46.9 mm out | RANSAC's own offset is -0.2981 m, 0.07 mm from this

| quantity | value | unit |
| --- | ---: | --- |
| bin width | 2 | mm |
| points binned | 42779 | count |
| share in the fullest bin | 24.61 | % |
| share required | 20 | % |
| modal bin centre | -0.2973 | m |
| refined estimate | -0.2981 | m |
| spread of the inliers | 1.651 | mm |
| inlier band | 6 | mm |
| a plain mean would say | -0.2512 | m |
| that mean's error | 46.95 | mm |
| RANSAC's own offset | -0.298 | m |
| the two estimators differ by | 0.071 | mm |

- **note** Two independent estimators of one surface, so they can be compared: RANSAC votes on inliers, this votes on the fullest histogram bin. A mean is dragged up by everything standing on the table and down by every hole, and it MOVES as the objects move -- the 'measurement' would change between trials on a motionless table.

### 22  What is standing on the surface  [ok]

![on_surface](stages/right_gripper/22_on_surface.png)

- **formula** `{ pixels whose lifted point has h > 5 mm }, 8-connected`
- **source** the plane of stage 18 fitted in the colour frame; heights from the aligned depth of stage 16
- **summary** 152263 px, 103448 pts, top 64 mm | 38812 px, 26656 pts, top 224 mm | 13640 px, 9771 pts, top 228 mm | 727 px, 538 pts, top 12 mm | 68 px, 58 pts, top 7 mm | 62 px, 54 pts, top 9 mm

| quantity | value | unit |
| --- | ---: | --- |
| pieces standing on the surface | 7 | count |
| height threshold | 5 | mm |
| largest piece | 152263 | px |
| tallest piece | 227.7 | mm |


*pieces above the plane* -- 7 rows, full data in `data/right_gripper/22_on_surface__pieces_above_the_plane.csv`

- **note** This is the pre-2020 method on its own -- threshold above a plane, then cluster what is left. It is exactly what fails when two 40 mm cubes sit 20 mm apart: connectivity fuses them into one object 140 mm across. Compare this figure with stage 11.

### 23  The measurement the pick actually acts on  [ok]

![cube_measurement](stages/right_gripper/23_cube_measurement.png)

- **formula** `cube = colour(u,v) AND h > 5 mm;  foot = mean(Pk) - n (n.mean(Pk) + d);  centre = foot + n * max(h)/2`
- **source** scripts/servo_pick_left.py:measure -- this is the measurement the servo loop nulls against, in the CAMERA's frame, where the data is good
- **summary** 160 object points | top 55.0 mm above the plane | centre (0.0511, 0.0519, 0.7260) m in the camera frame | plane RMS 1.59 mm, inlier fraction 0.53

| quantity | value | unit |
| --- | ---: | --- |
| object points | 160 | count |
| points that are the colour but ON the plane | 708 | count |
| points off the plane but not the colour | 20033 | count |
| top above the plane | 55.01 | mm |
| median height | 19.57 | mm |
| centre x | 0.0511 | m |
| centre y | 0.0519 | m |
| centre z | 0.726 | m |
| range to the centre | 0.7296 | m |
| plane RMS | 1.593 | mm |
| plane inlier fraction | 0.5279 | - |
| minimum points the pick requires | 60 | count |

- **note** Two independent gates, and that is the point: colour alone picks up the green pad the cube stands on and anything green in the room; height alone picks up everything on the table. The centre uses the foot point rather than the cloud's centroid because a depth camera sees a SHELL -- the front surface only -- and a shell's centroid is biased toward the camera, measured at 10.5 mm in the error budget against a 30 mm capture gate.

### 24  Splitting a cube from the pad it stands on  [ok]

![layers](stages/right_gripper/24_layers.png)

- **formula** `find the MODAL height bin (5 mm bins); cut 12 mm above it; recurse on what is left, twice`
- **source** srl_perception/segment_lift.py:_layers
- **summary** layer 1: 4-41 mm, 46552 points | layer 2: 41-53 mm, 14079 points | layer 3: 53-64 mm, 8239 points

| quantity | value | unit |
| --- | ---: | --- |
| region area | 265965 | px |
| points lifted from it | 159236 | count |
| points above the plane | 68870 | count |
| pixels removed by the 1 px erosion | 3748 | px |
| layers found | 3 | count |
| lowest point | 4 | mm |
| highest point | 64.23 | mm |


*height layers* -- 3 rows, full data in `data/right_gripper/24_layers__height_layers.csv`

- **note** Appearance decides what is SIDE BY SIDE; height decides what is STACKED. A blue cube on a blue pad is correctly ONE region -- same colour, touching, no edge between them. And the split cannot look for an empty band, because there is none: the cube RESTS on the pad, so its sides fill every bin. What is there is a MODE -- the support's top face is a large flat area and dominates the histogram.

### 25  Rotating calipers, against the two controls  [ok]

![min_width](stages/right_gripper/25_min_width.png)

- **formula** `width = min over theta of [ max(p.u(theta)) - min(p.u(theta)) ],  u(theta) = (cos, sin) in the support plane, swept at 0.25 deg`
- **source** srl_perception/segment_lift.py:_min_width and rgbd_grasp.min_width_frame
- **summary** calipers 192.0 mm | PCA's short axis 221.3 mm | axis-aligned box 241.7 mm | long axis 390.4 mm, height 60.2 mm | jaws close on 85 mm, so this is TOO WIDE

| quantity | value | unit |
| --- | ---: | --- |
| instance points | 68870 | count |
| minimum width, rotating calipers | 191.96 | mm |
| at this angle in the plane | 170 | deg |
| PCA short axis, THE CONTROL | 221.34 | mm |
| axis-aligned box, THE CONTROL | 241.74 | mm |
| PCA overstates by | 29.39 | mm |
| long axis | 390.41 | mm |
| height | 60.23 | mm |
| jaw span | 85 | mm |
| graspable as it lies | no | - |
| sweep step | 0.5 | deg |

- **note** The two controls are here because both were used and both were wrong. The principal axes of a SQUARE are degenerate -- equal variance in both directions -- so SVD returns the diagonal and a 40 mm cube measures 40*sqrt(2) = 53 mm; against an 85 mm jaw that refuses a 60 mm object as ungraspable. The error that matters is not the 13 mm, it is the DIRECTION: a jaw told to close across a diagonal approaches the corner and the object rolls out.

### 26  Dropping masks that are unions of finer ones  [ok]

![nested](stages/right_gripper/26_nested.png)

- **formula** `voxelise each instance at 5 mm; accept smallest first; reject one whose claimed-voxel overlap exceeds 0.55`
- **source** srl_perception/segment_lift.py:_suppress_nested
- **summary** 25 instances in, 20 kept, 5 dropped | overlaps: 0.90, 0.96, 1.00, 1.00, 0.76

| quantity | value | unit |
| --- | ---: | --- |
| instances in | 25 | count |
| kept | 20 | count |
| dropped as already explained | 5 | count |
| voxel size | 5 | mm |
| overlap fraction that rejects | 0.55 | - |


*per instance* -- 25 rows, full data in `data/right_gripper/26_nested__per_instance.csv`

- **note** FastSAM returns a HIERARCHY, not a partition: for one picture it hands back the cube, the pad, and a region covering both. All three are legitimate. Lifted, they became three objects in the same space and the merge absorbed a 40 mm cube -- measured EXACT at 40.3 mm against a 40 mm truth -- into a 148 mm blob that was then refused as ungraspable. Volume, not pixels: two masks that overlap in the image can be a metre apart in depth.

### 27  The parallel-jaw grasp  [refused]

![grasp](stages/right_gripper/27_grasp.png)

- **refused** the grasp planner refused, which is an ANSWER: object is 110 mm across its narrowest axis; the Robotiq 85 opens 75 mm usable. It cannot be grasped as it lies.

### 28  Fiducials: the primary detector  [refused]

![apriltag](stages/right_gripper/28_apriltag.png)

- **refused** no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That is an answer, not a failure: a tag either decodes or it does not, which is exactly why this is the PRIMARY detector and colour is the capped fallback. Detector used: cv2.aruco.ArucoDetector (OpenCV 4.11.0).

### 29  The support surface, as an object  [ok]

![table](stages/right_gripper/29_table.png)

- **formula** `fit the plane, keep its inliers, span them in the plane's own basis; extent taken at the 2nd and 98th percentile so one stray return cannot stretch it`
- **source** the plane of stage 18, expressed as a rectangle in the camera frame
- **summary** 0.428 x 0.467 m of surface, 0.290 m away, flat to 1.69 mm

| quantity | value | unit |
| --- | ---: | --- |
| points on the surface | 175957 | count |
| surface extent along its first axis | 0.4284 | m |
| surface extent along its second axis | 0.4671 | m |
| area spanned | 0.2001 | m2 |
| perpendicular distance from the camera | 0.2902 | m |
| distance to the middle of that extent | 0.6679 | m |
| nearest point on the surface | 0.4645 | m |
| furthest point on the surface | 0.9372 | m |
| tilt from the optical axis | 71.71 | deg |
| flatness, RMS of the inliers | 1.688 | mm |

- **note** This is a MEASURED surface, not the declared one. srl_experiments/work_surface.py has had set_measured() since it was written and nothing ever called it, so a table 20 mm out would have been absorbed silently into every grasp.

### 30  Every object, as an oriented 3-D box  [ok]

![boxes3d](stages/right_gripper/30_boxes3d.png)

- **formula** `centre = mid-extent ACROSS the surface + (foot + top/2) ALONG the normal; axes = (minimum-width direction, its perpendicular, the plane normal); size = the spans along them`
- **source** segment_lift._measure for the horizontal centre and rgbd_grasp.shell_corrected_centre's reasoning for the vertical -- the combination is checked to 2 mm against a constructed cube in --self-test
- **summary** 12 objects, 0.387 to 0.897 m away

| quantity | value | unit |
| --- | ---: | --- |
| objects posed | 12 | count |
| nearest object | 0.3869 | m |
| furthest object | 0.8966 | m |
| largest footprint | 599.6 | mm |
| tallest object | 223.9 | mm |
| graspable within the 85 mm jaw | 7 | count |


*every object, in the camera frame* -- 12 rows, full data in `data/right_gripper/30_boxes3d__every_object__in_the_camera_frame.csv`

- **note** These coordinates are in the CAMERA's frame. Putting them in the robot's frame needs the camera extrinsic, which on the wrist is measured at ~8.5 deg wrong and rotates with the joint -- 34 mm of error at 0.23 m. That is why the pick servos on an error measured HERE rather than planning through that transform.

### 31  How far apart everything is  [ok]

![distances](stages/right_gripper/31_distances.png)

- **formula** `range = |centre| (the camera is the origin); height = n.c + d; separation = |c_i - c_j|; free gap subtracts each object's half-width`
- **source** the poses of stage 30, all in the camera's own frame
- **summary** 10 objects, nearest 0.394 m, closest pair 0.006 m

| quantity | value | unit |
| --- | ---: | --- |
| objects measured | 10 | count |
| nearest object | 0.3936 | m |
| furthest object | 0.8966 | m |
| closest pair | 0.0057 | m |
| tightest free gap | 0 | m |
| what the origin IS | the wrist camera, so range is distance from the HAND | - |


*each object* -- 10 rows, full data in `data/right_gripper/31_distances__each_object.csv`


*between objects* -- 45 rows, full data in `data/right_gripper/31_distances__between_objects.csv`

- **note** On a WRIST camera the origin is the hand, so 'range' is already the distance the arm has to close -- that is the quantity the servo loop nulls. On the scene camera it is the distance from the camera, and turning that into a distance from the ARM needs the scene-camera-to-robot calibration, which this repository does not have measured.

### 32  People in the picture  [refused]

![people](stages/right_gripper/32_people.png)

- **refused** no person is in this frame. MediaPipe ran in 2.2 s and found nobody, which is an answer -- the wearer-tracking safety case only ever makes the modelled body BIGGER, so 'nobody seen' falls back to the mannequin rather than to an empty scene.

### 33  The robot in the picture, and what is NOT detected  [ok]

![self_view](stages/right_gripper/33_self_view.png)

- **formula** `{ pixels with 0 < depth < 0.08 m } -- nearer than anything this camera works on`
- **source** REPLAY of Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=0
- **summary** 0 px nearer than 0.08 m

| quantity | value | unit |
| --- | ---: | --- |
| pixels nearer than the gate | 0 | px |
| that fraction | 0 | % |
| nearest return of all | 0.299 | m |
| blobs of near return | 0 | count |
| this camera is on the hand | yes | - |
| near edge of this camera's band | 0.08 | m |


*near-field blobs* -- 0 rows, full data in `data/right_gripper/33_self_view__near_field_blobs.csv`

- **note** THE ROBOT ITSELF IS NOT DETECTED IN THE IMAGE, and no stage in this program claims to. Nothing in this repository finds an arm in a picture: the robot's pose comes from its own encoders through the URDF, and drawing it into a camera image needs the camera-to-robot extrinsic -- measured at ~8.5 deg wrong on the wrist, and never measured at all for the scene cameras. On a wrist camera the near-field blobs above are usually the gripper's own fingers entering the bottom of the frame, which is also why the last ~80 mm of a grasp is completed from the last good fix rather than from live vision.

## scene_hd

REPLAY of HD USB webcam /dev/video6 (HD USB CAMERA: HD USB CAMERA 32e4:0317), MJPG 1280x720

- NO INTRINSICS: this camera has never been calibrated in this repository, so every stage that needs fx, fy, cx, cy refuses rather than assuming cx = w/2. Calibrate it with scripts/calibrate_scene_camera.py.

Contact sheet: `sheet_scene_hd.png`

### 01  The frame, as the camera delivered it  [ok]

![raw_colour](stages/scene_hd/01_raw_colour.png)

- **formula** `none -- this is the input every later stage is a function of`
- **source** REPLAY of HD USB webcam /dev/video6 (HD USB CAMERA: HD USB CAMERA 32e4:0317), MJPG 1280x720
- **summary** 1280 x 720 px | focus (variance of Laplacian) 1141 | 8.58% of pixels clipped white, 1.95% crushed black

| quantity | value | unit |
| --- | ---: | --- |
| frame width | 1280 | px |
| frame height | 720 | px |
| focus, variance of Laplacian | 1141 | - |
| pixels clipped white | 8.58 | % |
| pixels crushed black | 1.954 | % |
| mean blue | 129.86 | 0-255 |
| mean green | 130.33 | 0-255 |
| mean red | 127.16 | 0-255 |
| median grey | 135 | 0-255 |
| grey std | 89.47 | 0-255 |

- **note** NO INTRINSICS: this camera has never been calibrated in this repository, so every stage that needs fx, fy, cx, cy refuses rather than assuming cx = w/2. Calibrate it with scripts/calibrate_scene_camera.py.

### 02  Intrinsics and the ray each pixel stands for  [refused]

![intrinsics](stages/scene_hd/02_intrinsics.png)

- **refused** this camera has no intrinsics in this repository, and this stage is arithmetic on fx, fy, cx, cy. Assuming cx = w/2 is exactly the error that throws a ray by 122 px on the Kinova module -- 94 mm at 1 m, three times the grasp tolerance. Calibrate it: scripts/calibrate_scene_camera.py

### 03  BGR to HSV, the space the colour tests live in  [ok]

![hsv](stages/scene_hd/03_hsv.png)

- **formula** `V = max(B,G,R);  S = (V - min(B,G,R))/V;  H = the sector of the RGB hexagon, 0..179 in OpenCV (not 0..359)`
- **source** REPLAY of HD USB webcam /dev/video6 (HD USB CAMERA: HD USB CAMERA 32e4:0317), MJPG 1280x720
- **summary** median H 75  S 5  V 145 | 82.8% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 75 | 0-179 |
| median saturation | 5 | 0-255 |
| median value | 145 | 0-255 |
| pixels below S=80 | 82.773 | % |
| pixels below V=40 | 22.311 | % |
| pixels above V=200 | 31.584 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/scene_hd/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 1777 px selected (0.193% of the frame) | of those, median S 116 and median V 84

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 1777 | px |
| fraction of the frame | 0.1928 | % |
| median S of the selected | 116 | 0-255 |
| median V of the selected | 84 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/scene_hd/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 1777 px in, 501 px out, 1276 px removed (71.8%) | 137 separate blobs before, 1 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 1777 | px |
| pixels after | 501 | px |
| pixels removed | 1276 | px |
| removed | 71.806 | % |
| blobs before | 137 | count |
| blobs after | 1 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/scene_hd/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** REPLAY of HD USB webcam /dev/video6 (HD USB CAMERA: HD USB CAMERA 32e4:0317), MJPG 1280x720
- **summary** 1 components over 40 px | largest 501 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 1 | count |
| largest area | 501 | px |
| total area kept | 501 | px |
| median area | 501 | px |


*every component* -- 1 rows, full data in `data/scene_hd/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/scene_hd/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** white 283537 | black 201828 | cyan 45512 | red 9295 | green 574 | orange 496

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 8 | count |
| largest term | white | - |
| its pixel count | 283537 | px |


*pixels per colour term* -- 11 rows, full data in `data/scene_hd/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/scene_hd/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 501 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 574 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 534 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

### 08  The region-MEAN ratio test, and its near-black guard  [ok]

![region_mean](stages/scene_hd/08_region_mean.png)

- **formula** `green  <=>  mean_G > 1.25 * mean_B  and  mean_G > 1.8 * mean_R,  refused outright when max(B,G,R) < 25`
- **source** srl_perception/prompt_detector.py:_colour_matches
- **summary** 1 of 1 regions pass

| quantity | value | unit |
| --- | ---: | --- |
| regions tested | 1 | count |
| regions passing | 1 | count |
| regions rejected as near-black | 0 | count |
| blue ratio threshold | 1.25 | - |
| red ratio threshold | 1.8 | - |
| near-black floor | 25 | 0-255 |


*region means and verdicts* -- 1 rows, full data in `data/scene_hd/08_region_mean__region_means_and_verdicts.csv`

- **note** Per-pixel hue picked the room's teal robots three times: their shadowed hue is 76 against the target cube's 74, two apart and inside noise. Region MEANS separate them, because teal carries far more blue. The near-black guard exists because a region measuring (4.1, 8.8, 3.9) -- visually black -- satisfies the ratios on sensor noise alone and was reported as the cube.

### 09  Rotated box, in-image yaw, and the degeneracy gate  [ok]

![minarearect](stages/scene_hd/09_minarearect.png)

- **formula** `minAreaRect -> ((cx,cy), (w,h), angle);  yaw is published as q = (0, 0, sin(angle/2), cos(angle/2)) ONLY when max(w,h)/min(w,h) >= 1.15`
- **source** srl_perception/colour_shape_detector.py
- **summary** 21 x 22 px, 90.0 deg, aspect 1.05, identity (not measured)

| quantity | value | unit |
| --- | ---: | --- |
| contours measured | 1 | count |
| aspect gate for publishing yaw | 1.15 | - |
| yaws published | 0 | count |
| yaws withheld as degenerate | 1 | count |


*rotated boxes* -- 1 rows, full data in `data/scene_hd/09_minarearect__rotated_boxes.csv`

- **note** Rotation about the OPTICAL AXIS is exactly what this measures; out-of-plane tilt is what it cannot see. The detector published identity for months while measuring the angle into a debug string, so an object at 30 deg got a square grasp and no check could tell that from a square object. A near-square blob still gets identity, because minAreaRect's angle is degenerate there and publishing it would be inventing a direction.

### 10  The size-at-range gate  [refused]

![size_at_range](stages/scene_hd/10_size_at_range.png)

- **refused** this camera has no intrinsics in this repository, and this stage is arithmetic on fx, fy, cx, cy. Assuming cx = w/2 is exactly the error that throws a ray by 122 px on the Kinova module -- 94 mm at 1 m, three times the grasp tolerance. Calibrate it: scripts/calibrate_scene_camera.py

### 11  FastSAM: every region in the picture, no prompt  [ok]

![fastsam](stages/scene_hd/11_fastsam.png)

- **formula** `object-agnostic instance masks; no class, no prompt, no scene knowledge -- 'what are the regions', not 'where is the green cube'`
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 1.3 s for this frame
- **summary** 109 regions | largest 228216 px (24.8% of frame), smallest 57 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 109 | count |
| largest region | 228216 | px |
| largest as a fraction of the frame | 24.76 | % |
| smallest region | 57 | px |
| median region | 1440 | px |
| inference time | 1.31 | s |
| input size | 1024 | px |
| confidence threshold | 0.25 | - |
| NMS IoU | 0.7 | - |
| device | cpu | - |


*the twenty largest regions* -- 20 rows, full data in `data/scene_hd/11_fastsam__the_twenty_largest_regions.csv`

- **note** This is what replaced clustering. Two 40 mm cubes on a 60 mm pitch are 20 mm apart, which is the cluster distance itself, so depth-clustering fused them into one object 140 mm across the jaws and refused it. In the PICTURE they are not remotely ambiguous. Masks decide WHAT is an object; depth decides WHERE it is.

### 12  The area filter: noise below, the table above  [ok]

![area_filter](stages/scene_hd/12_area_filter.png)

- **formula** `keep iff  60 px <= area <= 0.35 * W * H  (= 322559 px here)`
- **source** srl_perception/segment_lift.py: MIN_AREA_PX, MAX_AREA_FRAC
- **summary** 109 in, 108 kept, 1 too small, 0 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 109 | count |
| kept | 108 | count |
| rejected as noise | 1 | count |
| rejected as too large | 0 | count |
| lower bound | 60 | px |
| upper bound | 322560 | px |
| upper bound as a fraction | 0.35 | - |

- **note** The upper bound is not tidiness. A segmenter returns the table and the room as legitimate regions, and a region covering a third of the frame is never a thing to be picked up.

### 13  Segment everything, then pick the one that matches  [ok]

![segment_match](stages/scene_hd/13_segment_match.png)

- **formula** `for each region: mean BGR over the MASK, then the ratio test of stage 08; score = min(1, area/2000)`
- **source** srl_perception/prompt_detector.py:_segment -- the backend that actually found the target on this rig
- **summary** 598 px at (612, 442), mean BGR [105.1, 139.3, 74.0]

| quantity | value | unit |
| --- | ---: | --- |
| colour word parsed | green | - |
| regions considered | 109 | count |
| regions matched | 1 | count |
| regions rejected on colour | 108 | count |
| best score | 0.299 | 0-1 |


*matched regions* -- 1 rows, full data in `data/scene_hd/13_segment_match__matched_regions.csv`

- **note** Naming the cube failed at every confidence -- it is ~19 px and dark, and an open-vocabulary detector cannot recognise it as anything. Segmenting it is trivial: 89 regions, exactly one green-dominant, no false positives. The MASK is the other win: a colour threshold gives whatever pixels passed, a segment gives the object's actual extent, which is what a width measurement needs.

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](stages/scene_hd/14_yoloworld.png)

- **refused** YOLO-World ran in 4.0 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

### 15  The depth frame, and where it has no answer  [by_design]

![raw_depth](stages/scene_hd/15_raw_depth.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

### 16  Depth put into the colour frame  [by_design]

![alignment](stages/scene_hd/16_alignment.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

### 17  Pixels and depth become a cloud  [by_design]

![deprojection](stages/scene_hd/17_deprojection.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

### 18  RANSAC: the support plane  [by_design]

![ransac](stages/scene_hd/18_ransac.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

### 19  The PCA refinement, and the NaN it once produced  [by_design]

![pca_refine](stages/scene_hd/19_pca_refine.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

### 20  Height above the plane  [by_design]

![height_map](stages/scene_hd/20_height_map.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

### 21  The surface height as a MODE, not a mean  [by_design]

![mode_surface](stages/scene_hd/21_mode_surface.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

### 22  What is standing on the surface  [by_design]

![on_surface](stages/scene_hd/22_on_surface.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

### 23  The measurement the pick actually acts on  [refused]

![cube_measurement](stages/scene_hd/23_cube_measurement.png)

- **refused** this camera has no intrinsics in this repository, and this stage is arithmetic on fx, fy, cx, cy. Assuming cx = w/2 is exactly the error that throws a ray by 122 px on the Kinova module -- 94 mm at 1 m, three times the grasp tolerance. Calibrate it: scripts/calibrate_scene_camera.py

### 24  Splitting a cube from the pad it stands on  [by_design]

![layers](stages/scene_hd/24_layers.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

### 25  Rotating calipers, against the two controls  [by_design]

![min_width](stages/scene_hd/25_min_width.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

### 26  Dropping masks that are unions of finer ones  [by_design]

![nested](stages/scene_hd/26_nested.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

### 27  The parallel-jaw grasp  [by_design]

![grasp](stages/scene_hd/27_grasp.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

### 28  Fiducials: the primary detector  [refused]

![apriltag](stages/scene_hd/28_apriltag.png)

- **refused** no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That is an answer, not a failure: a tag either decodes or it does not, which is exactly why this is the PRIMARY detector and colour is the capped fallback. Detector used: cv2.aruco.ArucoDetector (OpenCV 4.11.0).

### 29  The support surface, as an object  [refused]

![table](stages/scene_hd/29_table.png)

- **refused** this camera has no intrinsics in this repository, and this stage is arithmetic on fx, fy, cx, cy. Assuming cx = w/2 is exactly the error that throws a ray by 122 px on the Kinova module -- 94 mm at 1 m, three times the grasp tolerance. Calibrate it: scripts/calibrate_scene_camera.py

### 30  Every object, as an oriented 3-D box  [refused]

![boxes3d](stages/scene_hd/30_boxes3d.png)

- **refused** this camera has no intrinsics in this repository, and this stage is arithmetic on fx, fy, cx, cy. Assuming cx = w/2 is exactly the error that throws a ray by 122 px on the Kinova module -- 94 mm at 1 m, three times the grasp tolerance. Calibrate it: scripts/calibrate_scene_camera.py

### 31  How far apart everything is  [refused]

![distances](stages/scene_hd/31_distances.png)

- **refused** this camera has no intrinsics in this repository, and this stage is arithmetic on fx, fy, cx, cy. Assuming cx = w/2 is exactly the error that throws a ray by 122 px on the Kinova module -- 94 mm at 1 m, three times the grasp tolerance. Calibrate it: scripts/calibrate_scene_camera.py

### 32  People in the picture  [ok]

![people](stages/scene_hd/32_people.png)

- **formula** `markerless pose landmarks; each landmark's range read from the depth at its own pixel (median of a 9x9 window)`
- **source** MediaPipe PoseLandmarker (full), num_poses=4, run in .venv_pose as a separate interpreter -- the same model srl_perception/wearer_tracker_node.py uses, 1.4 s
- **summary** 1 person(s)

| quantity | value | unit |
| --- | ---: | --- |
| people detected | 1 | count |
| landmarks per person | 33 | count |
| inference time | 1.41 | s |
| depth attached to landmarks | no | - |


*body landmarks* -- 8 rows, full data in `data/scene_hd/32_people__body_landmarks.csv`

- **note** This is the wearer-tracking path. The rule the whole safety case rests on: a camera may only ever make the modelled body BIGGER -- fuse() keeps whichever of the tracked and mannequin primitive is CLOSER to the robot, per part, so a body measured further away changes nothing and a lost track falls back rather than shrinking the person.

### 33  The robot in the picture, and what is NOT detected  [by_design]

![self_view](stages/scene_hd/33_self_view.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

## scene_rs

REPLAY of RealSense D435i across the room, depth 640x480@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.

- ORIENTATION CORRECTED 2026-08-30: this frame had been rotated 180 deg on the assumption that the camera is mounted upside down. It is not. Measured against the HD webcam, which watches the same scene and is rotated by nothing: as-saved r=+0.269, rotated r=+0.662. The rotation has been undone on the image, the depth and the principal point together (cx=W-1-cx).
- liveness: 12 frames, 7 distinct depth images, 7 distinct frame numbers, 15% of pixels valid

Contact sheet: `sheet_scene_rs.png`

### 01  The frame, as the camera delivered it  [ok]

![raw_colour](stages/scene_rs/01_raw_colour.png)

- **formula** `none -- this is the input every later stage is a function of`
- **source** REPLAY of RealSense D435i across the room, depth 640x480@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 640 x 480 px | focus (variance of Laplacian) 410 | 1.52% of pixels clipped white, 0.00% crushed black

| quantity | value | unit |
| --- | ---: | --- |
| frame width | 640 | px |
| frame height | 480 | px |
| focus, variance of Laplacian | 409.7 | - |
| pixels clipped white | 1.519 | % |
| pixels crushed black | 0 | % |
| mean blue | 108.15 | 0-255 |
| mean green | 110.93 | 0-255 |
| mean red | 108.85 | 0-255 |
| median grey | 116 | 0-255 |
| grey std | 64.91 | 0-255 |

- **note** ORIENTATION CORRECTED 2026-08-30: this frame had been rotated 180 deg on the assumption that the camera is mounted upside down. It is not. Measured against the HD webcam, which watches the same scene and is rotated by nothing: as-saved r=+0.269, rotated r=+0.662. The rotation has been undone on the image, the depth and the principal point together (cx=W-1-cx).; liveness: 12 frames, 7 distinct depth images, 7 distinct frame numbers, 15% of pixels valid

### 02  Intrinsics and the ray each pixel stands for  [ok]

![intrinsics](stages/scene_rs/02_intrinsics.png)

- **formula** `bearing = ((u - cx)/fx, (v - cy)/fy);  X = bearing * Z`
- **source** REPLAY of RealSense D435i across the room, depth 640x480@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** fx 603.02  fy 603.13  cx 318.87  cy 231.22 | field of view 55.9 x 43.4 deg | principal point is 8.9 px from the image centre, which is 15 mm of sideways error at 1 m if you assume the centre

| quantity | value | unit |
| --- | ---: | --- |
| fx | 603.0215 | px |
| fy | 603.1288 | px |
| cx | 318.8748 | px |
| cy | 231.2179 | px |
| image centre u | 320 | px |
| image centre v | 240 | px |
| principal point offset from centre | 8.85 | px |
| that offset at 1 m range | 14.7 | mm |
| horizontal field of view | 55.906 | deg |
| vertical field of view | 43.398 | deg |
| ground sample distance at 1 m | 1.6583 | mm/px |


### 03  BGR to HSV, the space the colour tests live in  [ok]

![hsv](stages/scene_rs/03_hsv.png)

- **formula** `V = max(B,G,R);  S = (V - min(B,G,R))/V;  H = the sector of the RGB hexagon, 0..179 in OpenCV (not 0..359)`
- **source** REPLAY of RealSense D435i across the room, depth 640x480@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** median H 38  S 14  V 122 | 87.9% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 38 | 0-179 |
| median saturation | 14 | 0-255 |
| median value | 122 | 0-255 |
| pixels below S=80 | 87.905 | % |
| pixels below V=40 | 17.14 | % |
| pixels above V=200 | 8.781 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/scene_rs/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 274 px selected (0.089% of the frame) | of those, median S 150 and median V 120

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 274 | px |
| fraction of the frame | 0.0892 | % |
| median S of the selected | 149.5 | 0-255 |
| median V of the selected | 120 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/scene_rs/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 274 px in, 172 px out, 102 px removed (37.2%) | 13 separate blobs before, 1 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 274 | px |
| pixels after | 172 | px |
| pixels removed | 102 | px |
| removed | 37.226 | % |
| blobs before | 13 | count |
| blobs after | 1 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/scene_rs/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** REPLAY of RealSense D435i across the room, depth 640x480@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 1 components over 40 px | largest 172 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 1 | count |
| largest area | 172 | px |
| total area kept | 172 | px |
| median area | 172 | px |


*every component* -- 1 rows, full data in `data/scene_rs/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/scene_rs/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** black 58817 | white 26248 | cyan 10572 | red 1932 | orange 1580 | green 202

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 6 | count |
| largest term | black | - |
| its pixel count | 58817 | px |


*pixels per colour term* -- 11 rows, full data in `data/scene_rs/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/scene_rs/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 172 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 202 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 172 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

### 08  The region-MEAN ratio test, and its near-black guard  [ok]

![region_mean](stages/scene_rs/08_region_mean.png)

- **formula** `green  <=>  mean_G > 1.25 * mean_B  and  mean_G > 1.8 * mean_R,  refused outright when max(B,G,R) < 25`
- **source** srl_perception/prompt_detector.py:_colour_matches
- **summary** 1 of 1 regions pass

| quantity | value | unit |
| --- | ---: | --- |
| regions tested | 1 | count |
| regions passing | 1 | count |
| regions rejected as near-black | 0 | count |
| blue ratio threshold | 1.25 | - |
| red ratio threshold | 1.8 | - |
| near-black floor | 25 | 0-255 |


*region means and verdicts* -- 1 rows, full data in `data/scene_rs/08_region_mean__region_means_and_verdicts.csv`

- **note** Per-pixel hue picked the room's teal robots three times: their shadowed hue is 76 against the target cube's 74, two apart and inside noise. Region MEANS separate them, because teal carries far more blue. The near-black guard exists because a region measuring (4.1, 8.8, 3.9) -- visually black -- satisfies the ratios on sensor noise alone and was reported as the cube.

### 09  Rotated box, in-image yaw, and the degeneracy gate  [refused]

![minarearect](stages/scene_rs/09_minarearect.png)

- **refused** no contour over the 400 px floor in this frame, so there is no rotated box to fit

### 10  The size-at-range gate  [refused]

![size_at_range](stages/scene_rs/10_size_at_range.png)

- **refused** no blob over 400 px to test

### 11  FastSAM: every region in the picture, no prompt  [ok]

![fastsam](stages/scene_rs/11_fastsam.png)

- **formula** `object-agnostic instance masks; no class, no prompt, no scene knowledge -- 'what are the regions', not 'where is the green cube'`
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 1.1 s for this frame
- **summary** 96 regions | largest 75485 px (24.6% of frame), smallest 66 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 96 | count |
| largest region | 75485 | px |
| largest as a fraction of the frame | 24.57 | % |
| smallest region | 66 | px |
| median region | 841.5 | px |
| inference time | 1.1 | s |
| input size | 1024 | px |
| confidence threshold | 0.25 | - |
| NMS IoU | 0.7 | - |
| device | cpu | - |


*the twenty largest regions* -- 20 rows, full data in `data/scene_rs/11_fastsam__the_twenty_largest_regions.csv`

- **note** This is what replaced clustering. Two 40 mm cubes on a 60 mm pitch are 20 mm apart, which is the cluster distance itself, so depth-clustering fused them into one object 140 mm across the jaws and refused it. In the PICTURE they are not remotely ambiguous. Masks decide WHAT is an object; depth decides WHERE it is.

### 12  The area filter: noise below, the table above  [ok]

![area_filter](stages/scene_rs/12_area_filter.png)

- **formula** `keep iff  60 px <= area <= 0.35 * W * H  (= 107520 px here)`
- **source** srl_perception/segment_lift.py: MIN_AREA_PX, MAX_AREA_FRAC
- **summary** 96 in, 96 kept, 0 too small, 0 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 96 | count |
| kept | 96 | count |
| rejected as noise | 0 | count |
| rejected as too large | 0 | count |
| lower bound | 60 | px |
| upper bound | 107520 | px |
| upper bound as a fraction | 0.35 | - |

- **note** The upper bound is not tidiness. A segmenter returns the table and the room as legitimate regions, and a region covering a third of the frame is never a thing to be picked up.

### 13  Segment everything, then pick the one that matches  [ok]

![segment_match](stages/scene_rs/13_segment_match.png)

- **formula** `for each region: mean BGR over the MASK, then the ratio test of stage 08; score = min(1, area/2000)`
- **source** srl_perception/prompt_detector.py:_segment -- the backend that actually found the target on this rig
- **summary** 192 px at (319, 284), mean BGR [84.6, 124.5, 46.5]

| quantity | value | unit |
| --- | ---: | --- |
| colour word parsed | green | - |
| regions considered | 96 | count |
| regions matched | 1 | count |
| regions rejected on colour | 95 | count |
| best score | 0.096 | 0-1 |


*matched regions* -- 1 rows, full data in `data/scene_rs/13_segment_match__matched_regions.csv`

- **note** Naming the cube failed at every confidence -- it is ~19 px and dark, and an open-vocabulary detector cannot recognise it as anything. Segmenting it is trivial: 89 regions, exactly one green-dominant, no false positives. The MASK is the other win: a colour threshold gives whatever pixels passed, a segment gives the object's actual extent, which is what a width measurement needs.

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](stages/scene_rs/14_yoloworld.png)

- **refused** YOLO-World ran in 4.3 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

### 15  The depth frame, and where it has no answer  [ok]

![raw_depth](stages/scene_rs/15_raw_depth.png)

- **formula** `one 16-bit value per pixel, in MILLIMETRES on the wire, divided by 1000 here. 0 means 'no return' and is NOT a distance of zero.`
- **source** REPLAY of RealSense D435i across the room, depth 640x480@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 640 x 480 | 14.5% of pixels have a return | median 4.706 m, 5th-95th 2.427-9.668 m | 38526 px between the 0.15 and 8.00 m gate the pick path uses

| quantity | value | unit |
| --- | ---: | --- |
| depth width | 640 | px |
| depth height | 480 | px |
| pixels with a return | 44570 | px |
| that fraction | 14.51 | % |
| median range | 4.7061 | m |
| 5th percentile | 2.427 | m |
| 95th percentile | 9.668 | m |
| nearest return | 0.317 | m |
| furthest return | 65.535 | m |
| returns inside this camera's working band | 38526 | px |
| band near | 0.15 | m |
| band far | 8 | m |

- **note** A hole is not a far surface. Averaging a hole in as 0 drags every edge pixel toward the camera, which is why the RealSense average is taken over VALID pixels only.

### 16  Depth put into the colour frame  [ok]

![alignment](stages/scene_rs/16_alignment.png)

- **formula** `P = ((u-cx_d)z/fx_d, (v-cy_d)z/fy_d, z);  P' = P + t;  u' = fx_c X'/Z' + cx_c,  v' = fy_c Y'/Z' + cy_c`
- **source** aligned by librealsense (rs.align to colour); no re-projection done here
- **summary** no re-projection was needed

| quantity | value | unit |
| --- | ---: | --- |
| re-projection needed | no | - |
| aligner | librealsense rs.align | - |

- **note** The rotation between the two sensors is taken as IDENTITY here, because a recorded translation is all this program has. The live pick path uses the URDF FK between camera_depth_frame and camera_color_frame instead -- and that extrinsic is known to be ~8.5 deg wrong and to rotate with the wrist, which is why the pick servos in the camera frame rather than planning through it.

### 17  Pixels and depth become a cloud  [ok]

![deprojection](stages/scene_rs/17_deprojection.png)

- **formula** `X = (u - cx) Z / fx,   Y = (v - cy) Z / fy,   Z = depth`
- **source** rgbd_grasp.deproject / servo_pick_left.measure, with fx 603.02 fy 603.13 cx 318.87 cy 231.22
- **summary** 38526 points in the 0.15-8.00 m band | extent X -1.141 to 3.477, Y -3.051 to 1.457, Z 0.317 to 7.994 m

| quantity | value | unit |
| --- | ---: | --- |
| points lifted | 38526 | count |
| fx | 603.022 | px |
| fy | 603.129 | px |
| cx | 318.875 | px |
| cy | 231.218 | px |
| X minimum | -1.141 | m |
| X maximum | 3.4767 | m |
| Y minimum | -3.0512 | m |
| Y maximum | 1.4574 | m |
| Z minimum | 0.317 | m |
| Z maximum | 7.9937 | m |
| centroid X | 0.6143 | m |
| centroid Y | -0.31 | m |
| centroid Z | 4.2642 | m |

- **note** This is the whole of 'depth decides WHERE'. Everything after it is geometry on these points, in the CAMERA's frame -- no robot transform is involved yet, which is exactly why the pick measures its error here.

### 18  RANSAC: the support plane  [ok]

![ransac](stages/scene_rs/18_ransac.png)

- **formula** `draw 3 points -> n = (b-a) x (c-a) / |...|,  d = -n.a;  score = #{ |n.P + d| < 6 mm };  keep the best of 400 draws`
- **source** scripts/servo_pick_left.py:fit_plane, imported; fitted on 38526 of 38526 points
- **summary** normal (0.2580, -0.0210, -0.9659), offset 4.3516 m | 1475 of 38526 points are inliers (3.8%) | RMS 3.41 mm | 15.0 deg between the plane normal and the camera's optical axis

| quantity | value | unit |
| --- | ---: | --- |
| normal x | 0.258 | - |
| normal y | -0.021 | - |
| normal z | -0.9659 | - |
| offset d | 4.3516 | m |
| points fitted | 38526 | count |
| points scored | 38526 | count |
| inliers | 1475 | count |
| inlier fraction | 3.83 | % |
| residual RMS | 3.405 | mm |
| inlier tolerance | 6 | mm |
| RANSAC iterations | 400 | count |
| angle to the optical axis | 15 | deg |

- **note** A plane with too few inliers is not a plane. The pick refuses below 200 inliers rather than returning a geometry whose RMS is NaN -- everything downstream would treat that as a fix and the arm would move on it.

### 19  The PCA refinement, and the NaN it once produced  [ok]

![pca_refine](stages/scene_rs/19_pca_refine.png)

- **formula** `C = cov(Q - mean(Q)) for inliers Q;  n = eigvec(C)[:, argmin];  d = -n . mean(Q)   <-- BOTH halves, together`
- **source** scripts/servo_pick_left.py:fit_plane, imported
- **summary** the normal moved 0.000 deg | RMS 3.41 mm -> 3.41 mm | mixing the refined normal with the RANSAC offset selects 1475 inliers instead of 1475

| quantity | value | unit |
| --- | ---: | --- |
| normal moved | 0 | deg |
| RMS before refinement | 3.405 | mm |
| RMS after refinement | 3.405 | mm |
| inliers with the matched pair | 1475 | count |
| inliers with a mixed normal and offset | 1475 | count |
| inliers lost by mixing | 0 | count |

- **note** That last number is the bug this stage exists to show. The function once returned the REFINED normal with the OLD offset; those describe different planes, the inlier mask could select ZERO points, and mean() of an empty slice is NaN. The pick printed 'plane RMS nan mm' and 'FOUND' in the same breath, then closed 540 mm blind on a centre computed against that plane.

### 20  Height above the plane  [ok]

![height_map](stages/scene_rs/20_height_map.png)

- **formula** `h(P) = n . P + d,  signed;  positive is off the surface, toward the camera`
- **source** servo_pick_left.measure -- the same expression the live pick uses to separate the cube from the table
- **summary** 23275 of 38526 points stand more than 5 mm off the plane (60.41%) | tallest 4071.8 mm

| quantity | value | unit |
| --- | ---: | --- |
| height threshold | 5 | mm |
| points above it | 23275 | count |
| that fraction | 60.414 | % |
| tallest point | 4072 | mm |
| lowest point | -3421 | mm |
| median height | 45.928 | mm |

- **note** The floor is 5 mm because the plane RMS is under 1 mm on this sensor. Set it below the depth noise and the table itself becomes an object; set it far above and a flat item is missed.

### 21  The surface height as a MODE, not a mean  [ok]

![mode_surface](stages/scene_rs/21_mode_surface.png)

- **formula** `bin at 2 mm; take the fullest bin; require it to hold >= 20% of the points; refine as the mean of everything within +/-6 mm of it`
- **source** srl_perception/surface_from_depth.py, applied along the fitted plane normal because this program has no camera-to-world transform; the node applies it to world z
- **summary** modal bin holds 0.7% of 38526 points (floor 20%) | refined -2.6643 m, spread 3.29 mm | a plain MEAN would say -3.9538 m, which is 1289.6 mm out | RANSAC's own offset is -4.3516 m, 1687.39 mm from this

| quantity | value | unit |
| --- | ---: | --- |
| bin width | 2 | mm |
| points binned | 38526 | count |
| share in the fullest bin | 0.74 | % |
| share required | 20 | % |
| modal bin centre | -2.664 | m |
| refined estimate | -2.6643 | m |
| spread of the inliers | 3.288 | mm |
| inlier band | 6 | mm |
| a plain mean would say | -3.9539 | m |
| that mean's error | 1290 | mm |
| RANSAC's own offset | -4.3516 | m |
| the two estimators differ by | 1687 | mm |

- **note** Two independent estimators of one surface, so they can be compared: RANSAC votes on inliers, this votes on the fullest histogram bin. A mean is dragged up by everything standing on the table and down by every hole, and it MOVES as the objects move -- the 'measurement' would change between trials on a motionless table.

### 22  What is standing on the surface  [ok]

![on_surface](stages/scene_rs/22_on_surface.png)

- **formula** `{ pixels whose lifted point has h > 5 mm }, 8-connected`
- **source** the plane of stage 18 fitted in the colour frame; heights from the aligned depth of stage 16
- **summary** 6382 px, 6254 pts, top 1601 mm | 5465 px, 5284 pts, top 1715 mm | 3067 px, 2982 pts, top 1718 mm | 2560 px, 2487 pts, top 2406 mm | 1665 px, 1634 pts, top 88 mm | 1600 px, 1578 pts, top 1479 mm

| quantity | value | unit |
| --- | ---: | --- |
| pieces standing on the surface | 19 | count |
| height threshold | 5 | mm |
| largest piece | 6382 | px |
| tallest piece | 4072 | mm |


*pieces above the plane* -- 19 rows, full data in `data/scene_rs/22_on_surface__pieces_above_the_plane.csv`

- **note** This is the pre-2020 method on its own -- threshold above a plane, then cluster what is left. It is exactly what fails when two 40 mm cubes sit 20 mm apart: connectivity fuses them into one object 140 mm across. Compare this figure with stage 11.

### 23  The measurement the pick actually acts on  [refused]

![cube_measurement](stages/scene_rs/23_cube_measurement.png)

- **refused** only 20 points are both green-coloured AND more than 5 mm above the plane (the pick needs 60 and returns None below that). 0 points are coloured but ON the plane; 23255 stand off the plane but are not the colour.

### 24  Splitting a cube from the pad it stands on  [ok]

![layers](stages/scene_rs/24_layers.png)

- **formula** `find the MODAL height bin (5 mm bins); cut 12 mm above it; recurse on what is left, twice`
- **source** srl_perception/segment_lift.py:_layers
- **summary** layer 1: 4-21 mm, 724 points | layer 2: 21-33 mm, 445 points | layer 3: 33-4072 mm, 4693 points

| quantity | value | unit |
| --- | ---: | --- |
| region area | 75485 | px |
| points lifted from it | 9293 | count |
| points above the plane | 5862 | count |
| pixels removed by the 1 px erosion | 4339 | px |
| layers found | 3 | count |
| lowest point | 4 | mm |
| highest point | 4072 | mm |


*height layers* -- 3 rows, full data in `data/scene_rs/24_layers__height_layers.csv`

- **note** Appearance decides what is SIDE BY SIDE; height decides what is STACKED. A blue cube on a blue pad is correctly ONE region -- same colour, touching, no edge between them. And the split cannot look for an empty band, because there is none: the cube RESTS on the pad, so its sides fill every bin. What is there is a MODE -- the support's top face is a large flat area and dominates the histogram.

### 25  Rotating calipers, against the two controls  [ok]

![min_width](stages/scene_rs/25_min_width.png)

- **formula** `width = min over theta of [ max(p.u(theta)) - min(p.u(theta)) ],  u(theta) = (cos, sin) in the support plane, swept at 0.25 deg`
- **source** srl_perception/segment_lift.py:_min_width and rgbd_grasp.min_width_frame
- **summary** calipers 1313.6 mm | PCA's short axis 1322.1 mm | axis-aligned box 1722.4 mm | long axis 2440.5 mm, height 4067.8 mm | jaws close on 85 mm, so this is TOO WIDE

| quantity | value | unit |
| --- | ---: | --- |
| instance points | 5862 | count |
| minimum width, rotating calipers | 1314 | mm |
| at this angle in the plane | 103 | deg |
| PCA short axis, THE CONTROL | 1322 | mm |
| axis-aligned box, THE CONTROL | 1722 | mm |
| PCA overstates by | 8.49 | mm |
| long axis | 2441 | mm |
| height | 4068 | mm |
| jaw span | 85 | mm |
| graspable as it lies | no | - |
| sweep step | 0.5 | deg |

- **note** The two controls are here because both were used and both were wrong. The principal axes of a SQUARE are degenerate -- equal variance in both directions -- so SVD returns the diagonal and a 40 mm cube measures 40*sqrt(2) = 53 mm; against an 85 mm jaw that refuses a 60 mm object as ungraspable. The error that matters is not the 13 mm, it is the DIRECTION: a jaw told to close across a diagonal approaches the corner and the object rolls out.

### 26  Dropping masks that are unions of finer ones  [ok]

![nested](stages/scene_rs/26_nested.png)

- **formula** `voxelise each instance at 5 mm; accept smallest first; reject one whose claimed-voxel overlap exceeds 0.55`
- **source** srl_perception/segment_lift.py:_suppress_nested
- **summary** 49 instances in, 41 kept, 8 dropped | overlaps: 0.97, 0.80, 0.80, 0.95, 0.82, 0.76

| quantity | value | unit |
| --- | ---: | --- |
| instances in | 49 | count |
| kept | 41 | count |
| dropped as already explained | 8 | count |
| voxel size | 5 | mm |
| overlap fraction that rejects | 0.55 | - |


*per instance* -- 49 rows, full data in `data/scene_rs/26_nested__per_instance.csv`

- **note** FastSAM returns a HIERARCHY, not a partition: for one picture it hands back the cube, the pad, and a region covering both. All three are legitimate. Lifted, they became three objects in the same space and the merge absorbed a 40 mm cube -- measured EXACT at 40.3 mm against a 40 mm truth -- into a 148 mm blob that was then refused as ungraspable. Volume, not pixels: two masks that overlap in the image can be a metre apart in depth.

### 27  The parallel-jaw grasp  [refused]

![grasp](stages/scene_rs/27_grasp.png)

- **refused** the grasp planner refused, which is an ANSWER: object is 1387 mm across its narrowest axis; the Robotiq 85 opens 75 mm usable. It cannot be grasped as it lies.

### 28  Fiducials: the primary detector  [refused]

![apriltag](stages/scene_rs/28_apriltag.png)

- **refused** no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That is an answer, not a failure: a tag either decodes or it does not, which is exactly why this is the PRIMARY detector and colour is the capped fallback. Detector used: cv2.aruco.ArucoDetector (OpenCV 4.11.0).

### 29  The support surface, as an object  [ok]

![table](stages/scene_rs/29_table.png)

- **formula** `fit the plane, keep its inliers, span them in the plane's own basis; extent taken at the 2nd and 98th percentile so one stray return cannot stretch it`
- **source** the plane of stage 18, expressed as a rectangle in the camera frame
- **summary** 1.012 x 1.762 m of surface, 4.352 m away, flat to 3.41 mm

| quantity | value | unit |
| --- | ---: | --- |
| points on the surface | 1475 | count |
| surface extent along its first axis | 1.0115 | m |
| surface extent along its second axis | 1.7624 | m |
| area spanned | 1.7828 | m2 |
| perpendicular distance from the camera | 4.3516 | m |
| distance to the middle of that extent | 4.9018 | m |
| nearest point on the surface | 4.6302 | m |
| furthest point on the surface | 5.589 | m |
| tilt from the optical axis | 15 | deg |
| flatness, RMS of the inliers | 3.405 | mm |

- **note** This is a MEASURED surface, not the declared one. srl_experiments/work_surface.py has had set_measured() since it was written and nothing ever called it, so a table 20 mm out would have been absorbed silently into every grasp.

### 30  Every object, as an oriented 3-D box  [ok]

![boxes3d](stages/scene_rs/30_boxes3d.png)

- **formula** `centre = mid-extent ACROSS the surface + (foot + top/2) ALONG the normal; axes = (minimum-width direction, its perpendicular, the plane normal); size = the spans along them`
- **source** segment_lift._measure for the horizontal centre and rgbd_grasp.shell_corrected_centre's reasoning for the vertical -- the combination is checked to 2 mm against a constructed cube in --self-test
- **summary** 12 objects, 2.609 to 4.567 m away

| quantity | value | unit |
| --- | ---: | --- |
| objects posed | 12 | count |
| nearest object | 2.6092 | m |
| furthest object | 4.5669 | m |
| largest footprint | 2855 | mm |
| tallest object | 4072 | mm |
| graspable within the 85 mm jaw | 0 | count |


*every object, in the camera frame* -- 12 rows, full data in `data/scene_rs/30_boxes3d__every_object__in_the_camera_frame.csv`

- **note** These coordinates are in the CAMERA's frame. Putting them in the robot's frame needs the camera extrinsic, which on the wrist is measured at ~8.5 deg wrong and rotates with the joint -- 34 mm of error at 0.23 m. That is why the pick servos on an error measured HERE rather than planning through that transform.

### 31  How far apart everything is  [ok]

![distances](stages/scene_rs/31_distances.png)

- **formula** `range = |centre| (the camera is the origin); height = n.c + d; separation = |c_i - c_j|; free gap subtracts each object's half-width`
- **source** the poses of stage 30, all in the camera's own frame
- **summary** 10 objects, nearest 2.609 m, closest pair 0.298 m

| quantity | value | unit |
| --- | ---: | --- |
| objects measured | 10 | count |
| nearest object | 2.6092 | m |
| furthest object | 4.5669 | m |
| closest pair | 0.2985 | m |
| tightest free gap | 0 | m |
| what the origin IS | the scene camera across the room | - |


*each object* -- 10 rows, full data in `data/scene_rs/31_distances__each_object.csv`


*between objects* -- 45 rows, full data in `data/scene_rs/31_distances__between_objects.csv`

- **note** On a WRIST camera the origin is the hand, so 'range' is already the distance the arm has to close -- that is the quantity the servo loop nulls. On the scene camera it is the distance from the camera, and turning that into a distance from the ARM needs the scene-camera-to-robot calibration, which this repository does not have measured.

### 32  People in the picture  [ok]

![people](stages/scene_rs/32_people.png)

- **formula** `markerless pose landmarks; each landmark's range read from the depth at its own pixel (median of a 9x9 window)`
- **source** MediaPipe PoseLandmarker (full), num_poses=4, run in .venv_pose as a separate interpreter -- the same model srl_perception/wearer_tracker_node.py uses, 1.3 s
- **summary** 1 person(s)

| quantity | value | unit |
| --- | ---: | --- |
| people detected | 1 | count |
| landmarks per person | 33 | count |
| inference time | 1.29 | s |
| depth attached to landmarks | yes | - |


*body landmarks* -- 8 rows, full data in `data/scene_rs/32_people__body_landmarks.csv`

- **note** This is the wearer-tracking path. The rule the whole safety case rests on: a camera may only ever make the modelled body BIGGER -- fuse() keeps whichever of the tracked and mannequin primitive is CLOSER to the robot, per part, so a body measured further away changes nothing and a lost track falls back rather than shrinking the person.

### 33  The robot in the picture, and what is NOT detected  [ok]

![self_view](stages/scene_rs/33_self_view.png)

- **formula** `{ pixels with 0 < depth < 0.15 m } -- nearer than anything this camera works on`
- **source** REPLAY of RealSense D435i across the room, depth 640x480@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 0 px nearer than 0.15 m

| quantity | value | unit |
| --- | ---: | --- |
| pixels nearer than the gate | 0 | px |
| that fraction | 0 | % |
| nearest return of all | 0.317 | m |
| blobs of near return | 0 | count |
| this camera is on the hand | no | - |
| near edge of this camera's band | 0.15 | m |


*near-field blobs* -- 0 rows, full data in `data/scene_rs/33_self_view__near_field_blobs.csv`

- **note** THE ROBOT ITSELF IS NOT DETECTED IN THE IMAGE, and no stage in this program claims to. Nothing in this repository finds an arm in a picture: the robot's pose comes from its own encoders through the URDF, and drawing it into a camera image needs the camera-to-robot extrinsic -- measured at ~8.5 deg wrong on the wrist, and never measured at all for the scene cameras. On a wrist camera the near-field blobs above are usually the gripper's own fingers entering the bottom of the frame, which is also why the last ~80 mm of a grasp is completed from the last good fix rather than from live vision.

