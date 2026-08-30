# Computer vision stages, 2026-08-30T07:02:58

Written by `scripts/cv_pickpose_visuals.py`. The arms were NOT commanded; this program has no publisher, no service client and no Kortex session.

## The arms

Read from `/real/joint_states` (144 distinct source stamps), compared with the saved ideal pick pose.

| arm | reference | joints read | worst deviation | at the pick pose |
| --- | --- | --- | --- | --- |
| left | `config/pick_pose_ideal_left.txt` | 7 of 7 | 1.12 deg | YES |
| right | `config/pick_pose_ideal_right.txt` | 7 of 7 | 1.19 deg | YES |

<details><summary>left, joint by joint</summary>

| joint | measured (deg) | pick pose (deg) | delta (deg) |
| --- | --- | --- | --- |
| joint_1 | 98.780 | 97.743 | +1.037 |
| joint_2 | 36.406 | 37.529 | -1.123 |
| joint_3 | 134.078 | 134.368 | -0.290 |
| joint_4 | -88.387 | -87.339 | -1.048 |
| joint_5 | -17.950 | -18.818 | +0.869 |
| joint_6 | -28.499 | -28.989 | +0.490 |
| joint_7 | -127.799 | -128.712 | +0.912 |

</details>

<details><summary>right, joint by joint</summary>

| joint | measured (deg) | pick pose (deg) | delta (deg) |
| --- | --- | --- | --- |
| joint_1 | 133.830 | 134.916 | -1.086 |
| joint_2 | -65.382 | -66.576 | +1.194 |
| joint_3 | 27.014 | 25.867 | +1.147 |
| joint_4 | -31.510 | -31.455 | -0.055 |
| joint_5 | 162.857 | 161.696 | +1.160 |
| joint_6 | 72.299 | 71.186 | +1.113 |
| joint_7 | 154.451 | 153.932 | +0.519 |

</details>

## left_gripper

Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42

- intrinsics read from the published camera_info, not assumed: colour cx=620.9 cy=238.3 on a 1280x720 image -- the principal point is not the image centre and assuming it is throws every ray

Contact sheet: `sheet_left_gripper.png`

### 01  The frame, as the camera delivered it  [ok]

![raw_colour](stages/left_gripper/01_raw_colour.png)

- **formula** `none -- this is the input every later stage is a function of`
- **source** Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 1280 x 720 px | focus (variance of Laplacian) 38 | 0.01% of pixels clipped white, 0.11% crushed black

| quantity | value | unit |
| --- | ---: | --- |
| frame width | 1280 | px |
| frame height | 720 | px |
| focus, variance of Laplacian | 38.2 | - |
| pixels clipped white | 0.012 | % |
| pixels crushed black | 0.113 | % |
| mean blue | 148.6 | 0-255 |
| mean green | 144.67 | 0-255 |
| mean red | 149.76 | 0-255 |
| median grey | 153 | 0-255 |
| grey std | 56.81 | 0-255 |

- **note** intrinsics read from the published camera_info, not assumed: colour cx=620.9 cy=238.3 on a 1280x720 image -- the principal point is not the image centre and assuming it is throws every ray

### 02  Intrinsics and the ray each pixel stands for  [ok]

![intrinsics](stages/left_gripper/02_intrinsics.png)

- **formula** `bearing = ((u - cx)/fx, (v - cy)/fy);  X = bearing * Z`
- **source** Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
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
- **source** Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** median H 128  S 17  V 162 | 86.2% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 128 | 0-179 |
| median saturation | 17 | 0-255 |
| median value | 162 | 0-255 |
| pixels below S=80 | 86.175 | % |
| pixels below V=40 | 0.834 | % |
| pixels above V=200 | 25.699 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/left_gripper/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 32359 px selected (3.511% of the frame) | of those, median S 109 and median V 142

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 32359 | px |
| fraction of the frame | 3.5112 | % |
| median S of the selected | 109 | 0-255 |
| median V of the selected | 142 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/left_gripper/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 32359 px in, 32012 px out, 347 px removed (1.1%) | 28 separate blobs before, 1 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 32359 | px |
| pixels after | 32012 | px |
| pixels removed | 347 | px |
| removed | 1.072 | % |
| blobs before | 28 | count |
| blobs after | 1 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/left_gripper/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 1 components over 40 px | largest 32012 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 1 | count |
| largest area | 32012 | px |
| total area kept | 32012 | px |
| median area | 3.201e+04 | px |


*every component* -- 1 rows, full data in `data/left_gripper/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/left_gripper/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** white 238327 | orange 38505 | green 32102 | black 10822 | red 8734 | blue 452

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 8 | count |
| largest term | white | - |
| its pixel count | 238327 | px |


*pixels per colour term* -- 11 rows, full data in `data/left_gripper/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/left_gripper/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 32012 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 32102 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 32102 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

### 08  The region-MEAN ratio test, and its near-black guard  [ok]

![region_mean](stages/left_gripper/08_region_mean.png)

- **formula** `green  <=>  mean_G > 1.25 * mean_B  and  mean_G > 1.8 * mean_R,  refused outright when max(B,G,R) < 25`
- **source** srl_perception/prompt_detector.py:_colour_matches
- **summary** 0 of 1 regions pass

| quantity | value | unit |
| --- | ---: | --- |
| regions tested | 1 | count |
| regions passing | 0 | count |
| regions rejected as near-black | 0 | count |
| blue ratio threshold | 1.25 | - |
| red ratio threshold | 1.8 | - |
| near-black floor | 25 | 0-255 |


*region means and verdicts* -- 1 rows, full data in `data/left_gripper/08_region_mean__region_means_and_verdicts.csv`

- **note** Per-pixel hue picked the room's teal robots three times: their shadowed hue is 76 against the target cube's 74, two apart and inside noise. Region MEANS separate them, because teal carries far more blue. The near-black guard exists because a region measuring (4.1, 8.8, 3.9) -- visually black -- satisfies the ratios on sensor noise alone and was reported as the cube.

### 09  Rotated box, in-image yaw, and the degeneracy gate  [ok]

![minarearect](stages/left_gripper/09_minarearect.png)

- **formula** `minAreaRect -> ((cx,cy), (w,h), angle);  yaw is published as q = (0, 0, sin(angle/2), cos(angle/2)) ONLY when max(w,h)/min(w,h) >= 1.15`
- **source** srl_perception/colour_shape_detector.py
- **summary** 210 x 199 px, 77.5 deg, aspect 1.06, identity (not measured)

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
- **summary** 210 px -> 0.247 m REJECT

| quantity | value | unit |
| --- | ---: | --- |
| fx used | 1298 | px |
| assumed object width | 40 | mm |
| near limit of the window | 0.25 | m |
| far limit of the window | 1.5 | m |
| blobs tested | 1 | count |
| blobs accepted | 0 | count |
| blobs rejected | 1 | count |


*implied range per blob* -- 1 rows, full data in `data/left_gripper/10_size_at_range__implied_range_per_blob.csv`

- **note** This gate was added after a measured failure: T1's coloured pads are 210 x 130 mm in exactly the cube colours, and with a minimum-area floor and no upper bound the detector locked onto the pads and scored 0 of 4 cubes. A 210 mm pad read as a 40 mm cube implies z = 0.134 m -- 'a cube 134 mm from the lens' -- which is outside anything the arm works in, so the pad disappears while a real cube at 0.5 m passes untouched.

### 11  FastSAM: every region in the picture, no prompt  [ok]

![fastsam](stages/left_gripper/11_fastsam.png)

- **formula** `object-agnostic instance masks; no class, no prompt, no scene knowledge -- 'what are the regions', not 'where is the green cube'`
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 5.9 s for this frame
- **summary** 28 regions | largest 437571 px (47.5% of frame), smallest 316 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 28 | count |
| largest region | 437571 | px |
| largest as a fraction of the frame | 47.48 | % |
| smallest region | 316 | px |
| median region | 1.128e+04 | px |
| inference time | 5.93 | s |
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
- **summary** 28 in, 26 kept, 0 too small, 2 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 28 | count |
| kept | 26 | count |
| rejected as noise | 0 | count |
| rejected as too large | 2 | count |
| lower bound | 60 | px |
| upper bound | 322560 | px |
| upper bound as a fraction | 0.35 | - |

- **note** The upper bound is not tidiness. A segmenter returns the table and the room as legitimate regions, and a region covering a third of the frame is never a thing to be picked up.

### 13  Segment everything, then pick the one that matches  [refused]

![segment_match](stages/left_gripper/13_segment_match.png)

- **refused** FastSAM returned 28 regions and NONE of them is green-dominant by the region-mean test. That is a real 'not found' for this prompt in this frame -- the segmenter ran, the gate ran, and nothing matched.

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](stages/left_gripper/14_yoloworld.png)

- **refused** YOLO-World ran in 18.2 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

### 15  The depth frame, and where it has no answer  [ok]

![raw_depth](stages/left_gripper/15_raw_depth.png)

- **formula** `one 16-bit value per pixel, in MILLIMETRES on the wire, divided by 1000 here. 0 means 'no return' and is NOT a distance of zero.`
- **source** Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 480 x 270 | 89.3% of pixels have a return | median 2.226 m, 5th-95th 0.424-5.377 m | 54620 px between the 0.08 and 1.50 m gate the pick path uses

| quantity | value | unit |
| --- | ---: | --- |
| depth width | 480 | px |
| depth height | 270 | px |
| pixels with a return | 115694 | px |
| that fraction | 89.27 | % |
| median range | 2.226 | m |
| 5th percentile | 0.424 | m |
| 95th percentile | 5.377 | m |
| nearest return | 0.362 | m |
| furthest return | 10.949 | m |
| returns inside the working gate | 54620 | px |
| gate near | 0.08 | m |
| gate far | 1.5 | m |

- **note** A hole is not a far surface. Averaging a hole in as 0 drags every edge pixel toward the camera, which is why the RealSense average is taken over VALID pixels only.

### 16  Depth put into the colour frame  [ok]

![alignment](stages/left_gripper/16_alignment.png)

- **formula** `P = ((u-cx_d)z/fx_d, (v-cy_d)z/fy_d, z);  P' = P + t;  u' = fx_c X'/Z' + cx_c,  v' = fy_c Y'/Z' + cy_c`
- **source** forward-projected here: deproject with the depth intrinsics, translate by the recorded -27.1, -10.0, -4.7 mm baseline, project with the colour intrinsics, nearest point wins
- **summary** 66653 points projected, 48678 fell outside the colour frame, 7.2% of colour pixels got a depth

| quantity | value | unit |
| --- | ---: | --- |
| points projected | 66653 | count |
| points falling outside the colour frame | 48678 | count |
| colour pixels given a depth | 7.22 | % |
| baseline x | -27.06 | mm |
| baseline y | -9.97 | mm |
| baseline z | -4.71 | mm |
| rotation assumed | identity | - |

- **note** The rotation between the two sensors is taken as IDENTITY here, because a recorded translation is all this program has. The live pick path uses the URDF FK between camera_depth_frame and camera_color_frame instead -- and that extrinsic is known to be ~8.5 deg wrong and to rotate with the wrist, which is why the pick servos in the camera frame rather than planning through it.

### 17  Pixels and depth become a cloud  [ok]

![deprojection](stages/left_gripper/17_deprojection.png)

- **formula** `X = (u - cx) Z / fx,   Y = (v - cy) Z / fy,   Z = depth`
- **source** rgbd_grasp.deproject / servo_pick_left.measure, with fx 360.01 fy 360.01 cx 243.87 cy 137.92
- **summary** 54620 points in the 0.08-1.50 m band | extent X -0.418 to 0.707, Y -0.067 to 0.365, Z 0.362 to 1.115 m

| quantity | value | unit |
| --- | ---: | --- |
| points lifted | 54620 | count |
| fx | 360.013 | px |
| fy | 360.013 | px |
| cx | 243.872 | px |
| cy | 137.922 | px |
| X minimum | -0.4183 | m |
| X maximum | 0.7065 | m |
| Y minimum | -0.0673 | m |
| Y maximum | 0.3648 | m |
| Z minimum | 0.362 | m |
| Z maximum | 1.115 | m |
| centroid X | 0.0137 | m |
| centroid Y | 0.0818 | m |
| centroid Z | 0.5672 | m |

- **note** This is the whole of 'depth decides WHERE'. Everything after it is geometry on these points, in the CAMERA's frame -- no robot transform is involved yet, which is exactly why the pick measures its error here.

### 18  RANSAC: the support plane  [ok]

![ransac](stages/left_gripper/18_ransac.png)

- **formula** `draw 3 points -> n = (b-a) x (c-a) / |...|,  d = -n.a;  score = #{ |n.P + d| < 6 mm };  keep the best of 400 draws`
- **source** scripts/servo_pick_left.py:fit_plane, imported; fitted on 54620 of 54620 points
- **summary** normal (-0.2487, -0.8830, -0.3982), offset 0.2951 m | 40528 of 54620 points are inliers (74.2%) | RMS 1.41 mm | 66.5 deg between the plane normal and the camera's optical axis

| quantity | value | unit |
| --- | ---: | --- |
| normal x | -0.2487 | - |
| normal y | -0.883 | - |
| normal z | -0.3982 | - |
| offset d | 0.2951 | m |
| points fitted | 54620 | count |
| points scored | 54620 | count |
| inliers | 40528 | count |
| inlier fraction | 74.2 | % |
| residual RMS | 1.413 | mm |
| inlier tolerance | 6 | mm |
| RANSAC iterations | 400 | count |
| angle to the optical axis | 66.54 | deg |

- **note** A plane with too few inliers is not a plane. The pick refuses below 200 inliers rather than returning a geometry whose RMS is NaN -- everything downstream would treat that as a fix and the arm would move on it.

### 19  The PCA refinement, and the NaN it once produced  [ok]

![pca_refine](stages/left_gripper/19_pca_refine.png)

- **formula** `C = cov(Q - mean(Q)) for inliers Q;  n = eigvec(C)[:, argmin];  d = -n . mean(Q)   <-- BOTH halves, together`
- **source** scripts/servo_pick_left.py:fit_plane, imported
- **summary** the normal moved 0.000 deg | RMS 1.41 mm -> 1.41 mm | mixing the refined normal with the RANSAC offset selects 40528 inliers instead of 40528

| quantity | value | unit |
| --- | ---: | --- |
| normal moved | 0 | deg |
| RMS before refinement | 1.413 | mm |
| RMS after refinement | 1.413 | mm |
| inliers with the matched pair | 40528 | count |
| inliers with a mixed normal and offset | 40528 | count |
| inliers lost by mixing | 0 | count |

- **note** That last number is the bug this stage exists to show. The function once returned the REFINED normal with the OLD offset; those describe different planes, the inlier mask could select ZERO points, and mean() of an empty slice is NaN. The pick printed 'plane RMS nan mm' and 'FOUND' in the same breath, then closed 540 mm blind on a centre computed against that plane.

### 20  Height above the plane  [ok]

![height_map](stages/left_gripper/20_height_map.png)

- **formula** `h(P) = n . P + d,  signed;  positive is off the surface, toward the camera`
- **source** servo_pick_left.measure -- the same expression the live pick uses to separate the cube from the table
- **summary** 9582 of 54620 points stand more than 5 mm off the plane (17.54%) | tallest 93.9 mm

| quantity | value | unit |
| --- | ---: | --- |
| height threshold | 5 | mm |
| points above it | 9582 | count |
| that fraction | 17.543 | % |
| tallest point | 93.88 | mm |
| lowest point | -640.27 | mm |
| median height | 0.191 | mm |

- **note** The floor is 5 mm because the plane RMS is under 1 mm on this sensor. Set it below the depth noise and the table itself becomes an object; set it far above and a flat item is missed.

### 21  The surface height as a MODE, not a mean  [ok]

![mode_surface](stages/left_gripper/21_mode_surface.png)

- **formula** `bin at 2 mm; take the fullest bin; require it to hold >= 20% of the points; refine as the mean of everything within +/-6 mm of it`
- **source** srl_perception/surface_from_depth.py, applied along the fitted plane normal because this program has no camera-to-world transform; the node applies it to world z
- **summary** modal bin holds 38.7% of 54620 points (floor 20%) | refined -0.2951 m, spread 1.43 mm | a plain MEAN would say -0.3015 m, which is 6.4 mm out | RANSAC's own offset is -0.2951 m, 0.06 mm from this

| quantity | value | unit |
| --- | ---: | --- |
| bin width | 2 | mm |
| points binned | 54620 | count |
| share in the fullest bin | 38.68 | % |
| share required | 20 | % |
| modal bin centre | -0.2944 | m |
| refined estimate | -0.2951 | m |
| spread of the inliers | 1.431 | mm |
| inlier band | 6 | mm |
| a plain mean would say | -0.3015 | m |
| that mean's error | 6.42 | mm |
| RANSAC's own offset | -0.2951 | m |
| the two estimators differ by | 0.057 | mm |

- **note** Two independent estimators of one surface, so they can be compared: RANSAC votes on inliers, this votes on the fullest histogram bin. A mean is dragged up by everything standing on the table and down by every hole, and it MOVES as the objects move -- the 'measurement' would change between trials on a motionless table.

### 22  What is standing on the surface  [refused]

![on_surface](stages/left_gripper/22_on_surface.png)

- **refused** nothing stands more than 5 mm above the fitted plane in this view

### 23  The measurement the pick actually acts on  [ok]

![cube_measurement](stages/left_gripper/23_cube_measurement.png)

- **formula** `cube = colour(u,v) AND h > 5 mm;  foot = mean(Pk) - n (n.mean(Pk) + d);  centre = foot + n * max(h)/2`
- **source** scripts/servo_pick_left.py:measure -- this is the measurement the servo loop nulls against, in the CAMERA's frame, where the data is good
- **summary** 1115 object points | top 58.4 mm above the plane | centre (0.0709, 0.0470, 0.5193) m in the camera frame | plane RMS 1.41 mm, inlier fraction 0.74

| quantity | value | unit |
| --- | ---: | --- |
| object points | 1115 | count |
| points that are the colour but ON the plane | 1245 | count |
| points off the plane but not the colour | 8467 | count |
| top above the plane | 58.44 | mm |
| median height | 33.45 | mm |
| centre x | 0.0709 | m |
| centre y | 0.047 | m |
| centre z | 0.5193 | m |
| range to the centre | 0.5262 | m |
| plane RMS | 1.413 | mm |
| plane inlier fraction | 0.742 | - |
| minimum points the pick requires | 60 | count |

- **note** Two independent gates, and that is the point: colour alone picks up the green pad the cube stands on and anything green in the room; height alone picks up everything on the table. The centre uses the foot point rather than the cloud's centroid because a depth camera sees a SHELL -- the front surface only -- and a shell's centroid is biased toward the camera, measured at 10.5 mm in the error budget against a 30 mm capture gate.

### 24  Splitting a cube from the pad it stands on  [ok]

![layers](stages/left_gripper/24_layers.png)

- **formula** `find the MODAL height bin (5 mm bins); cut 12 mm above it; recurse on what is left, twice`
- **source** srl_perception/segment_lift.py:_layers
- **summary** layer 1: 4-16 mm, 1407 points | layer 2: 16-43 mm, 2552 points | layer 3: 43-67 mm, 1785 points

| quantity | value | unit |
| --- | ---: | --- |
| region area | 104883 | px |
| points lifted from it | 7391 | count |
| points above the plane | 5744 | count |
| pixels removed by the 1 px erosion | 1788 | px |
| layers found | 3 | count |
| lowest point | 4 | mm |
| highest point | 66.99 | mm |


*height layers* -- 3 rows, full data in `data/left_gripper/24_layers__height_layers.csv`

- **note** Appearance decides what is SIDE BY SIDE; height decides what is STACKED. A blue cube on a blue pad is correctly ONE region -- same colour, touching, no edge between them. And the split cannot look for an empty band, because there is none: the cube RESTS on the pad, so its sides fill every bin. What is there is a MODE -- the support's top face is a large flat area and dominates the histogram.

### 25  Rotating calipers, against the two controls  [ok]

![min_width](stages/left_gripper/25_min_width.png)

- **formula** `width = min over theta of [ max(p.u(theta)) - min(p.u(theta)) ],  u(theta) = (cos, sin) in the support plane, swept at 0.25 deg`
- **source** srl_perception/segment_lift.py:_min_width and rgbd_grasp.min_width_frame
- **summary** calipers 276.7 mm | PCA's short axis 293.3 mm | axis-aligned box 277.8 mm | long axis 360.4 mm, height 63.0 mm | jaws close on 85 mm, so this is TOO WIDE

| quantity | value | unit |
| --- | ---: | --- |
| instance points | 5744 | count |
| minimum width, rotating calipers | 276.68 | mm |
| at this angle in the plane | 58 | deg |
| PCA short axis, THE CONTROL | 293.26 | mm |
| axis-aligned box, THE CONTROL | 277.81 | mm |
| PCA overstates by | 16.59 | mm |
| long axis | 360.38 | mm |
| height | 62.99 | mm |
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

- **refused** the grasp planner refused, which is an ANSWER: object is 142 mm across its narrowest axis; the Robotiq 85 opens 75 mm usable. It cannot be grasped as it lies.

### 28  Fiducials: the primary detector  [refused]

![apriltag](stages/left_gripper/28_apriltag.png)

- **refused** no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That is an answer, not a failure: a tag either decodes or it does not, which is exactly why this is the PRIMARY detector and colour is the capped fallback. Detector used: cv2.aruco.ArucoDetector (OpenCV 4.11.0).

### 29  The support surface, as an object  [ok]

![table](stages/left_gripper/29_table.png)

- **formula** `fit the plane, keep its inliers, span them in the plane's own basis; extent taken at the 2nd and 98th percentile so one stray return cannot stretch it`
- **source** the plane of stage 18, expressed as a rectangle in the camera frame
- **summary** 0.578 x 0.422 m of surface, 0.277 m away, flat to 1.36 mm

| quantity | value | unit |
| --- | ---: | --- |
| points on the surface | 37764 | count |
| surface extent along its first axis | 0.5777 | m |
| surface extent along its second axis | 0.4217 | m |
| area spanned | 0.2436 | m2 |
| perpendicular distance from the camera | 0.2773 | m |
| distance to the middle of that extent | 0.5907 | m |
| nearest point on the surface | 0.3982 | m |
| furthest point on the surface | 0.9227 | m |
| tilt from the optical axis | 66.57 | deg |
| flatness, RMS of the inliers | 1.357 | mm |

- **note** This is a MEASURED surface, not the declared one. srl_experiments/work_surface.py has had set_measured() since it was written and nothing ever called it, so a table 20 mm out would have been absorbed silently into every grasp.

### 30  Every object, as an oriented 3-D box  [ok]

![boxes3d](stages/left_gripper/30_boxes3d.png)

- **formula** `centre = mid-extent ACROSS the surface + (foot + top/2) ALONG the normal; axes = (minimum-width direction, its perpendicular, the plane normal); size = the spans along them`
- **source** segment_lift._measure for the horizontal centre and rgbd_grasp.shell_corrected_centre's reasoning for the vertical -- the combination is checked to 2 mm against a constructed cube in --self-test
- **summary** 5 objects, 0.541 to 0.846 m away

| quantity | value | unit |
| --- | ---: | --- |
| objects posed | 5 | count |
| nearest object | 0.5408 | m |
| furthest object | 0.8465 | m |
| largest footprint | 360.4 | mm |
| tallest object | 67 | mm |
| graspable within the 85 mm jaw | 4 | count |


*every object, in the camera frame* -- 5 rows, full data in `data/left_gripper/30_boxes3d__every_object__in_the_camera_frame.csv`

- **note** These coordinates are in the CAMERA's frame. Putting them in the robot's frame needs the camera extrinsic, which on the wrist is measured at ~8.5 deg wrong and rotates with the joint -- 34 mm of error at 0.23 m. That is why the pick servos on an error measured HERE rather than planning through that transform.

### 31  How far apart everything is  [ok]

![distances](stages/left_gripper/31_distances.png)

- **formula** `range = |centre| (the camera is the origin); height = n.c + d; separation = |c_i - c_j|; free gap subtracts each object's half-width`
- **source** the poses of stage 30, all in the camera's own frame
- **summary** 5 objects, nearest 0.541 m, closest pair 0.093 m

| quantity | value | unit |
| --- | ---: | --- |
| objects measured | 5 | count |
| nearest object | 0.5408 | m |
| furthest object | 0.8465 | m |
| closest pair | 0.0926 | m |
| tightest free gap | 0.0409 | m |
| what the origin IS | the wrist camera, so range is distance from the HAND | - |


*each object* -- 5 rows, full data in `data/left_gripper/31_distances__each_object.csv`


*between objects* -- 10 rows, full data in `data/left_gripper/31_distances__between_objects.csv`

- **note** On a WRIST camera the origin is the hand, so 'range' is already the distance the arm has to close -- that is the quantity the servo loop nulls. On the scene camera it is the distance from the camera, and turning that into a distance from the ARM needs the scene-camera-to-robot calibration, which this repository does not have measured.

### 32  People in the picture  [refused]

![people](stages/left_gripper/32_people.png)

- **refused** no person is in this frame. MediaPipe ran in 16.0 s and found nobody, which is an answer -- the wearer-tracking safety case only ever makes the modelled body BIGGER, so 'nobody seen' falls back to the mannequin rather than to an empty scene.

### 33  The robot in the picture, and what is NOT detected  [ok]

![self_view](stages/left_gripper/33_self_view.png)

- **formula** `{ pixels with 0 < depth < 0.08 m } -- nearer than anything the arm works on`
- **source** Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 0 px nearer than 0.08 m

| quantity | value | unit |
| --- | ---: | --- |
| pixels nearer than the gate | 0 | px |
| that fraction | 0 | % |
| nearest return of all | 0.362 | m |
| blobs of near return | 0 | count |
| this camera is on the hand | yes | - |


*near-field blobs* -- 0 rows, full data in `data/left_gripper/33_self_view__near_field_blobs.csv`

- **note** THE ROBOT ITSELF IS NOT DETECTED IN THE IMAGE, and no stage in this program claims to. Nothing in this repository finds an arm in a picture: the robot's pose comes from its own encoders through the URDF, and drawing it into a camera image needs the camera-to-robot extrinsic -- measured at ~8.5 deg wrong on the wrist, and never measured at all for the scene cameras. On a wrist camera the near-field blobs above are usually the gripper's own fingers entering the bottom of the frame, which is also why the last ~80 mm of a grasp is completed from the last good fix rather than from live vision.

## right_gripper

Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42

- intrinsics read from the published camera_info, not assumed: colour cx=620.9 cy=238.3 on a 1280x720 image -- the principal point is not the image centre and assuming it is throws every ray

Contact sheet: `sheet_right_gripper.png`

### 01  The frame, as the camera delivered it  [ok]

![raw_colour](stages/right_gripper/01_raw_colour.png)

- **formula** `none -- this is the input every later stage is a function of`
- **source** Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 1280 x 720 px | focus (variance of Laplacian) 120 | 5.35% of pixels clipped white, 0.06% crushed black

| quantity | value | unit |
| --- | ---: | --- |
| frame width | 1280 | px |
| frame height | 720 | px |
| focus, variance of Laplacian | 120 | - |
| pixels clipped white | 5.355 | % |
| pixels crushed black | 0.065 | % |
| mean blue | 139.8 | 0-255 |
| mean green | 139.75 | 0-255 |
| mean red | 143.21 | 0-255 |
| median grey | 132 | 0-255 |
| grey std | 70.42 | 0-255 |

- **note** intrinsics read from the published camera_info, not assumed: colour cx=620.9 cy=238.3 on a 1280x720 image -- the principal point is not the image centre and assuming it is throws every ray

### 02  Intrinsics and the ray each pixel stands for  [ok]

![intrinsics](stages/right_gripper/02_intrinsics.png)

- **formula** `bearing = ((u - cx)/fx, (v - cy)/fy);  X = bearing * Z`
- **source** Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
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
- **source** Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** median H 116  S 25  V 150 | 71.7% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 116 | 0-179 |
| median saturation | 25 | 0-255 |
| median value | 150 | 0-255 |
| pixels below S=80 | 71.656 | % |
| pixels below V=40 | 5.369 | % |
| pixels above V=200 | 30.659 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/right_gripper/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 12376 px selected (1.343% of the frame) | of those, median S 139 and median V 127

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 12376 | px |
| fraction of the frame | 1.3429 | % |
| median S of the selected | 139 | 0-255 |
| median V of the selected | 127 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/right_gripper/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 12376 px in, 12258 px out, 118 px removed (1.0%) | 6 separate blobs before, 1 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 12376 | px |
| pixels after | 12258 | px |
| pixels removed | 118 | px |
| removed | 0.953 | % |
| blobs before | 6 | count |
| blobs after | 1 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/right_gripper/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 1 components over 40 px | largest 12258 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 1 | count |
| largest area | 12258 | px |
| total area kept | 12258 | px |
| median area | 1.226e+04 | px |


*every component* -- 1 rows, full data in `data/right_gripper/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/right_gripper/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** white 280424 | orange 76600 | black 58731 | cyan 39501 | blue 24593 | green 12303

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 8 | count |
| largest term | white | - |
| its pixel count | 280424 | px |


*pixels per colour term* -- 11 rows, full data in `data/right_gripper/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/right_gripper/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 12258 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 12303 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 12277 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

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
- **summary** 124 x 116 px, 10.3 deg, aspect 1.07, identity (not measured)

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
- **summary** 124 px -> 0.420 m accept

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
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 5.9 s for this frame
- **summary** 40 regions | largest 247200 px (26.8% of frame), smallest 296 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 40 | count |
| largest region | 247200 | px |
| largest as a fraction of the frame | 26.82 | % |
| smallest region | 296 | px |
| median region | 7602 | px |
| inference time | 5.9 | s |
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
- **summary** 40 in, 40 kept, 0 too small, 0 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 40 | count |
| kept | 40 | count |
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
- **summary** 13100 px at (703, 343), mean BGR [99.8, 148.6, 74.5]

| quantity | value | unit |
| --- | ---: | --- |
| colour word parsed | green | - |
| regions considered | 40 | count |
| regions matched | 1 | count |
| regions rejected on colour | 39 | count |
| best score | 1 | 0-1 |


*matched regions* -- 1 rows, full data in `data/right_gripper/13_segment_match__matched_regions.csv`

- **note** Naming the cube failed at every confidence -- it is ~19 px and dark, and an open-vocabulary detector cannot recognise it as anything. Segmenting it is trivial: 89 regions, exactly one green-dominant, no false positives. The MASK is the other win: a colour threshold gives whatever pixels passed, a segment gives the object's actual extent, which is what a width measurement needs.

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](stages/right_gripper/14_yoloworld.png)

- **refused** YOLO-World ran in 18.0 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

### 15  The depth frame, and where it has no answer  [ok]

![raw_depth](stages/right_gripper/15_raw_depth.png)

- **formula** `one 16-bit value per pixel, in MILLIMETRES on the wire, divided by 1000 here. 0 means 'no return' and is NOT a distance of zero.`
- **source** Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 480 x 270 | 80.2% of pixels have a return | median 1.838 m, 5th-95th 0.340-5.105 m | 42575 px between the 0.08 and 1.50 m gate the pick path uses

| quantity | value | unit |
| --- | ---: | --- |
| depth width | 480 | px |
| depth height | 270 | px |
| pixels with a return | 103926 | px |
| that fraction | 80.19 | % |
| median range | 1.838 | m |
| 5th percentile | 0.34 | m |
| 95th percentile | 5.105 | m |
| nearest return | 0.298 | m |
| furthest return | 6.287 | m |
| returns inside the working gate | 42575 | px |
| gate near | 0.08 | m |
| gate far | 1.5 | m |

- **note** A hole is not a far surface. Averaging a hole in as 0 drags every edge pixel toward the camera, which is why the RealSense average is taken over VALID pixels only.

### 16  Depth put into the colour frame  [ok]

![alignment](stages/right_gripper/16_alignment.png)

- **formula** `P = ((u-cx_d)z/fx_d, (v-cy_d)z/fy_d, z);  P' = P + t;  u' = fx_c X'/Z' + cx_c,  v' = fy_c Y'/Z' + cy_c`
- **source** forward-projected here: deproject with the depth intrinsics, translate by the recorded -27.1, -10.0, -4.7 mm baseline, project with the colour intrinsics, nearest point wins
- **summary** 60119 points projected, 43714 fell outside the colour frame, 6.5% of colour pixels got a depth

| quantity | value | unit |
| --- | ---: | --- |
| points projected | 60119 | count |
| points falling outside the colour frame | 43714 | count |
| colour pixels given a depth | 6.52 | % |
| baseline x | -27.06 | mm |
| baseline y | -9.97 | mm |
| baseline z | -4.71 | mm |
| rotation assumed | identity | - |

- **note** The rotation between the two sensors is taken as IDENTITY here, because a recorded translation is all this program has. The live pick path uses the URDF FK between camera_depth_frame and camera_color_frame instead -- and that extrinsic is known to be ~8.5 deg wrong and to rotate with the wrist, which is why the pick servos in the camera frame rather than planning through it.

### 17  Pixels and depth become a cloud  [ok]

![deprojection](stages/right_gripper/17_deprojection.png)

- **formula** `X = (u - cx) Z / fx,   Y = (v - cy) Z / fy,   Z = depth`
- **source** rgbd_grasp.deproject / servo_pick_left.measure, with fx 360.01 fy 360.01 cx 243.87 cy 137.92
- **summary** 42575 points in the 0.08-1.50 m band | extent X -0.283 to 0.442, Y -0.183 to 0.180, Z 0.298 to 0.925 m

| quantity | value | unit |
| --- | ---: | --- |
| points lifted | 42575 | count |
| fx | 360.013 | px |
| fy | 360.013 | px |
| cx | 243.872 | px |
| cy | 137.922 | px |
| X minimum | -0.2834 | m |
| X maximum | 0.4422 | m |
| Y minimum | -0.1827 | m |
| Y maximum | 0.1799 | m |
| Z minimum | 0.298 | m |
| Z maximum | 0.925 | m |
| centroid X | 0.0281 | m |
| centroid Y | 0.0911 | m |
| centroid Z | 0.5414 | m |

- **note** This is the whole of 'depth decides WHERE'. Everything after it is geometry on these points, in the CAMERA's frame -- no robot transform is involved yet, which is exactly why the pick measures its error here.

### 18  RANSAC: the support plane  [ok]

![ransac](stages/right_gripper/18_ransac.png)

- **formula** `draw 3 points -> n = (b-a) x (c-a) / |...|,  d = -n.a;  score = #{ |n.P + d| < 6 mm };  keep the best of 400 draws`
- **source** scripts/servo_pick_left.py:fit_plane, imported; fitted on 42575 of 42575 points
- **summary** normal (0.1263, -0.9408, -0.3147), offset 0.2982 m | 22667 of 42575 points are inliers (53.2%) | RMS 1.60 mm | 71.7 deg between the plane normal and the camera's optical axis

| quantity | value | unit |
| --- | ---: | --- |
| normal x | 0.1263 | - |
| normal y | -0.9408 | - |
| normal z | -0.3147 | - |
| offset d | 0.2982 | m |
| points fitted | 42575 | count |
| points scored | 42575 | count |
| inliers | 22667 | count |
| inlier fraction | 53.24 | % |
| residual RMS | 1.604 | mm |
| inlier tolerance | 6 | mm |
| RANSAC iterations | 400 | count |
| angle to the optical axis | 71.66 | deg |

- **note** A plane with too few inliers is not a plane. The pick refuses below 200 inliers rather than returning a geometry whose RMS is NaN -- everything downstream would treat that as a fix and the arm would move on it.

### 19  The PCA refinement, and the NaN it once produced  [ok]

![pca_refine](stages/right_gripper/19_pca_refine.png)

- **formula** `C = cov(Q - mean(Q)) for inliers Q;  n = eigvec(C)[:, argmin];  d = -n . mean(Q)   <-- BOTH halves, together`
- **source** scripts/servo_pick_left.py:fit_plane, imported
- **summary** the normal moved 0.000 deg | RMS 1.60 mm -> 1.60 mm | mixing the refined normal with the RANSAC offset selects 22667 inliers instead of 22667

| quantity | value | unit |
| --- | ---: | --- |
| normal moved | 0 | deg |
| RMS before refinement | 1.604 | mm |
| RMS after refinement | 1.604 | mm |
| inliers with the matched pair | 22667 | count |
| inliers with a mixed normal and offset | 22667 | count |
| inliers lost by mixing | 0 | count |

- **note** That last number is the bug this stage exists to show. The function once returned the REFINED normal with the OLD offset; those describe different planes, the inlier mask could select ZERO points, and mean() of an empty slice is NaN. The pick printed 'plane RMS nan mm' and 'FOUND' in the same breath, then closed 540 mm blind on a centre computed against that plane.

### 20  Height above the plane  [ok]

![height_map](stages/right_gripper/20_height_map.png)

- **formula** `h(P) = n . P + d,  signed;  positive is off the surface, toward the camera`
- **source** servo_pick_left.measure -- the same expression the live pick uses to separate the cube from the table
- **summary** 19936 of 42575 points stand more than 5 mm off the plane (46.83%) | tallest 357.8 mm

| quantity | value | unit |
| --- | ---: | --- |
| height threshold | 5 | mm |
| points above it | 19936 | count |
| that fraction | 46.826 | % |
| tallest point | 357.78 | mm |
| lowest point | -16.42 | mm |
| median height | 1.999 | mm |

- **note** The floor is 5 mm because the plane RMS is under 1 mm on this sensor. Set it below the depth noise and the table itself becomes an object; set it far above and a flat item is missed.

### 21  The surface height as a MODE, not a mean  [ok]

![mode_surface](stages/right_gripper/21_mode_surface.png)

- **formula** `bin at 2 mm; take the fullest bin; require it to hold >= 20% of the points; refine as the mean of everything within +/-6 mm of it`
- **source** srl_perception/surface_from_depth.py, applied along the fitted plane normal because this program has no camera-to-world transform; the node applies it to world z
- **summary** modal bin holds 27.6% of 42575 points (floor 20%) | refined -0.2982 m, spread 1.65 mm | a plain MEAN would say -0.2525 m, which is 45.7 mm out | RANSAC's own offset is -0.2982 m, 0.06 mm from this

| quantity | value | unit |
| --- | ---: | --- |
| bin width | 2 | mm |
| points binned | 42575 | count |
| share in the fullest bin | 27.55 | % |
| share required | 20 | % |
| modal bin centre | -0.2976 | m |
| refined estimate | -0.2982 | m |
| spread of the inliers | 1.647 | mm |
| inlier band | 6 | mm |
| a plain mean would say | -0.2525 | m |
| that mean's error | 45.71 | mm |
| RANSAC's own offset | -0.2982 | m |
| the two estimators differ by | 0.059 | mm |

- **note** Two independent estimators of one surface, so they can be compared: RANSAC votes on inliers, this votes on the fullest histogram bin. A mean is dragged up by everything standing on the table and down by every hole, and it MOVES as the objects move -- the 'measurement' would change between trials on a motionless table.

### 22  What is standing on the surface  [refused]

![on_surface](stages/right_gripper/22_on_surface.png)

- **refused** nothing stands more than 5 mm above the fitted plane in this view

### 23  The measurement the pick actually acts on  [ok]

![cube_measurement](stages/right_gripper/23_cube_measurement.png)

- **formula** `cube = colour(u,v) AND h > 5 mm;  foot = mean(Pk) - n (n.mean(Pk) + d);  centre = foot + n * max(h)/2`
- **source** scripts/servo_pick_left.py:measure -- this is the measurement the servo loop nulls against, in the CAMERA's frame, where the data is good
- **summary** 155 object points | top 56.4 mm above the plane | centre (0.0503, 0.0521, 0.7222) m in the camera frame | plane RMS 1.60 mm, inlier fraction 0.53

| quantity | value | unit |
| --- | ---: | --- |
| object points | 155 | count |
| points that are the colour but ON the plane | 717 | count |
| points off the plane but not the colour | 19781 | count |
| top above the plane | 56.43 | mm |
| median height | 20.8 | mm |
| centre x | 0.0503 | m |
| centre y | 0.0521 | m |
| centre z | 0.7222 | m |
| range to the centre | 0.7258 | m |
| plane RMS | 1.604 | mm |
| plane inlier fraction | 0.5324 | - |
| minimum points the pick requires | 60 | count |

- **note** Two independent gates, and that is the point: colour alone picks up the green pad the cube stands on and anything green in the room; height alone picks up everything on the table. The centre uses the foot point rather than the cloud's centroid because a depth camera sees a SHELL -- the front surface only -- and a shell's centroid is biased toward the camera, measured at 10.5 mm in the error budget against a 30 mm capture gate.

### 24  Splitting a cube from the pad it stands on  [ok]

![layers](stages/right_gripper/24_layers.png)

- **formula** `find the MODAL height bin (5 mm bins); cut 12 mm above it; recurse on what is left, twice`
- **source** srl_perception/segment_lift.py:_layers
- **summary** layer 1: 4-41 mm, 5048 points | layer 2: 41-53 mm, 1621 points | layer 3: 53-64 mm, 992 points

| quantity | value | unit |
| --- | ---: | --- |
| region area | 247200 | px |
| points lifted from it | 16669 | count |
| points above the plane | 7661 | count |
| pixels removed by the 1 px erosion | 3317 | px |
| layers found | 3 | count |
| lowest point | 4.02 | mm |
| highest point | 63.7 | mm |


*height layers* -- 3 rows, full data in `data/right_gripper/24_layers__height_layers.csv`

- **note** Appearance decides what is SIDE BY SIDE; height decides what is STACKED. A blue cube on a blue pad is correctly ONE region -- same colour, touching, no edge between them. And the split cannot look for an empty band, because there is none: the cube RESTS on the pad, so its sides fill every bin. What is there is a MODE -- the support's top face is a large flat area and dominates the histogram.

### 25  Rotating calipers, against the two controls  [ok]

![min_width](stages/right_gripper/25_min_width.png)

- **formula** `width = min over theta of [ max(p.u(theta)) - min(p.u(theta)) ],  u(theta) = (cos, sin) in the support plane, swept at 0.25 deg`
- **source** srl_perception/segment_lift.py:_min_width and rgbd_grasp.min_width_frame
- **summary** calipers 190.7 mm | PCA's short axis 225.7 mm | axis-aligned box 238.2 mm | long axis 399.4 mm, height 59.7 mm | jaws close on 85 mm, so this is TOO WIDE

| quantity | value | unit |
| --- | ---: | --- |
| instance points | 7661 | count |
| minimum width, rotating calipers | 190.72 | mm |
| at this angle in the plane | 170.5 | deg |
| PCA short axis, THE CONTROL | 225.73 | mm |
| axis-aligned box, THE CONTROL | 238.2 | mm |
| PCA overstates by | 35.01 | mm |
| long axis | 399.44 | mm |
| height | 59.69 | mm |
| jaw span | 85 | mm |
| graspable as it lies | no | - |
| sweep step | 0.5 | deg |

- **note** The two controls are here because both were used and both were wrong. The principal axes of a SQUARE are degenerate -- equal variance in both directions -- so SVD returns the diagonal and a 40 mm cube measures 40*sqrt(2) = 53 mm; against an 85 mm jaw that refuses a 60 mm object as ungraspable. The error that matters is not the 13 mm, it is the DIRECTION: a jaw told to close across a diagonal approaches the corner and the object rolls out.

### 26  Dropping masks that are unions of finer ones  [ok]

![nested](stages/right_gripper/26_nested.png)

- **formula** `voxelise each instance at 5 mm; accept smallest first; reject one whose claimed-voxel overlap exceeds 0.55`
- **source** srl_perception/segment_lift.py:_suppress_nested
- **summary** 21 instances in, 18 kept, 3 dropped | overlaps: 0.96, 1.00, 0.79

| quantity | value | unit |
| --- | ---: | --- |
| instances in | 21 | count |
| kept | 18 | count |
| dropped as already explained | 3 | count |
| voxel size | 5 | mm |
| overlap fraction that rejects | 0.55 | - |


*per instance* -- 21 rows, full data in `data/right_gripper/26_nested__per_instance.csv`

- **note** FastSAM returns a HIERARCHY, not a partition: for one picture it hands back the cube, the pad, and a region covering both. All three are legitimate. Lifted, they became three objects in the same space and the merge absorbed a 40 mm cube -- measured EXACT at 40.3 mm against a 40 mm truth -- into a 148 mm blob that was then refused as ungraspable. Volume, not pixels: two masks that overlap in the image can be a metre apart in depth.

### 27  The parallel-jaw grasp  [refused]

![grasp](stages/right_gripper/27_grasp.png)

- **refused** the grasp planner refused, which is an ANSWER: object is 114 mm across its narrowest axis; the Robotiq 85 opens 75 mm usable. It cannot be grasped as it lies.

### 28  Fiducials: the primary detector  [refused]

![apriltag](stages/right_gripper/28_apriltag.png)

- **refused** no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That is an answer, not a failure: a tag either decodes or it does not, which is exactly why this is the PRIMARY detector and colour is the capped fallback. Detector used: cv2.aruco.ArucoDetector (OpenCV 4.11.0).

### 29  The support surface, as an object  [ok]

![table](stages/right_gripper/29_table.png)

- **formula** `fit the plane, keep its inliers, span them in the plane's own basis; extent taken at the 2nd and 98th percentile so one stray return cannot stretch it`
- **source** the plane of stage 18, expressed as a rectangle in the camera frame
- **summary** 0.432 x 0.464 m of surface, 0.290 m away, flat to 1.66 mm

| quantity | value | unit |
| --- | ---: | --- |
| points on the surface | 19655 | count |
| surface extent along its first axis | 0.4315 | m |
| surface extent along its second axis | 0.4637 | m |
| area spanned | 0.2001 | m2 |
| perpendicular distance from the camera | 0.2902 | m |
| distance to the middle of that extent | 0.6661 | m |
| nearest point on the surface | 0.4646 | m |
| furthest point on the surface | 0.935 | m |
| tilt from the optical axis | 71.69 | deg |
| flatness, RMS of the inliers | 1.664 | mm |

- **note** This is a MEASURED surface, not the declared one. srl_experiments/work_surface.py has had set_measured() since it was written and nothing ever called it, so a table 20 mm out would have been absorbed silently into every grasp.

### 30  Every object, as an oriented 3-D box  [ok]

![boxes3d](stages/right_gripper/30_boxes3d.png)

- **formula** `centre = mid-extent ACROSS the surface + (foot + top/2) ALONG the normal; axes = (minimum-width direction, its perpendicular, the plane normal); size = the spans along them`
- **source** segment_lift._measure for the horizontal centre and rgbd_grasp.shell_corrected_centre's reasoning for the vertical -- the combination is checked to 2 mm against a constructed cube in --self-test
- **summary** 12 objects, 0.396 to 0.902 m away

| quantity | value | unit |
| --- | ---: | --- |
| objects posed | 12 | count |
| nearest object | 0.3959 | m |
| furthest object | 0.9021 | m |
| largest footprint | 600.8 | mm |
| tallest object | 221.1 | mm |
| graspable within the 85 mm jaw | 8 | count |


*every object, in the camera frame* -- 12 rows, full data in `data/right_gripper/30_boxes3d__every_object__in_the_camera_frame.csv`

- **note** These coordinates are in the CAMERA's frame. Putting them in the robot's frame needs the camera extrinsic, which on the wrist is measured at ~8.5 deg wrong and rotates with the joint -- 34 mm of error at 0.23 m. That is why the pick servos on an error measured HERE rather than planning through that transform.

### 31  How far apart everything is  [ok]

![distances](stages/right_gripper/31_distances.png)

- **formula** `range = |centre| (the camera is the origin); height = n.c + d; separation = |c_i - c_j|; free gap subtracts each object's half-width`
- **source** the poses of stage 30, all in the camera's own frame
- **summary** 10 objects, nearest 0.397 m, closest pair 0.003 m

| quantity | value | unit |
| --- | ---: | --- |
| objects measured | 10 | count |
| nearest object | 0.3974 | m |
| furthest object | 0.9021 | m |
| closest pair | 0.0025 | m |
| tightest free gap | 0 | m |
| what the origin IS | the wrist camera, so range is distance from the HAND | - |


*each object* -- 10 rows, full data in `data/right_gripper/31_distances__each_object.csv`


*between objects* -- 45 rows, full data in `data/right_gripper/31_distances__between_objects.csv`

- **note** On a WRIST camera the origin is the hand, so 'range' is already the distance the arm has to close -- that is the quantity the servo loop nulls. On the scene camera it is the distance from the camera, and turning that into a distance from the ARM needs the scene-camera-to-robot calibration, which this repository does not have measured.

### 32  People in the picture  [refused]

![people](stages/right_gripper/32_people.png)

- **refused** no person is in this frame. MediaPipe ran in 6.4 s and found nobody, which is an answer -- the wearer-tracking safety case only ever makes the modelled body BIGGER, so 'nobody seen' falls back to the mannequin rather than to an empty scene.

### 33  The robot in the picture, and what is NOT detected  [ok]

![self_view](stages/right_gripper/33_self_view.png)

- **formula** `{ pixels with 0 < depth < 0.08 m } -- nearer than anything the arm works on`
- **source** Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 0 px nearer than 0.08 m

| quantity | value | unit |
| --- | ---: | --- |
| pixels nearer than the gate | 0 | px |
| that fraction | 0 | % |
| nearest return of all | 0.298 | m |
| blobs of near return | 0 | count |
| this camera is on the hand | yes | - |


*near-field blobs* -- 0 rows, full data in `data/right_gripper/33_self_view__near_field_blobs.csv`

- **note** THE ROBOT ITSELF IS NOT DETECTED IN THE IMAGE, and no stage in this program claims to. Nothing in this repository finds an arm in a picture: the robot's pose comes from its own encoders through the URDF, and drawing it into a camera image needs the camera-to-robot extrinsic -- measured at ~8.5 deg wrong on the wrist, and never measured at all for the scene cameras. On a wrist camera the near-field blobs above are usually the gripper's own fingers entering the bottom of the frame, which is also why the last ~80 mm of a grasp is completed from the last good fix rather than from live vision.

## scene_rs

RealSense D435i across the room, depth 480x270@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.

- rotated 180 deg: the camera is mounted upside down and the principal point is rotated WITH the image (cx'=W-1-cx)
- liveness: 12 frames, 12 distinct depth images, 12 distinct frame numbers, 28% of pixels valid

Contact sheet: `sheet_scene_rs.png`

### 01  The frame, as the camera delivered it  [ok]

![raw_colour](stages/scene_rs/01_raw_colour.png)

- **formula** `none -- this is the input every later stage is a function of`
- **source** RealSense D435i across the room, depth 480x270@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 640 x 480 px | focus (variance of Laplacian) 428 | 0.21% of pixels clipped white, 0.00% crushed black

| quantity | value | unit |
| --- | ---: | --- |
| frame width | 640 | px |
| frame height | 480 | px |
| focus, variance of Laplacian | 427.8 | - |
| pixels clipped white | 0.208 | % |
| pixels crushed black | 0 | % |
| mean blue | 107.04 | 0-255 |
| mean green | 115.92 | 0-255 |
| mean red | 101.81 | 0-255 |
| median grey | 117 | 0-255 |
| grey std | 65.62 | 0-255 |

- **note** rotated 180 deg: the camera is mounted upside down and the principal point is rotated WITH the image (cx'=W-1-cx); liveness: 12 frames, 12 distinct depth images, 12 distinct frame numbers, 28% of pixels valid

### 02  Intrinsics and the ray each pixel stands for  [ok]

![intrinsics](stages/scene_rs/02_intrinsics.png)

- **formula** `bearing = ((u - cx)/fx, (v - cy)/fy);  X = bearing * Z`
- **source** RealSense D435i across the room, depth 480x270@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** fx 603.02  fy 603.13  cx 320.13  cy 247.78 | field of view 55.9 x 43.4 deg | principal point is 7.8 px from the image centre, which is 13 mm of sideways error at 1 m if you assume the centre

| quantity | value | unit |
| --- | ---: | --- |
| fx | 603.0215 | px |
| fy | 603.1288 | px |
| cx | 320.1252 | px |
| cy | 247.7821 | px |
| image centre u | 320 | px |
| image centre v | 240 | px |
| principal point offset from centre | 7.78 | px |
| that offset at 1 m range | 12.9 | mm |
| horizontal field of view | 55.906 | deg |
| vertical field of view | 43.398 | deg |
| ground sample distance at 1 m | 1.6583 | mm/px |


### 03  BGR to HSV, the space the colour tests live in  [ok]

![hsv](stages/scene_rs/03_hsv.png)

- **formula** `V = max(B,G,R);  S = (V - min(B,G,R))/V;  H = the sector of the RGB hexagon, 0..179 in OpenCV (not 0..359)`
- **source** RealSense D435i across the room, depth 480x270@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** median H 71  S 35  V 126 | 85.4% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 71 | 0-179 |
| median saturation | 35 | 0-255 |
| median value | 126 | 0-255 |
| pixels below S=80 | 85.432 | % |
| pixels below V=40 | 16.223 | % |
| pixels above V=200 | 9.992 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/scene_rs/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 1733 px selected (0.564% of the frame) | of those, median S 89 and median V 46

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 1733 | px |
| fraction of the frame | 0.5641 | % |
| median S of the selected | 89 | 0-255 |
| median V of the selected | 46 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/scene_rs/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 1733 px in, 187 px out, 1546 px removed (89.2%) | 234 separate blobs before, 1 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 1733 | px |
| pixels after | 187 | px |
| pixels removed | 1546 | px |
| removed | 89.209 | % |
| blobs before | 234 | count |
| blobs after | 1 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/scene_rs/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** RealSense D435i across the room, depth 480x270@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 1 components over 40 px | largest 187 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 1 | count |
| largest area | 187 | px |
| total area kept | 187 | px |
| median area | 187 | px |


*every component* -- 1 rows, full data in `data/scene_rs/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/scene_rs/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** black 55580 | white 30277 | cyan 10802 | orange 4798 | green 2780 | red 0

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 5 | count |
| largest term | black | - |
| its pixel count | 55580 | px |


*pixels per colour term* -- 11 rows, full data in `data/scene_rs/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/scene_rs/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 187 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 2780 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 302 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

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
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 6.0 s for this frame
- **summary** 83 regions | largest 74681 px (24.3% of frame), smallest 77 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 83 | count |
| largest region | 74681 | px |
| largest as a fraction of the frame | 24.31 | % |
| smallest region | 77 | px |
| median region | 596 | px |
| inference time | 5.96 | s |
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
- **summary** 83 in, 83 kept, 0 too small, 0 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 83 | count |
| kept | 83 | count |
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
- **summary** 179 px at (320, 196), mean BGR [78.8, 126.4, 31.8]

| quantity | value | unit |
| --- | ---: | --- |
| colour word parsed | green | - |
| regions considered | 83 | count |
| regions matched | 1 | count |
| regions rejected on colour | 82 | count |
| best score | 0.089 | 0-1 |


*matched regions* -- 1 rows, full data in `data/scene_rs/13_segment_match__matched_regions.csv`

- **note** Naming the cube failed at every confidence -- it is ~19 px and dark, and an open-vocabulary detector cannot recognise it as anything. Segmenting it is trivial: 89 regions, exactly one green-dominant, no false positives. The MASK is the other win: a colour threshold gives whatever pixels passed, a segment gives the object's actual extent, which is what a width measurement needs.

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](stages/scene_rs/14_yoloworld.png)

- **refused** YOLO-World ran in 35.6 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

### 15  The depth frame, and where it has no answer  [ok]

![raw_depth](stages/scene_rs/15_raw_depth.png)

- **formula** `one 16-bit value per pixel, in MILLIMETRES on the wire, divided by 1000 here. 0 means 'no return' and is NOT a distance of zero.`
- **source** RealSense D435i across the room, depth 480x270@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 640 x 480 | 27.7% of pixels have a return | median 3.830 m, 5th-95th 2.997-12.689 m | 12 px between the 0.08 and 1.50 m gate the pick path uses

| quantity | value | unit |
| --- | ---: | --- |
| depth width | 640 | px |
| depth height | 480 | px |
| pixels with a return | 85137 | px |
| that fraction | 27.71 | % |
| median range | 3.8297 | m |
| 5th percentile | 2.997 | m |
| 95th percentile | 12.689 | m |
| nearest return | 0.741 | m |
| furthest return | 65.535 | m |
| returns inside the working gate | 12 | px |
| gate near | 0.08 | m |
| gate far | 1.5 | m |

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

### 17  Pixels and depth become a cloud  [refused]

![deprojection](stages/scene_rs/17_deprojection.png)

- **refused** only 12 depth returns fall between 0.08 and 1.50 m. The camera is not looking at anything in its working band, or the depth stream is delivering holes.

### 18  RANSAC: the support plane  [refused]

![ransac](stages/scene_rs/18_ransac.png)

- **refused** only 12 depth returns fall between 0.08 and 1.50 m. The camera is not looking at anything in its working band, or the depth stream is delivering holes.

### 19  The PCA refinement, and the NaN it once produced  [refused]

![pca_refine](stages/scene_rs/19_pca_refine.png)

- **refused** only 12 depth returns fall between 0.08 and 1.50 m. The camera is not looking at anything in its working band, or the depth stream is delivering holes.

### 20  Height above the plane  [refused]

![height_map](stages/scene_rs/20_height_map.png)

- **refused** only 12 depth returns fall between 0.08 and 1.50 m. The camera is not looking at anything in its working band, or the depth stream is delivering holes.

### 21  The surface height as a MODE, not a mean  [refused]

![mode_surface](stages/scene_rs/21_mode_surface.png)

- **refused** only 12 depth returns fall between 0.08 and 1.50 m. The camera is not looking at anything in its working band, or the depth stream is delivering holes.

### 22  What is standing on the surface  [refused]

![on_surface](stages/scene_rs/22_on_surface.png)

- **refused** only 12 aligned depth points; nothing to lift the masks through

### 23  The measurement the pick actually acts on  [refused]

![cube_measurement](stages/scene_rs/23_cube_measurement.png)

- **refused** only 12 depth returns fall between 0.08 and 1.50 m. The camera is not looking at anything in its working band, or the depth stream is delivering holes.

### 24  Splitting a cube from the pad it stands on  [refused]

![layers](stages/scene_rs/24_layers.png)

- **refused** only 12 aligned depth points; nothing to lift the masks through

### 25  Rotating calipers, against the two controls  [refused]

![min_width](stages/scene_rs/25_min_width.png)

- **refused** only 12 aligned depth points; nothing to lift the masks through

### 26  Dropping masks that are unions of finer ones  [refused]

![nested](stages/scene_rs/26_nested.png)

- **refused** only 12 aligned depth points; nothing to lift the masks through

### 27  The parallel-jaw grasp  [refused]

![grasp](stages/scene_rs/27_grasp.png)

- **refused** only 12 aligned depth points; nothing to lift the masks through

### 28  Fiducials: the primary detector  [refused]

![apriltag](stages/scene_rs/28_apriltag.png)

- **refused** no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That is an answer, not a failure: a tag either decodes or it does not, which is exactly why this is the PRIMARY detector and colour is the capped fallback. Detector used: cv2.aruco.ArucoDetector (OpenCV 4.11.0).

### 29  The support surface, as an object  [refused]

![table](stages/scene_rs/29_table.png)

- **refused** only 12 aligned depth points; nothing to lift the masks through

### 30  Every object, as an oriented 3-D box  [refused]

![boxes3d](stages/scene_rs/30_boxes3d.png)

- **refused** only 12 aligned depth points; nothing to lift the masks through

### 31  How far apart everything is  [refused]

![distances](stages/scene_rs/31_distances.png)

- **refused** only 12 aligned depth points; nothing to lift the masks through

### 32  People in the picture  [ok]

![people](stages/scene_rs/32_people.png)

- **formula** `markerless pose landmarks; each landmark's range read from the depth at its own pixel (median of a 9x9 window)`
- **source** MediaPipe PoseLandmarker (full), num_poses=4, run in .venv_pose as a separate interpreter -- the same model srl_perception/wearer_tracker_node.py uses, 4.6 s
- **summary** 1 person(s)

| quantity | value | unit |
| --- | ---: | --- |
| people detected | 1 | count |
| landmarks per person | 33 | count |
| inference time | 4.61 | s |
| depth attached to landmarks | yes | - |


*body landmarks* -- 8 rows, full data in `data/scene_rs/32_people__body_landmarks.csv`

- **note** This is the wearer-tracking path. The rule the whole safety case rests on: a camera may only ever make the modelled body BIGGER -- fuse() keeps whichever of the tracked and mannequin primitive is CLOSER to the robot, per part, so a body measured further away changes nothing and a lost track falls back rather than shrinking the person.

### 33  The robot in the picture, and what is NOT detected  [ok]

![self_view](stages/scene_rs/33_self_view.png)

- **formula** `{ pixels with 0 < depth < 0.08 m } -- nearer than anything the arm works on`
- **source** RealSense D435i across the room, depth 480x270@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 0 px nearer than 0.08 m

| quantity | value | unit |
| --- | ---: | --- |
| pixels nearer than the gate | 0 | px |
| that fraction | 0 | % |
| nearest return of all | 0.741 | m |
| blobs of near return | 0 | count |
| this camera is on the hand | no | - |


*near-field blobs* -- 0 rows, full data in `data/scene_rs/33_self_view__near_field_blobs.csv`

- **note** THE ROBOT ITSELF IS NOT DETECTED IN THE IMAGE, and no stage in this program claims to. Nothing in this repository finds an arm in a picture: the robot's pose comes from its own encoders through the URDF, and drawing it into a camera image needs the camera-to-robot extrinsic -- measured at ~8.5 deg wrong on the wrist, and never measured at all for the scene cameras. On a wrist camera the near-field blobs above are usually the gripper's own fingers entering the bottom of the frame, which is also why the last ~80 mm of a grasp is completed from the last good fix rather than from live vision.

## scene_hd

HD USB webcam through scene_camera_node's own topic /scene_camera/image_raw on ROS_DOMAIN_ID=42 -- the node holds /dev/video*, and V4L2 allows one client

- NO INTRINSICS: the node published a zero camera_info, which it does deliberately until it is calibrated. Stages needing fx, fy, cx, cy refuse rather than assume cx = w/2.

Contact sheet: `sheet_scene_hd.png`

### 01  The frame, as the camera delivered it  [ok]

![raw_colour](stages/scene_hd/01_raw_colour.png)

- **formula** `none -- this is the input every later stage is a function of`
- **source** HD USB webcam through scene_camera_node's own topic /scene_camera/image_raw on ROS_DOMAIN_ID=42 -- the node holds /dev/video*, and V4L2 allows one client
- **summary** 640 x 480 px | focus (variance of Laplacian) 1690 | 10.66% of pixels clipped white, 2.06% crushed black

| quantity | value | unit |
| --- | ---: | --- |
| frame width | 640 | px |
| frame height | 480 | px |
| focus, variance of Laplacian | 1690 | - |
| pixels clipped white | 10.659 | % |
| pixels crushed black | 2.059 | % |
| mean blue | 139.86 | 0-255 |
| mean green | 140.06 | 0-255 |
| mean red | 138 | 0-255 |
| median grey | 166 | 0-255 |
| grey std | 90.45 | 0-255 |

- **note** NO INTRINSICS: the node published a zero camera_info, which it does deliberately until it is calibrated. Stages needing fx, fy, cx, cy refuse rather than assume cx = w/2.

### 02  Intrinsics and the ray each pixel stands for  [refused]

![intrinsics](stages/scene_hd/02_intrinsics.png)

- **refused** this camera has no intrinsics in this repository, and this stage is arithmetic on fx, fy, cx, cy. Assuming cx = w/2 is exactly the error that throws a ray by 122 px on the Kinova module -- 94 mm at 1 m, three times the grasp tolerance. Calibrate it: scripts/calibrate_scene_camera.py

### 03  BGR to HSV, the space the colour tests live in  [ok]

![hsv](stages/scene_hd/03_hsv.png)

- **formula** `V = max(B,G,R);  S = (V - min(B,G,R))/V;  H = the sector of the RGB hexagon, 0..179 in OpenCV (not 0..359)`
- **source** HD USB webcam through scene_camera_node's own topic /scene_camera/image_raw on ROS_DOMAIN_ID=42 -- the node holds /dev/video*, and V4L2 allows one client
- **summary** median H 83  S 6  V 170 | 83.2% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 83 | 0-179 |
| median saturation | 6 | 0-255 |
| median value | 170 | 0-255 |
| pixels below S=80 | 83.236 | % |
| pixels below V=40 | 20.806 | % |
| pixels above V=200 | 36.761 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/scene_hd/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 280 px selected (0.091% of the frame) | of those, median S 132 and median V 139

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 280 | px |
| fraction of the frame | 0.0911 | % |
| median S of the selected | 132 | 0-255 |
| median V of the selected | 139 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/scene_hd/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 280 px in, 218 px out, 62 px removed (22.1%) | 14 separate blobs before, 1 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 280 | px |
| pixels after | 218 | px |
| pixels removed | 62 | px |
| removed | 22.143 | % |
| blobs before | 14 | count |
| blobs after | 1 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/scene_hd/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** HD USB webcam through scene_camera_node's own topic /scene_camera/image_raw on ROS_DOMAIN_ID=42 -- the node holds /dev/video*, and V4L2 allows one client
- **summary** 1 components over 40 px | largest 218 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 1 | count |
| largest area | 218 | px |
| total area kept | 218 | px |
| median area | 218 | px |


*every component* -- 1 rows, full data in `data/scene_hd/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/scene_hd/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** white 110628 | black 57492 | red 3630 | cyan 2637 | green 218 | blue 27

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 6 | count |
| largest term | white | - |
| its pixel count | 110628 | px |


*pixels per colour term* -- 11 rows, full data in `data/scene_hd/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/scene_hd/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 218 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 218 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 218 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

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

### 09  Rotated box, in-image yaw, and the degeneracy gate  [refused]

![minarearect](stages/scene_hd/09_minarearect.png)

- **refused** no contour over the 400 px floor in this frame, so there is no rotated box to fit

### 10  The size-at-range gate  [refused]

![size_at_range](stages/scene_hd/10_size_at_range.png)

- **refused** this camera has no intrinsics in this repository, and this stage is arithmetic on fx, fy, cx, cy. Assuming cx = w/2 is exactly the error that throws a ray by 122 px on the Kinova module -- 94 mm at 1 m, three times the grasp tolerance. Calibrate it: scripts/calibrate_scene_camera.py

### 11  FastSAM: every region in the picture, no prompt  [ok]

![fastsam](stages/scene_hd/11_fastsam.png)

- **formula** `object-agnostic instance masks; no class, no prompt, no scene knowledge -- 'what are the regions', not 'where is the green cube'`
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 3.9 s for this frame
- **summary** 83 regions | largest 85395 px (27.8% of frame), smallest 40 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 83 | count |
| largest region | 85395 | px |
| largest as a fraction of the frame | 27.8 | % |
| smallest region | 40 | px |
| median region | 624 | px |
| inference time | 3.89 | s |
| input size | 1024 | px |
| confidence threshold | 0.25 | - |
| NMS IoU | 0.7 | - |
| device | cpu | - |


*the twenty largest regions* -- 20 rows, full data in `data/scene_hd/11_fastsam__the_twenty_largest_regions.csv`

- **note** This is what replaced clustering. Two 40 mm cubes on a 60 mm pitch are 20 mm apart, which is the cluster distance itself, so depth-clustering fused them into one object 140 mm across the jaws and refused it. In the PICTURE they are not remotely ambiguous. Masks decide WHAT is an object; depth decides WHERE it is.

### 12  The area filter: noise below, the table above  [ok]

![area_filter](stages/scene_hd/12_area_filter.png)

- **formula** `keep iff  60 px <= area <= 0.35 * W * H  (= 107520 px here)`
- **source** srl_perception/segment_lift.py: MIN_AREA_PX, MAX_AREA_FRAC
- **summary** 83 in, 82 kept, 1 too small, 0 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 83 | count |
| kept | 82 | count |
| rejected as noise | 1 | count |
| rejected as too large | 0 | count |
| lower bound | 60 | px |
| upper bound | 107520 | px |
| upper bound as a fraction | 0.35 | - |

- **note** The upper bound is not tidiness. A segmenter returns the table and the room as legitimate regions, and a region covering a third of the frame is never a thing to be picked up.

### 13  Segment everything, then pick the one that matches  [ok]

![segment_match](stages/scene_hd/13_segment_match.png)

- **formula** `for each region: mean BGR over the MASK, then the ratio test of stage 08; score = min(1, area/2000)`
- **source** srl_perception/prompt_detector.py:_segment -- the backend that actually found the target on this rig
- **summary** 267 px at (301, 294), mean BGR [110.7, 143.4, 77.9]

| quantity | value | unit |
| --- | ---: | --- |
| colour word parsed | green | - |
| regions considered | 83 | count |
| regions matched | 1 | count |
| regions rejected on colour | 82 | count |
| best score | 0.134 | 0-1 |


*matched regions* -- 1 rows, full data in `data/scene_hd/13_segment_match__matched_regions.csv`

- **note** Naming the cube failed at every confidence -- it is ~19 px and dark, and an open-vocabulary detector cannot recognise it as anything. Segmenting it is trivial: 89 regions, exactly one green-dominant, no false positives. The MASK is the other win: a colour threshold gives whatever pixels passed, a segment gives the object's actual extent, which is what a width measurement needs.

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](stages/scene_hd/14_yoloworld.png)

- **refused** YOLO-World ran in 9.2 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

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
- **source** MediaPipe PoseLandmarker (full), num_poses=4, run in .venv_pose as a separate interpreter -- the same model srl_perception/wearer_tracker_node.py uses, 3.3 s
- **summary** 1 person(s)

| quantity | value | unit |
| --- | ---: | --- |
| people detected | 1 | count |
| landmarks per person | 33 | count |
| inference time | 3.33 | s |
| depth attached to landmarks | no | - |


*body landmarks* -- 8 rows, full data in `data/scene_hd/32_people__body_landmarks.csv`

- **note** This is the wearer-tracking path. The rule the whole safety case rests on: a camera may only ever make the modelled body BIGGER -- fuse() keeps whichever of the tracked and mannequin primitive is CLOSER to the robot, per part, so a body measured further away changes nothing and a lost track falls back rather than shrinking the person.

### 33  The robot in the picture, and what is NOT detected  [by_design]

![self_view](stages/scene_hd/33_self_view.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

