# Computer vision stages, 2026-08-30T07:05:29

Written by `scripts/cv_pickpose_visuals.py`. The arms were NOT commanded; this program has no publisher, no service client and no Kortex session.

## The arms

Read from `/real/joint_states` (502 distinct source stamps), compared with the saved ideal pick pose.

| arm | reference | joints read | worst deviation | at the pick pose |
| --- | --- | --- | --- | --- |
| left | `config/pick_pose_ideal_left.txt` | 7 of 7 | 79.99 deg | **NO** |
| right | `config/pick_pose_ideal_right.txt` | 7 of 7 | 1.19 deg | YES |

<details><summary>left, joint by joint</summary>

| joint | measured (deg) | pick pose (deg) | delta (deg) |
| --- | --- | --- | --- |
| joint_1 | 67.039 | 97.743 | -30.704 |
| joint_2 | 27.489 | 37.529 | -10.039 |
| joint_3 | 170.415 | 134.368 | +36.047 |
| joint_4 | -90.409 | -87.339 | -3.071 |
| joint_5 | -98.812 | -18.818 | -79.993 |
| joint_6 | -76.026 | -28.989 | -47.036 |
| joint_7 | -66.379 | -128.712 | +62.333 |

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
| focus, variance of Laplacian | 38 | - |
| pixels clipped white | 0.014 | % |
| pixels crushed black | 0.108 | % |
| mean blue | 147.21 | 0-255 |
| mean green | 144.93 | 0-255 |
| mean red | 151.36 | 0-255 |
| median grey | 154 | 0-255 |
| grey std | 56.71 | 0-255 |

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
- **summary** median H 133  S 16  V 163 | 85.8% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 133 | 0-179 |
| median saturation | 16 | 0-255 |
| median value | 163 | 0-255 |
| pixels below S=80 | 85.756 | % |
| pixels below V=40 | 0.842 | % |
| pixels above V=200 | 25.23 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/left_gripper/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 31551 px selected (3.424% of the frame) | of those, median S 107 and median V 142

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 31551 | px |
| fraction of the frame | 3.4235 | % |
| median S of the selected | 107 | 0-255 |
| median V of the selected | 142 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/left_gripper/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 31551 px in, 30751 px out, 800 px removed (2.5%) | 41 separate blobs before, 1 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 31551 | px |
| pixels after | 30751 | px |
| pixels removed | 800 | px |
| removed | 2.536 | % |
| blobs before | 41 | count |
| blobs after | 1 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/left_gripper/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 1 components over 40 px | largest 30751 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 1 | count |
| largest area | 30751 | px |
| total area kept | 30751 | px |
| median area | 3.075e+04 | px |


*every component* -- 1 rows, full data in `data/left_gripper/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/left_gripper/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** white 234039 | orange 46950 | green 31114 | black 11172 | red 6977 | blue 138

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 7 | count |
| largest term | white | - |
| its pixel count | 234039 | px |


*pixels per colour term* -- 11 rows, full data in `data/left_gripper/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/left_gripper/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 30751 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 31114 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 31114 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

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
- **summary** 209 x 199 px, 78.3 deg, aspect 1.05, identity (not measured)

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
- **summary** 209 px -> 0.248 m REJECT

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
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 10.9 s for this frame
- **summary** 30 regions | largest 455688 px (49.4% of frame), smallest 323 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 30 | count |
| largest region | 455688 | px |
| largest as a fraction of the frame | 49.45 | % |
| smallest region | 323 | px |
| median region | 1.042e+04 | px |
| inference time | 10.91 | s |
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
- **summary** 30 in, 28 kept, 0 too small, 2 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 30 | count |
| kept | 28 | count |
| rejected as noise | 0 | count |
| rejected as too large | 2 | count |
| lower bound | 60 | px |
| upper bound | 322560 | px |
| upper bound as a fraction | 0.35 | - |

- **note** The upper bound is not tidiness. A segmenter returns the table and the room as legitimate regions, and a region covering a third of the frame is never a thing to be picked up.

### 13  Segment everything, then pick the one that matches  [refused]

![segment_match](stages/left_gripper/13_segment_match.png)

- **refused** FastSAM returned 30 regions and NONE of them is green-dominant by the region-mean test. That is a real 'not found' for this prompt in this frame -- the segmenter ran, the gate ran, and nothing matched.

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](stages/left_gripper/14_yoloworld.png)

- **refused** YOLO-World ran in 16.8 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

### 15  The depth frame, and where it has no answer  [ok]

![raw_depth](stages/left_gripper/15_raw_depth.png)

- **formula** `one 16-bit value per pixel, in MILLIMETRES on the wire, divided by 1000 here. 0 means 'no return' and is NOT a distance of zero.`
- **source** Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 480 x 270 | 88.9% of pixels have a return | median 2.214 m, 5th-95th 0.425-5.377 m | 54570 px between the 0.08 and 1.50 m gate the pick path uses

| quantity | value | unit |
| --- | ---: | --- |
| depth width | 480 | px |
| depth height | 270 | px |
| pixels with a return | 115256 | px |
| that fraction | 88.93 | % |
| median range | 2.214 | m |
| 5th percentile | 0.425 | m |
| 95th percentile | 5.377 | m |
| nearest return | 0.363 | m |
| furthest return | 10.565 | m |
| returns inside the working gate | 54570 | px |
| gate near | 0.08 | m |
| gate far | 1.5 | m |

- **note** A hole is not a far surface. Averaging a hole in as 0 drags every edge pixel toward the camera, which is why the RealSense average is taken over VALID pixels only.

### 16  Depth put into the colour frame  [ok]

![alignment](stages/left_gripper/16_alignment.png)

- **formula** `P = ((u-cx_d)z/fx_d, (v-cy_d)z/fy_d, z);  P' = P + t;  u' = fx_c X'/Z' + cx_c,  v' = fy_c Y'/Z' + cy_c`
- **source** forward-projected here: deproject with the depth intrinsics, translate by the recorded -27.1, -10.0, -4.7 mm baseline, project with the colour intrinsics, nearest point wins
- **summary** 66597 points projected, 48069 fell outside the colour frame, 7.2% of colour pixels got a depth

| quantity | value | unit |
| --- | ---: | --- |
| points projected | 66597 | count |
| points falling outside the colour frame | 48069 | count |
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
- **summary** 54570 points in the 0.08-1.50 m band | extent X -0.418 to 0.420, Y -0.067 to 0.263, Z 0.363 to 0.860 m

| quantity | value | unit |
| --- | ---: | --- |
| points lifted | 54570 | count |
| fx | 360.013 | px |
| fy | 360.013 | px |
| cx | 243.872 | px |
| cy | 137.922 | px |
| X minimum | -0.4183 | m |
| X maximum | 0.4197 | m |
| Y minimum | -0.067 | m |
| Y maximum | 0.2629 | m |
| Z minimum | 0.363 | m |
| Z maximum | 0.86 | m |
| centroid X | 0.0125 | m |
| centroid Y | 0.0816 | m |
| centroid Z | 0.5672 | m |

- **note** This is the whole of 'depth decides WHERE'. Everything after it is geometry on these points, in the CAMERA's frame -- no robot transform is involved yet, which is exactly why the pick measures its error here.

### 18  RANSAC: the support plane  [ok]

![ransac](stages/left_gripper/18_ransac.png)

- **formula** `draw 3 points -> n = (b-a) x (c-a) / |...|,  d = -n.a;  score = #{ |n.P + d| < 6 mm };  keep the best of 400 draws`
- **source** scripts/servo_pick_left.py:fit_plane, imported; fitted on 54570 of 54570 points
- **summary** normal (-0.2487, -0.8833, -0.3975), offset 0.2947 m | 40560 of 54570 points are inliers (74.3%) | RMS 1.41 mm | 66.6 deg between the plane normal and the camera's optical axis

| quantity | value | unit |
| --- | ---: | --- |
| normal x | -0.2487 | - |
| normal y | -0.8833 | - |
| normal z | -0.3975 | - |
| offset d | 0.2947 | m |
| points fitted | 54570 | count |
| points scored | 54570 | count |
| inliers | 40560 | count |
| inlier fraction | 74.33 | % |
| residual RMS | 1.408 | mm |
| inlier tolerance | 6 | mm |
| RANSAC iterations | 400 | count |
| angle to the optical axis | 66.58 | deg |

- **note** A plane with too few inliers is not a plane. The pick refuses below 200 inliers rather than returning a geometry whose RMS is NaN -- everything downstream would treat that as a fix and the arm would move on it.

### 19  The PCA refinement, and the NaN it once produced  [ok]

![pca_refine](stages/left_gripper/19_pca_refine.png)

- **formula** `C = cov(Q - mean(Q)) for inliers Q;  n = eigvec(C)[:, argmin];  d = -n . mean(Q)   <-- BOTH halves, together`
- **source** scripts/servo_pick_left.py:fit_plane, imported
- **summary** the normal moved 0.000 deg | RMS 1.41 mm -> 1.41 mm | mixing the refined normal with the RANSAC offset selects 40560 inliers instead of 40560

| quantity | value | unit |
| --- | ---: | --- |
| normal moved | 0 | deg |
| RMS before refinement | 1.408 | mm |
| RMS after refinement | 1.408 | mm |
| inliers with the matched pair | 40560 | count |
| inliers with a mixed normal and offset | 40560 | count |
| inliers lost by mixing | 0 | count |

- **note** That last number is the bug this stage exists to show. The function once returned the REFINED normal with the OLD offset; those describe different planes, the inlier mask could select ZERO points, and mean() of an empty slice is NaN. The pick printed 'plane RMS nan mm' and 'FOUND' in the same breath, then closed 540 mm blind on a centre computed against that plane.

### 20  Height above the plane  [ok]

![height_map](stages/left_gripper/20_height_map.png)

- **formula** `h(P) = n . P + d,  signed;  positive is off the surface, toward the camera`
- **source** servo_pick_left.measure -- the same expression the live pick uses to separate the cube from the table
- **summary** 9610 of 54570 points stand more than 5 mm off the plane (17.61%) | tallest 94.2 mm

| quantity | value | unit |
| --- | ---: | --- |
| height threshold | 5 | mm |
| points above it | 9610 | count |
| that fraction | 17.61 | % |
| tallest point | 94.18 | mm |
| lowest point | -327.72 | mm |
| median height | 0.134 | mm |

- **note** The floor is 5 mm because the plane RMS is under 1 mm on this sensor. Set it below the depth noise and the table itself becomes an object; set it far above and a flat item is missed.

### 21  The surface height as a MODE, not a mean  [ok]

![mode_surface](stages/left_gripper/21_mode_surface.png)

- **formula** `bin at 2 mm; take the fullest bin; require it to hold >= 20% of the points; refine as the mean of everything within +/-6 mm of it`
- **source** srl_perception/surface_from_depth.py, applied along the fitted plane normal because this program has no camera-to-world transform; the node applies it to world z
- **summary** modal bin holds 40.0% of 54570 points (floor 20%) | refined -0.2948 m, spread 1.40 mm | a plain MEAN would say -0.3007 m, which is 5.9 mm out | RANSAC's own offset is -0.2947 m, 0.10 mm from this

| quantity | value | unit |
| --- | ---: | --- |
| bin width | 2 | mm |
| points binned | 54570 | count |
| share in the fullest bin | 39.98 | % |
| share required | 20 | % |
| modal bin centre | -0.2954 | m |
| refined estimate | -0.2948 | m |
| spread of the inliers | 1.397 | mm |
| inlier band | 6 | mm |
| a plain mean would say | -0.3007 | m |
| that mean's error | 5.86 | mm |
| RANSAC's own offset | -0.2947 | m |
| the two estimators differ by | 0.097 | mm |

- **note** Two independent estimators of one surface, so they can be compared: RANSAC votes on inliers, this votes on the fullest histogram bin. A mean is dragged up by everything standing on the table and down by every hole, and it MOVES as the objects move -- the 'measurement' would change between trials on a motionless table.

### 22  What is standing on the surface  [refused]

![on_surface](stages/left_gripper/22_on_surface.png)

- **refused** nothing stands more than 5 mm above the fitted plane in this view

### 23  The measurement the pick actually acts on  [ok]

![cube_measurement](stages/left_gripper/23_cube_measurement.png)

- **formula** `cube = colour(u,v) AND h > 5 mm;  foot = mean(Pk) - n (n.mean(Pk) + d);  centre = foot + n * max(h)/2`
- **source** scripts/servo_pick_left.py:measure -- this is the measurement the servo loop nulls against, in the CAMERA's frame, where the data is good
- **summary** 1085 object points | top 58.4 mm above the plane | centre (0.0708, 0.0472, 0.5189) m in the camera frame | plane RMS 1.41 mm, inlier fraction 0.74

| quantity | value | unit |
| --- | ---: | --- |
| object points | 1085 | count |
| points that are the colour but ON the plane | 1190 | count |
| points off the plane but not the colour | 8525 | count |
| top above the plane | 58.37 | mm |
| median height | 32.28 | mm |
| centre x | 0.0708 | m |
| centre y | 0.0472 | m |
| centre z | 0.5189 | m |
| range to the centre | 0.5258 | m |
| plane RMS | 1.408 | mm |
| plane inlier fraction | 0.7433 | - |
| minimum points the pick requires | 60 | count |

- **note** Two independent gates, and that is the point: colour alone picks up the green pad the cube stands on and anything green in the room; height alone picks up everything on the table. The centre uses the foot point rather than the cloud's centroid because a depth camera sees a SHELL -- the front surface only -- and a shell's centroid is biased toward the camera, measured at 10.5 mm in the error budget against a 30 mm capture gate.

### 24  Splitting a cube from the pad it stands on  [ok]

![layers](stages/left_gripper/24_layers.png)

- **formula** `find the MODAL height bin (5 mm bins); cut 12 mm above it; recurse on what is left, twice`
- **source** srl_perception/segment_lift.py:_layers
- **summary** layer 1: 4-16 mm, 1411 points | layer 2: 16-43 mm, 2572 points | layer 3: 43-67 mm, 1846 points

| quantity | value | unit |
| --- | ---: | --- |
| region area | 106510 | px |
| points lifted from it | 7484 | count |
| points above the plane | 5829 | count |
| pixels removed by the 1 px erosion | 2124 | px |
| layers found | 3 | count |
| lowest point | 4 | mm |
| highest point | 67.1 | mm |


*height layers* -- 3 rows, full data in `data/left_gripper/24_layers__height_layers.csv`

- **note** Appearance decides what is SIDE BY SIDE; height decides what is STACKED. A blue cube on a blue pad is correctly ONE region -- same colour, touching, no edge between them. And the split cannot look for an empty band, because there is none: the cube RESTS on the pad, so its sides fill every bin. What is there is a MODE -- the support's top face is a large flat area and dominates the histogram.

### 25  Rotating calipers, against the two controls  [ok]

![min_width](stages/left_gripper/25_min_width.png)

- **formula** `width = min over theta of [ max(p.u(theta)) - min(p.u(theta)) ],  u(theta) = (cos, sin) in the support plane, swept at 0.25 deg`
- **source** srl_perception/segment_lift.py:_min_width and rgbd_grasp.min_width_frame
- **summary** calipers 277.0 mm | PCA's short axis 293.9 mm | axis-aligned box 277.0 mm | long axis 340.8 mm, height 63.1 mm | jaws close on 85 mm, so this is TOO WIDE

| quantity | value | unit |
| --- | ---: | --- |
| instance points | 5829 | count |
| minimum width, rotating calipers | 276.97 | mm |
| at this angle in the plane | 0 | deg |
| PCA short axis, THE CONTROL | 293.92 | mm |
| axis-aligned box, THE CONTROL | 276.97 | mm |
| PCA overstates by | 16.95 | mm |
| long axis | 340.77 | mm |
| height | 63.1 | mm |
| jaw span | 85 | mm |
| graspable as it lies | no | - |
| sweep step | 0.5 | deg |

- **note** The two controls are here because both were used and both were wrong. The principal axes of a SQUARE are degenerate -- equal variance in both directions -- so SVD returns the diagonal and a 40 mm cube measures 40*sqrt(2) = 53 mm; against an 85 mm jaw that refuses a 60 mm object as ungraspable. The error that matters is not the 13 mm, it is the DIRECTION: a jaw told to close across a diagonal approaches the corner and the object rolls out.

### 26  Dropping masks that are unions of finer ones  [ok]

![nested](stages/left_gripper/26_nested.png)

- **formula** `voxelise each instance at 5 mm; accept smallest first; reject one whose claimed-voxel overlap exceeds 0.55`
- **source** srl_perception/segment_lift.py:_suppress_nested
- **summary** 6 instances in, 5 kept, 1 dropped | overlaps: 0.92

| quantity | value | unit |
| --- | ---: | --- |
| instances in | 6 | count |
| kept | 5 | count |
| dropped as already explained | 1 | count |
| voxel size | 5 | mm |
| overlap fraction that rejects | 0.55 | - |


*per instance* -- 6 rows, full data in `data/left_gripper/26_nested__per_instance.csv`

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
- **summary** 0.579 x 0.423 m of surface, 0.277 m away, flat to 1.36 mm

| quantity | value | unit |
| --- | ---: | --- |
| points on the surface | 37767 | count |
| surface extent along its first axis | 0.5787 | m |
| surface extent along its second axis | 0.4227 | m |
| area spanned | 0.2446 | m2 |
| perpendicular distance from the camera | 0.2772 | m |
| distance to the middle of that extent | 0.5912 | m |
| nearest point on the surface | 0.3977 | m |
| furthest point on the surface | 0.9313 | m |
| tilt from the optical axis | 66.58 | deg |
| flatness, RMS of the inliers | 1.355 | mm |

- **note** This is a MEASURED surface, not the declared one. srl_experiments/work_surface.py has had set_measured() since it was written and nothing ever called it, so a table 20 mm out would have been absorbed silently into every grasp.

### 30  Every object, as an oriented 3-D box  [ok]

![boxes3d](stages/left_gripper/30_boxes3d.png)

- **formula** `centre = mid-extent ACROSS the surface + (foot + top/2) ALONG the normal; axes = (minimum-width direction, its perpendicular, the plane normal); size = the spans along them`
- **source** segment_lift._measure for the horizontal centre and rgbd_grasp.shell_corrected_centre's reasoning for the vertical -- the combination is checked to 2 mm against a constructed cube in --self-test
- **summary** 6 objects, 0.536 to 0.847 m away

| quantity | value | unit |
| --- | ---: | --- |
| objects posed | 6 | count |
| nearest object | 0.5355 | m |
| furthest object | 0.8474 | m |
| largest footprint | 340.8 | mm |
| tallest object | 67.1 | mm |
| graspable within the 85 mm jaw | 5 | count |


*every object, in the camera frame* -- 6 rows, full data in `data/left_gripper/30_boxes3d__every_object__in_the_camera_frame.csv`

- **note** These coordinates are in the CAMERA's frame. Putting them in the robot's frame needs the camera extrinsic, which on the wrist is measured at ~8.5 deg wrong and rotates with the joint -- 34 mm of error at 0.23 m. That is why the pick servos on an error measured HERE rather than planning through that transform.

### 31  How far apart everything is  [ok]

![distances](stages/left_gripper/31_distances.png)

- **formula** `range = |centre| (the camera is the origin); height = n.c + d; separation = |c_i - c_j|; free gap subtracts each object's half-width`
- **source** the poses of stage 30, all in the camera's own frame
- **summary** 6 objects, nearest 0.536 m, closest pair 0.001 m

| quantity | value | unit |
| --- | ---: | --- |
| objects measured | 6 | count |
| nearest object | 0.5355 | m |
| furthest object | 0.8474 | m |
| closest pair | 0.0012 | m |
| tightest free gap | 0 | m |
| what the origin IS | the wrist camera, so range is distance from the HAND | - |


*each object* -- 6 rows, full data in `data/left_gripper/31_distances__each_object.csv`


*between objects* -- 15 rows, full data in `data/left_gripper/31_distances__between_objects.csv`

- **note** On a WRIST camera the origin is the hand, so 'range' is already the distance the arm has to close -- that is the quantity the servo loop nulls. On the scene camera it is the distance from the camera, and turning that into a distance from the ARM needs the scene-camera-to-robot calibration, which this repository does not have measured.

### 32  People in the picture  [refused]

![people](stages/left_gripper/32_people.png)

- **refused** no person is in this frame. MediaPipe ran in 4.4 s and found nobody, which is an answer -- the wearer-tracking safety case only ever makes the modelled body BIGGER, so 'nobody seen' falls back to the mannequin rather than to an empty scene.

### 33  The robot in the picture, and what is NOT detected  [ok]

![self_view](stages/left_gripper/33_self_view.png)

- **formula** `{ pixels with 0 < depth < 0.08 m } -- nearer than anything the arm works on`
- **source** Kinova vision module on the left arm, over ROS topics /left_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 0 px nearer than 0.08 m

| quantity | value | unit |
| --- | ---: | --- |
| pixels nearer than the gate | 0 | px |
| that fraction | 0 | % |
| nearest return of all | 0.363 | m |
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
- **summary** 1280 x 720 px | focus (variance of Laplacian) 117 | 5.36% of pixels clipped white, 0.05% crushed black

| quantity | value | unit |
| --- | ---: | --- |
| frame width | 1280 | px |
| frame height | 720 | px |
| focus, variance of Laplacian | 116.5 | - |
| pixels clipped white | 5.363 | % |
| pixels crushed black | 0.051 | % |
| mean blue | 144.03 | 0-255 |
| mean green | 140.03 | 0-255 |
| mean red | 141.16 | 0-255 |
| median grey | 132 | 0-255 |
| grey std | 70.39 | 0-255 |

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
- **summary** median H 114  S 38  V 151 | 68.9% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 114 | 0-179 |
| median saturation | 38 | 0-255 |
| median value | 151 | 0-255 |
| pixels below S=80 | 68.926 | % |
| pixels below V=40 | 4.174 | % |
| pixels above V=200 | 31.679 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/right_gripper/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 12436 px selected (1.349% of the frame) | of those, median S 143 and median V 128

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 12436 | px |
| fraction of the frame | 1.3494 | % |
| median S of the selected | 143 | 0-255 |
| median V of the selected | 128 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/right_gripper/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 12436 px in, 12354 px out, 82 px removed (0.7%) | 1 separate blobs before, 1 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 12436 | px |
| pixels after | 12354 | px |
| pixels removed | 82 | px |
| removed | 0.659 | % |
| blobs before | 1 | count |
| blobs after | 1 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/right_gripper/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 1 components over 40 px | largest 12354 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 1 | count |
| largest area | 12354 | px |
| total area kept | 12354 | px |
| median area | 1.235e+04 | px |


*every component* -- 1 rows, full data in `data/right_gripper/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/right_gripper/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** white 287431 | blue 74908 | black 46648 | orange 33563 | cyan 29063 | red 26575

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 8 | count |
| largest term | white | - |
| its pixel count | 287431 | px |


*pixels per colour term* -- 11 rows, full data in `data/right_gripper/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/right_gripper/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 12354 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 12416 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 12388 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

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
- **summary** 125 x 117 px, 12.5 deg, aspect 1.07, identity (not measured)

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
- **summary** 125 px -> 0.414 m accept

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
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 3.6 s for this frame
- **summary** 44 regions | largest 260241 px (28.2% of frame), smallest 319 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 44 | count |
| largest region | 260241 | px |
| largest as a fraction of the frame | 28.24 | % |
| smallest region | 319 | px |
| median region | 6880 | px |
| inference time | 3.6 | s |
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
- **summary** 44 in, 44 kept, 0 too small, 0 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 44 | count |
| kept | 44 | count |
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
- **summary** 13159 px at (703, 343), mean BGR [104.7, 149.3, 72.1]

| quantity | value | unit |
| --- | ---: | --- |
| colour word parsed | green | - |
| regions considered | 44 | count |
| regions matched | 1 | count |
| regions rejected on colour | 43 | count |
| best score | 1 | 0-1 |


*matched regions* -- 1 rows, full data in `data/right_gripper/13_segment_match__matched_regions.csv`

- **note** Naming the cube failed at every confidence -- it is ~19 px and dark, and an open-vocabulary detector cannot recognise it as anything. Segmenting it is trivial: 89 regions, exactly one green-dominant, no false positives. The MASK is the other win: a colour threshold gives whatever pixels passed, a segment gives the object's actual extent, which is what a width measurement needs.

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](stages/right_gripper/14_yoloworld.png)

- **refused** YOLO-World ran in 6.8 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

### 15  The depth frame, and where it has no answer  [ok]

![raw_depth](stages/right_gripper/15_raw_depth.png)

- **formula** `one 16-bit value per pixel, in MILLIMETRES on the wire, divided by 1000 here. 0 means 'no return' and is NOT a distance of zero.`
- **source** Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 480 x 270 | 80.8% of pixels have a return | median 1.838 m, 5th-95th 0.339-5.105 m | 42931 px between the 0.08 and 1.50 m gate the pick path uses

| quantity | value | unit |
| --- | ---: | --- |
| depth width | 480 | px |
| depth height | 270 | px |
| pixels with a return | 104751 | px |
| that fraction | 80.83 | % |
| median range | 1.838 | m |
| 5th percentile | 0.339 | m |
| 95th percentile | 5.105 | m |
| nearest return | 0.149 | m |
| furthest return | 7.657 | m |
| returns inside the working gate | 42931 | px |
| gate near | 0.08 | m |
| gate far | 1.5 | m |

- **note** A hole is not a far surface. Averaging a hole in as 0 drags every edge pixel toward the camera, which is why the RealSense average is taken over VALID pixels only.

### 16  Depth put into the colour frame  [ok]

![alignment](stages/right_gripper/16_alignment.png)

- **formula** `P = ((u-cx_d)z/fx_d, (v-cy_d)z/fy_d, z);  P' = P + t;  u' = fx_c X'/Z' + cx_c,  v' = fy_c Y'/Z' + cy_c`
- **source** forward-projected here: deproject with the depth intrinsics, translate by the recorded -27.1, -10.0, -4.7 mm baseline, project with the colour intrinsics, nearest point wins
- **summary** 60377 points projected, 44130 fell outside the colour frame, 6.6% of colour pixels got a depth

| quantity | value | unit |
| --- | ---: | --- |
| points projected | 60377 | count |
| points falling outside the colour frame | 44130 | count |
| colour pixels given a depth | 6.55 | % |
| baseline x | -27.06 | mm |
| baseline y | -9.97 | mm |
| baseline z | -4.71 | mm |
| rotation assumed | identity | - |

- **note** The rotation between the two sensors is taken as IDENTITY here, because a recorded translation is all this program has. The live pick path uses the URDF FK between camera_depth_frame and camera_color_frame instead -- and that extrinsic is known to be ~8.5 deg wrong and to rotate with the wrist, which is why the pick servos in the camera frame rather than planning through it.

### 17  Pixels and depth become a cloud  [ok]

![deprojection](stages/right_gripper/17_deprojection.png)

- **formula** `X = (u - cx) Z / fx,   Y = (v - cy) Z / fy,   Z = depth`
- **source** rgbd_grasp.deproject / servo_pick_left.measure, with fx 360.01 fy 360.01 cx 243.87 cy 137.92
- **summary** 42931 points in the 0.08-1.50 m band | extent X -0.283 to 0.441, Y -0.181 to 0.180, Z 0.149 to 0.926 m

| quantity | value | unit |
| --- | ---: | --- |
| points lifted | 42931 | count |
| fx | 360.013 | px |
| fy | 360.013 | px |
| cx | 243.872 | px |
| cy | 137.922 | px |
| X minimum | -0.2834 | m |
| X maximum | 0.4405 | m |
| Y minimum | -0.1812 | m |
| Y maximum | 0.1799 | m |
| Z minimum | 0.149 | m |
| Z maximum | 0.926 | m |
| centroid X | 0.0301 | m |
| centroid Y | 0.0901 | m |
| centroid Z | 0.5402 | m |

- **note** This is the whole of 'depth decides WHERE'. Everything after it is geometry on these points, in the CAMERA's frame -- no robot transform is involved yet, which is exactly why the pick measures its error here.

### 18  RANSAC: the support plane  [ok]

![ransac](stages/right_gripper/18_ransac.png)

- **formula** `draw 3 points -> n = (b-a) x (c-a) / |...|,  d = -n.a;  score = #{ |n.P + d| < 6 mm };  keep the best of 400 draws`
- **source** scripts/servo_pick_left.py:fit_plane, imported; fitted on 42931 of 42931 points
- **summary** normal (0.1260, -0.9409, -0.3144), offset 0.2981 m | 22654 of 42931 points are inliers (52.8%) | RMS 1.60 mm | 71.7 deg between the plane normal and the camera's optical axis

| quantity | value | unit |
| --- | ---: | --- |
| normal x | 0.126 | - |
| normal y | -0.9409 | - |
| normal z | -0.3144 | - |
| offset d | 0.2981 | m |
| points fitted | 42931 | count |
| points scored | 42931 | count |
| inliers | 22654 | count |
| inlier fraction | 52.77 | % |
| residual RMS | 1.597 | mm |
| inlier tolerance | 6 | mm |
| RANSAC iterations | 400 | count |
| angle to the optical axis | 71.68 | deg |

- **note** A plane with too few inliers is not a plane. The pick refuses below 200 inliers rather than returning a geometry whose RMS is NaN -- everything downstream would treat that as a fix and the arm would move on it.

### 19  The PCA refinement, and the NaN it once produced  [ok]

![pca_refine](stages/right_gripper/19_pca_refine.png)

- **formula** `C = cov(Q - mean(Q)) for inliers Q;  n = eigvec(C)[:, argmin];  d = -n . mean(Q)   <-- BOTH halves, together`
- **source** scripts/servo_pick_left.py:fit_plane, imported
- **summary** the normal moved 0.000 deg | RMS 1.60 mm -> 1.60 mm | mixing the refined normal with the RANSAC offset selects 22654 inliers instead of 22654

| quantity | value | unit |
| --- | ---: | --- |
| normal moved | 0 | deg |
| RMS before refinement | 1.597 | mm |
| RMS after refinement | 1.597 | mm |
| inliers with the matched pair | 22654 | count |
| inliers with a mixed normal and offset | 22654 | count |
| inliers lost by mixing | 0 | count |

- **note** That last number is the bug this stage exists to show. The function once returned the REFINED normal with the OLD offset; those describe different planes, the inlier mask could select ZERO points, and mean() of an empty slice is NaN. The pick printed 'plane RMS nan mm' and 'FOUND' in the same breath, then closed 540 mm blind on a centre computed against that plane.

### 20  Height above the plane  [ok]

![height_map](stages/right_gripper/20_height_map.png)

- **formula** `h(P) = n . P + d,  signed;  positive is off the surface, toward the camera`
- **source** servo_pick_left.measure -- the same expression the live pick uses to separate the cube from the table
- **summary** 20313 of 42931 points stand more than 5 mm off the plane (47.32%) | tallest 357.8 mm

| quantity | value | unit |
| --- | ---: | --- |
| height threshold | 5 | mm |
| points above it | 20313 | count |
| that fraction | 47.315 | % |
| tallest point | 357.84 | mm |
| lowest point | -12.85 | mm |
| median height | 2.358 | mm |

- **note** The floor is 5 mm because the plane RMS is under 1 mm on this sensor. Set it below the depth noise and the table itself becomes an object; set it far above and a flat item is missed.

### 21  The surface height as a MODE, not a mean  [ok]

![mode_surface](stages/right_gripper/21_mode_surface.png)

- **formula** `bin at 2 mm; take the fullest bin; require it to hold >= 20% of the points; refine as the mean of everything within +/-6 mm of it`
- **source** srl_perception/surface_from_depth.py, applied along the fitted plane normal because this program has no camera-to-world transform; the node applies it to world z
- **summary** modal bin holds 33.1% of 42931 points (floor 20%) | refined -0.2981 m, spread 1.61 mm | a plain MEAN would say -0.2508 m, which is 47.3 mm out | RANSAC's own offset is -0.2981 m, 0.04 mm from this

| quantity | value | unit |
| --- | ---: | --- |
| bin width | 2 | mm |
| points binned | 42931 | count |
| share in the fullest bin | 33.06 | % |
| share required | 20 | % |
| modal bin centre | -0.2979 | m |
| refined estimate | -0.2981 | m |
| spread of the inliers | 1.605 | mm |
| inlier band | 6 | mm |
| a plain mean would say | -0.2508 | m |
| that mean's error | 47.27 | mm |
| RANSAC's own offset | -0.2981 | m |
| the two estimators differ by | 0.039 | mm |

- **note** Two independent estimators of one surface, so they can be compared: RANSAC votes on inliers, this votes on the fullest histogram bin. A mean is dragged up by everything standing on the table and down by every hole, and it MOVES as the objects move -- the 'measurement' would change between trials on a motionless table.

### 22  What is standing on the surface  [ok]

![on_surface](stages/right_gripper/22_on_surface.png)

- **formula** `{ pixels whose lifted point has h > 5 mm }, 8-connected`
- **source** the plane of stage 18 fitted in the colour frame; heights from the aligned depth of stage 16
- **summary** 354 px, 112 pts, top 199 mm

| quantity | value | unit |
| --- | ---: | --- |
| pieces standing on the surface | 1 | count |
| height threshold | 5 | mm |
| largest piece | 354 | px |
| tallest piece | 198.9 | mm |


*pieces above the plane* -- 1 rows, full data in `data/right_gripper/22_on_surface__pieces_above_the_plane.csv`

- **note** This is the pre-2020 method on its own -- threshold above a plane, then cluster what is left. It is exactly what fails when two 40 mm cubes sit 20 mm apart: connectivity fuses them into one object 140 mm across. Compare this figure with stage 11.

### 23  The measurement the pick actually acts on  [ok]

![cube_measurement](stages/right_gripper/23_cube_measurement.png)

- **formula** `cube = colour(u,v) AND h > 5 mm;  foot = mean(Pk) - n (n.mean(Pk) + d);  centre = foot + n * max(h)/2`
- **source** scripts/servo_pick_left.py:measure -- this is the measurement the servo loop nulls against, in the CAMERA's frame, where the data is good
- **summary** 170 object points | top 56.5 mm above the plane | centre (0.0510, 0.0514, 0.7249) m in the camera frame | plane RMS 1.60 mm, inlier fraction 0.53

| quantity | value | unit |
| --- | ---: | --- |
| object points | 170 | count |
| points that are the colour but ON the plane | 711 | count |
| points off the plane but not the colour | 20143 | count |
| top above the plane | 56.51 | mm |
| median height | 20.01 | mm |
| centre x | 0.051 | m |
| centre y | 0.0514 | m |
| centre z | 0.7249 | m |
| range to the centre | 0.7285 | m |
| plane RMS | 1.597 | mm |
| plane inlier fraction | 0.5277 | - |
| minimum points the pick requires | 60 | count |

- **note** Two independent gates, and that is the point: colour alone picks up the green pad the cube stands on and anything green in the room; height alone picks up everything on the table. The centre uses the foot point rather than the cloud's centroid because a depth camera sees a SHELL -- the front surface only -- and a shell's centroid is biased toward the camera, measured at 10.5 mm in the error budget against a 30 mm capture gate.

### 24  Splitting a cube from the pad it stands on  [ok]

![layers](stages/right_gripper/24_layers.png)

- **formula** `find the MODAL height bin (5 mm bins); cut 12 mm above it; recurse on what is left, twice`
- **source** srl_perception/segment_lift.py:_layers
- **summary** layer 1: 4-41 mm, 5097 points | layer 2: 41-53 mm, 1636 points | layer 3: 53-199 mm, 966 points

| quantity | value | unit |
| --- | ---: | --- |
| region area | 260241 | px |
| points lifted from it | 17456 | count |
| points above the plane | 7699 | count |
| pixels removed by the 1 px erosion | 3882 | px |
| layers found | 3 | count |
| lowest point | 4 | mm |
| highest point | 198.87 | mm |


*height layers* -- 3 rows, full data in `data/right_gripper/24_layers__height_layers.csv`

- **note** Appearance decides what is SIDE BY SIDE; height decides what is STACKED. A blue cube on a blue pad is correctly ONE region -- same colour, touching, no edge between them. And the split cannot look for an empty band, because there is none: the cube RESTS on the pad, so its sides fill every bin. What is there is a MODE -- the support's top face is a large flat area and dominates the histogram.

### 25  Rotating calipers, against the two controls  [ok]

![min_width](stages/right_gripper/25_min_width.png)

- **formula** `width = min over theta of [ max(p.u(theta)) - min(p.u(theta)) ],  u(theta) = (cos, sin) in the support plane, swept at 0.25 deg`
- **source** srl_perception/segment_lift.py:_min_width and rgbd_grasp.min_width_frame
- **summary** calipers 190.8 mm | PCA's short axis 226.0 mm | axis-aligned box 241.2 mm | long axis 626.7 mm, height 194.9 mm | jaws close on 85 mm, so this is TOO WIDE

| quantity | value | unit |
| --- | ---: | --- |
| instance points | 7699 | count |
| minimum width, rotating calipers | 190.78 | mm |
| at this angle in the plane | 170 | deg |
| PCA short axis, THE CONTROL | 226.01 | mm |
| axis-aligned box, THE CONTROL | 241.19 | mm |
| PCA overstates by | 35.23 | mm |
| long axis | 626.72 | mm |
| height | 194.87 | mm |
| jaw span | 85 | mm |
| graspable as it lies | no | - |
| sweep step | 0.5 | deg |

- **note** The two controls are here because both were used and both were wrong. The principal axes of a SQUARE are degenerate -- equal variance in both directions -- so SVD returns the diagonal and a 40 mm cube measures 40*sqrt(2) = 53 mm; against an 85 mm jaw that refuses a 60 mm object as ungraspable. The error that matters is not the 13 mm, it is the DIRECTION: a jaw told to close across a diagonal approaches the corner and the object rolls out.

### 26  Dropping masks that are unions of finer ones  [ok]

![nested](stages/right_gripper/26_nested.png)

- **formula** `voxelise each instance at 5 mm; accept smallest first; reject one whose claimed-voxel overlap exceeds 0.55`
- **source** srl_perception/segment_lift.py:_suppress_nested
- **summary** 24 instances in, 20 kept, 4 dropped | overlaps: 0.95, 0.98, 0.99, 0.77

| quantity | value | unit |
| --- | ---: | --- |
| instances in | 24 | count |
| kept | 20 | count |
| dropped as already explained | 4 | count |
| voxel size | 5 | mm |
| overlap fraction that rejects | 0.55 | - |


*per instance* -- 24 rows, full data in `data/right_gripper/26_nested__per_instance.csv`

- **note** FastSAM returns a HIERARCHY, not a partition: for one picture it hands back the cube, the pad, and a region covering both. All three are legitimate. Lifted, they became three objects in the same space and the merge absorbed a 40 mm cube -- measured EXACT at 40.3 mm against a 40 mm truth -- into a 148 mm blob that was then refused as ungraspable. Volume, not pixels: two masks that overlap in the image can be a metre apart in depth.

### 27  The parallel-jaw grasp  [refused]

![grasp](stages/right_gripper/27_grasp.png)

- **refused** the grasp planner refused, which is an ANSWER: object is 113 mm across its narrowest axis; the Robotiq 85 opens 75 mm usable. It cannot be grasped as it lies.

### 28  Fiducials: the primary detector  [refused]

![apriltag](stages/right_gripper/28_apriltag.png)

- **refused** no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That is an answer, not a failure: a tag either decodes or it does not, which is exactly why this is the PRIMARY detector and colour is the capped fallback. Detector used: cv2.aruco.ArucoDetector (OpenCV 4.11.0).

### 29  The support surface, as an object  [ok]

![table](stages/right_gripper/29_table.png)

- **formula** `fit the plane, keep its inliers, span them in the plane's own basis; extent taken at the 2nd and 98th percentile so one stray return cannot stretch it`
- **source** the plane of stage 18, expressed as a rectangle in the camera frame
- **summary** 0.431 x 0.466 m of surface, 0.290 m away, flat to 1.69 mm

| quantity | value | unit |
| --- | ---: | --- |
| points on the surface | 19689 | count |
| surface extent along its first axis | 0.4305 | m |
| surface extent along its second axis | 0.4658 | m |
| area spanned | 0.2005 | m2 |
| perpendicular distance from the camera | 0.2901 | m |
| distance to the middle of that extent | 0.6671 | m |
| nearest point on the surface | 0.4656 | m |
| furthest point on the surface | 0.9366 | m |
| tilt from the optical axis | 71.7 | deg |
| flatness, RMS of the inliers | 1.688 | mm |

- **note** This is a MEASURED surface, not the declared one. srl_experiments/work_surface.py has had set_measured() since it was written and nothing ever called it, so a table 20 mm out would have been absorbed silently into every grasp.

### 30  Every object, as an oriented 3-D box  [ok]

![boxes3d](stages/right_gripper/30_boxes3d.png)

- **formula** `centre = mid-extent ACROSS the surface + (foot + top/2) ALONG the normal; axes = (minimum-width direction, its perpendicular, the plane normal); size = the spans along them`
- **source** segment_lift._measure for the horizontal centre and rgbd_grasp.shell_corrected_centre's reasoning for the vertical -- the combination is checked to 2 mm against a constructed cube in --self-test
- **summary** 12 objects, 0.338 to 0.895 m away

| quantity | value | unit |
| --- | ---: | --- |
| objects posed | 12 | count |
| nearest object | 0.3377 | m |
| furthest object | 0.8955 | m |
| largest footprint | 626.7 | mm |
| tallest object | 223.4 | mm |
| graspable within the 85 mm jaw | 6 | count |


*every object, in the camera frame* -- 12 rows, full data in `data/right_gripper/30_boxes3d__every_object__in_the_camera_frame.csv`

- **note** These coordinates are in the CAMERA's frame. Putting them in the robot's frame needs the camera extrinsic, which on the wrist is measured at ~8.5 deg wrong and rotates with the joint -- 34 mm of error at 0.23 m. That is why the pick servos on an error measured HERE rather than planning through that transform.

### 31  How far apart everything is  [ok]

![distances](stages/right_gripper/31_distances.png)

- **formula** `range = |centre| (the camera is the origin); height = n.c + d; separation = |c_i - c_j|; free gap subtracts each object's half-width`
- **source** the poses of stage 30, all in the camera's own frame
- **summary** 10 objects, nearest 0.338 m, closest pair 0.009 m

| quantity | value | unit |
| --- | ---: | --- |
| objects measured | 10 | count |
| nearest object | 0.3377 | m |
| furthest object | 0.8955 | m |
| closest pair | 0.0091 | m |
| tightest free gap | 0 | m |
| what the origin IS | the wrist camera, so range is distance from the HAND | - |


*each object* -- 10 rows, full data in `data/right_gripper/31_distances__each_object.csv`


*between objects* -- 45 rows, full data in `data/right_gripper/31_distances__between_objects.csv`

- **note** On a WRIST camera the origin is the hand, so 'range' is already the distance the arm has to close -- that is the quantity the servo loop nulls. On the scene camera it is the distance from the camera, and turning that into a distance from the ARM needs the scene-camera-to-robot calibration, which this repository does not have measured.

### 32  People in the picture  [refused]

![people](stages/right_gripper/32_people.png)

- **refused** no person is in this frame. MediaPipe ran in 3.3 s and found nobody, which is an answer -- the wearer-tracking safety case only ever makes the modelled body BIGGER, so 'nobody seen' falls back to the mannequin rather than to an empty scene.

### 33  The robot in the picture, and what is NOT detected  [ok]

![self_view](stages/right_gripper/33_self_view.png)

- **formula** `{ pixels with 0 < depth < 0.08 m } -- nearer than anything the arm works on`
- **source** Kinova vision module on the right arm, over ROS topics /right_camera/{color,depth} (driver: kinova_vision_node, started by bringup_arm.sh), ROS_DOMAIN_ID=42
- **summary** 0 px nearer than 0.08 m

| quantity | value | unit |
| --- | ---: | --- |
| pixels nearer than the gate | 0 | px |
| that fraction | 0 | % |
| nearest return of all | 0.149 | m |
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
- **summary** 640 x 480 px | focus (variance of Laplacian) 417 | 0.13% of pixels clipped white, 0.00% crushed black

| quantity | value | unit |
| --- | ---: | --- |
| frame width | 640 | px |
| frame height | 480 | px |
| focus, variance of Laplacian | 417.2 | - |
| pixels clipped white | 0.133 | % |
| pixels crushed black | 0 | % |
| mean blue | 103.14 | 0-255 |
| mean green | 115.87 | 0-255 |
| mean red | 98.12 | 0-255 |
| median grey | 115 | 0-255 |
| grey std | 64.7 | 0-255 |

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
- **summary** median H 68  S 43  V 125 | 76.3% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 68 | 0-179 |
| median saturation | 43 | 0-255 |
| median value | 125 | 0-255 |
| pixels below S=80 | 76.294 | % |
| pixels below V=40 | 15.678 | % |
| pixels above V=200 | 9.814 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/scene_rs/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 13120 px selected (4.271% of the frame) | of those, median S 89 and median V 47

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 13120 | px |
| fraction of the frame | 4.2708 | % |
| median S of the selected | 89 | 0-255 |
| median V of the selected | 47 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/scene_rs/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 13120 px in, 4430 px out, 8690 px removed (66.2%) | 838 separate blobs before, 11 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 13120 | px |
| pixels after | 4430 | px |
| pixels removed | 8690 | px |
| removed | 66.235 | % |
| blobs before | 838 | count |
| blobs after | 11 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/scene_rs/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** RealSense D435i across the room, depth 480x270@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 11 components over 40 px | largest 1678 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 11 | count |
| largest area | 1678 | px |
| total area kept | 4430 | px |
| median area | 190 | px |


*every component* -- 11 rows, full data in `data/scene_rs/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/scene_rs/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** black 53204 | white 29665 | green 23688 | cyan 9891 | orange 4677 | yellow 277

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 6 | count |
| largest term | black | - |
| its pixel count | 53204 | px |


*pixels per colour term* -- 11 rows, full data in `data/scene_rs/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/scene_rs/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 4430 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 23688 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 468 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

### 08  The region-MEAN ratio test, and its near-black guard  [ok]

![region_mean](stages/scene_rs/08_region_mean.png)

- **formula** `green  <=>  mean_G > 1.25 * mean_B  and  mean_G > 1.8 * mean_R,  refused outright when max(B,G,R) < 25`
- **source** srl_perception/prompt_detector.py:_colour_matches
- **summary** 1 of 8 regions pass

| quantity | value | unit |
| --- | ---: | --- |
| regions tested | 8 | count |
| regions passing | 1 | count |
| regions rejected as near-black | 0 | count |
| blue ratio threshold | 1.25 | - |
| red ratio threshold | 1.8 | - |
| near-black floor | 25 | 0-255 |


*region means and verdicts* -- 8 rows, full data in `data/scene_rs/08_region_mean__region_means_and_verdicts.csv`

- **note** Per-pixel hue picked the room's teal robots three times: their shadowed hue is 76 against the target cube's 74, two apart and inside noise. Region MEANS separate them, because teal carries far more blue. The near-black guard exists because a region measuring (4.1, 8.8, 3.9) -- visually black -- satisfies the ratios on sensor noise alone and was reported as the cube.

### 09  Rotated box, in-image yaw, and the degeneracy gate  [ok]

![minarearect](stages/scene_rs/09_minarearect.png)

- **formula** `minAreaRect -> ((cx,cy), (w,h), angle);  yaw is published as q = (0, 0, sin(angle/2), cos(angle/2)) ONLY when max(w,h)/min(w,h) >= 1.15`
- **source** srl_perception/colour_shape_detector.py
- **summary** 36 x 67 px, 88.4 deg, aspect 1.86, yaw published | 57 x 22 px, 90.0 deg, aspect 2.59, yaw published

| quantity | value | unit |
| --- | ---: | --- |
| contours measured | 2 | count |
| aspect gate for publishing yaw | 1.15 | - |
| yaws published | 2 | count |
| yaws withheld as degenerate | 0 | count |


*rotated boxes* -- 2 rows, full data in `data/scene_rs/09_minarearect__rotated_boxes.csv`

- **note** Rotation about the OPTICAL AXIS is exactly what this measures; out-of-plane tilt is what it cannot see. The detector published identity for months while measuring the angle into a debug string, so an object at 30 deg got a square grasp and no check could tell that from a square object. A near-square blob still gets identity, because minAreaRect's angle is degenerate there and publishing it would be inventing a direction.

### 10  The size-at-range gate  [ok]

![size_at_range](stages/scene_rs/10_size_at_range.png)

- **formula** `z = fx * object_width / px_across;  accept iff 0.25 <= z <= 1.50 m.  With depth the honest form is used instead: compare px against fx * size / measured_depth, tolerance 0.55 to 1.80`
- **source** srl_perception/colour_shape_detector.py -- expected_px() and size_at_range_ok()
- **summary** 67 px -> 0.358 m accept | 57 px -> 0.423 m accept

| quantity | value | unit |
| --- | ---: | --- |
| fx used | 603.022 | px |
| assumed object width | 40 | mm |
| near limit of the window | 0.25 | m |
| far limit of the window | 1.5 | m |
| blobs tested | 2 | count |
| blobs accepted | 2 | count |
| blobs rejected | 0 | count |


*implied range per blob* -- 2 rows, full data in `data/scene_rs/10_size_at_range__implied_range_per_blob.csv`

- **note** This gate was added after a measured failure: T1's coloured pads are 210 x 130 mm in exactly the cube colours, and with a minimum-area floor and no upper bound the detector locked onto the pads and scored 0 of 4 cubes. A 210 mm pad read as a 40 mm cube implies z = 0.134 m -- 'a cube 134 mm from the lens' -- which is outside anything the arm works in, so the pad disappears while a real cube at 0.5 m passes untouched.

### 11  FastSAM: every region in the picture, no prompt  [ok]

![fastsam](stages/scene_rs/11_fastsam.png)

- **formula** `object-agnostic instance masks; no class, no prompt, no scene knowledge -- 'what are the regions', not 'where is the green cube'`
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 1.5 s for this frame
- **summary** 80 regions | largest 76896 px (25.0% of frame), smallest 78 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 80 | count |
| largest region | 76896 | px |
| largest as a fraction of the frame | 25.03 | % |
| smallest region | 78 | px |
| median region | 586 | px |
| inference time | 1.46 | s |
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
- **summary** 80 in, 80 kept, 0 too small, 0 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 80 | count |
| kept | 80 | count |
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
- **summary** 179 px at (320, 196), mean BGR [73.9, 125.4, 27.9] | 173 px at (2, 102), mean BGR [50.9, 65.5, 35.7]

| quantity | value | unit |
| --- | ---: | --- |
| colour word parsed | green | - |
| regions considered | 80 | count |
| regions matched | 2 | count |
| regions rejected on colour | 78 | count |
| best score | 0.089 | 0-1 |


*matched regions* -- 2 rows, full data in `data/scene_rs/13_segment_match__matched_regions.csv`

- **note** Naming the cube failed at every confidence -- it is ~19 px and dark, and an open-vocabulary detector cannot recognise it as anything. Segmenting it is trivial: 89 regions, exactly one green-dominant, no false positives. The MASK is the other win: a colour threshold gives whatever pixels passed, a segment gives the object's actual extent, which is what a width measurement needs.

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](stages/scene_rs/14_yoloworld.png)

- **refused** YOLO-World ran in 4.8 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

### 15  The depth frame, and where it has no answer  [ok]

![raw_depth](stages/scene_rs/15_raw_depth.png)

- **formula** `one 16-bit value per pixel, in MILLIMETRES on the wire, divided by 1000 here. 0 means 'no return' and is NOT a distance of zero.`
- **source** RealSense D435i across the room, depth 480x270@15 + colour 640x480@15, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 640 x 480 | 27.7% of pixels have a return | median 3.836 m, 5th-95th 2.997-12.454 m | 0 px between the 0.08 and 1.50 m gate the pick path uses

| quantity | value | unit |
| --- | ---: | --- |
| depth width | 640 | px |
| depth height | 480 | px |
| pixels with a return | 85061 | px |
| that fraction | 27.69 | % |
| median range | 3.8358 | m |
| 5th percentile | 2.997 | m |
| 95th percentile | 12.4542 | m |
| nearest return | 1.692 | m |
| furthest return | 65.535 | m |
| returns inside the working gate | 0 | px |
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

- **refused** only 0 depth returns fall between 0.08 and 1.50 m. The camera is not looking at anything in its working band, or the depth stream is delivering holes.

### 18  RANSAC: the support plane  [refused]

![ransac](stages/scene_rs/18_ransac.png)

- **refused** only 0 depth returns fall between 0.08 and 1.50 m. The camera is not looking at anything in its working band, or the depth stream is delivering holes.

### 19  The PCA refinement, and the NaN it once produced  [refused]

![pca_refine](stages/scene_rs/19_pca_refine.png)

- **refused** only 0 depth returns fall between 0.08 and 1.50 m. The camera is not looking at anything in its working band, or the depth stream is delivering holes.

### 20  Height above the plane  [refused]

![height_map](stages/scene_rs/20_height_map.png)

- **refused** only 0 depth returns fall between 0.08 and 1.50 m. The camera is not looking at anything in its working band, or the depth stream is delivering holes.

### 21  The surface height as a MODE, not a mean  [refused]

![mode_surface](stages/scene_rs/21_mode_surface.png)

- **refused** only 0 depth returns fall between 0.08 and 1.50 m. The camera is not looking at anything in its working band, or the depth stream is delivering holes.

### 22  What is standing on the surface  [refused]

![on_surface](stages/scene_rs/22_on_surface.png)

- **refused** only 0 aligned depth points; nothing to lift the masks through

### 23  The measurement the pick actually acts on  [refused]

![cube_measurement](stages/scene_rs/23_cube_measurement.png)

- **refused** only 0 depth returns fall between 0.08 and 1.50 m. The camera is not looking at anything in its working band, or the depth stream is delivering holes.

### 24  Splitting a cube from the pad it stands on  [refused]

![layers](stages/scene_rs/24_layers.png)

- **refused** only 0 aligned depth points; nothing to lift the masks through

### 25  Rotating calipers, against the two controls  [refused]

![min_width](stages/scene_rs/25_min_width.png)

- **refused** only 0 aligned depth points; nothing to lift the masks through

### 26  Dropping masks that are unions of finer ones  [refused]

![nested](stages/scene_rs/26_nested.png)

- **refused** only 0 aligned depth points; nothing to lift the masks through

### 27  The parallel-jaw grasp  [refused]

![grasp](stages/scene_rs/27_grasp.png)

- **refused** only 0 aligned depth points; nothing to lift the masks through

### 28  Fiducials: the primary detector  [refused]

![apriltag](stages/scene_rs/28_apriltag.png)

- **refused** no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That is an answer, not a failure: a tag either decodes or it does not, which is exactly why this is the PRIMARY detector and colour is the capped fallback. Detector used: cv2.aruco.ArucoDetector (OpenCV 4.11.0).

### 29  The support surface, as an object  [refused]

![table](stages/scene_rs/29_table.png)

- **refused** only 0 aligned depth points; nothing to lift the masks through

### 30  Every object, as an oriented 3-D box  [refused]

![boxes3d](stages/scene_rs/30_boxes3d.png)

- **refused** only 0 aligned depth points; nothing to lift the masks through

### 31  How far apart everything is  [refused]

![distances](stages/scene_rs/31_distances.png)

- **refused** only 0 aligned depth points; nothing to lift the masks through

### 32  People in the picture  [ok]

![people](stages/scene_rs/32_people.png)

- **formula** `markerless pose landmarks; each landmark's range read from the depth at its own pixel (median of a 9x9 window)`
- **source** MediaPipe PoseLandmarker (full), num_poses=4, run in .venv_pose as a separate interpreter -- the same model srl_perception/wearer_tracker_node.py uses, 2.7 s
- **summary** 1 person(s)

| quantity | value | unit |
| --- | ---: | --- |
| people detected | 1 | count |
| landmarks per person | 33 | count |
| inference time | 2.66 | s |
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
| nearest return of all | 1.692 | m |
| blobs of near return | 0 | count |
| this camera is on the hand | no | - |


*near-field blobs* -- 0 rows, full data in `data/scene_rs/33_self_view__near_field_blobs.csv`

- **note** THE ROBOT ITSELF IS NOT DETECTED IN THE IMAGE, and no stage in this program claims to. Nothing in this repository finds an arm in a picture: the robot's pose comes from its own encoders through the URDF, and drawing it into a camera image needs the camera-to-robot extrinsic -- measured at ~8.5 deg wrong on the wrist, and never measured at all for the scene cameras. On a wrist camera the near-field blobs above are usually the gripper's own fingers entering the bottom of the frame, which is also why the last ~80 mm of a grasp is completed from the last good fix rather than from live vision.

## scene_hd

HD USB webcam /dev/video6 (HD USB CAMERA: HD USB CAMERA 32e4:0317), MJPG 1280x720

- NO INTRINSICS: this camera has never been calibrated in this repository, so every stage that needs fx, fy, cx, cy refuses rather than assuming cx = w/2. Calibrate it with scripts/calibrate_scene_camera.py.

Contact sheet: `sheet_scene_hd.png`

### 01  The frame, as the camera delivered it  [ok]

![raw_colour](stages/scene_hd/01_raw_colour.png)

- **formula** `none -- this is the input every later stage is a function of`
- **source** HD USB webcam /dev/video6 (HD USB CAMERA: HD USB CAMERA 32e4:0317), MJPG 1280x720
- **summary** 1280 x 720 px | focus (variance of Laplacian) 1181 | 9.08% of pixels clipped white, 1.78% crushed black

| quantity | value | unit |
| --- | ---: | --- |
| frame width | 1280 | px |
| frame height | 720 | px |
| focus, variance of Laplacian | 1181 | - |
| pixels clipped white | 9.08 | % |
| pixels crushed black | 1.781 | % |
| mean blue | 132.25 | 0-255 |
| mean green | 132.61 | 0-255 |
| mean red | 128.22 | 0-255 |
| median grey | 138 | 0-255 |
| grey std | 90.1 | 0-255 |

- **note** NO INTRINSICS: this camera has never been calibrated in this repository, so every stage that needs fx, fy, cx, cy refuses rather than assuming cx = w/2. Calibrate it with scripts/calibrate_scene_camera.py.

### 02  Intrinsics and the ray each pixel stands for  [refused]

![intrinsics](stages/scene_hd/02_intrinsics.png)

- **refused** this camera has no intrinsics in this repository, and this stage is arithmetic on fx, fy, cx, cy. Assuming cx = w/2 is exactly the error that throws a ray by 122 px on the Kinova module -- 94 mm at 1 m, three times the grasp tolerance. Calibrate it: scripts/calibrate_scene_camera.py

### 03  BGR to HSV, the space the colour tests live in  [ok]

![hsv](stages/scene_hd/03_hsv.png)

- **formula** `V = max(B,G,R);  S = (V - min(B,G,R))/V;  H = the sector of the RGB hexagon, 0..179 in OpenCV (not 0..359)`
- **source** HD USB webcam /dev/video6 (HD USB CAMERA: HD USB CAMERA 32e4:0317), MJPG 1280x720
- **summary** median H 80  S 9  V 148 | 79.7% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 80 | 0-179 |
| median saturation | 9 | 0-255 |
| median value | 148 | 0-255 |
| pixels below S=80 | 79.675 | % |
| pixels below V=40 | 21.723 | % |
| pixels above V=200 | 33.521 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/scene_hd/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 1648 px selected (0.179% of the frame) | of those, median S 121 and median V 84

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 1648 | px |
| fraction of the frame | 0.1788 | % |
| median S of the selected | 121 | 0-255 |
| median V of the selected | 84.5 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/scene_hd/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 1648 px in, 513 px out, 1135 px removed (68.9%) | 138 separate blobs before, 1 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 1648 | px |
| pixels after | 513 | px |
| pixels removed | 1135 | px |
| removed | 68.871 | % |
| blobs before | 138 | count |
| blobs after | 1 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/scene_hd/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** HD USB webcam /dev/video6 (HD USB CAMERA: HD USB CAMERA 32e4:0317), MJPG 1280x720
- **summary** 1 components over 40 px | largest 513 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 1 | count |
| largest area | 513 | px |
| total area kept | 513 | px |
| median area | 513 | px |


*every component* -- 1 rows, full data in `data/scene_hd/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/scene_hd/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** white 301606 | black 194407 | cyan 45917 | red 9148 | green 555 | yellow 475

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 8 | count |
| largest term | white | - |
| its pixel count | 301606 | px |


*pixels per colour term* -- 11 rows, full data in `data/scene_hd/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/scene_hd/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 513 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 555 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 513 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

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
- **summary** 22 x 22 px, 90.0 deg, aspect 1.00, identity (not measured)

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
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 1.4 s for this frame
- **summary** 114 regions | largest 265469 px (28.8% of frame), smallest 53 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 114 | count |
| largest region | 265469 | px |
| largest as a fraction of the frame | 28.81 | % |
| smallest region | 53 | px |
| median region | 1485 | px |
| inference time | 1.36 | s |
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
- **summary** 114 in, 113 kept, 1 too small, 0 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 114 | count |
| kept | 113 | count |
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
- **summary** 598 px at (612, 442), mean BGR [107.9, 141.7, 74.4]

| quantity | value | unit |
| --- | ---: | --- |
| colour word parsed | green | - |
| regions considered | 114 | count |
| regions matched | 1 | count |
| regions rejected on colour | 113 | count |
| best score | 0.299 | 0-1 |


*matched regions* -- 1 rows, full data in `data/scene_hd/13_segment_match__matched_regions.csv`

- **note** Naming the cube failed at every confidence -- it is ~19 px and dark, and an open-vocabulary detector cannot recognise it as anything. Segmenting it is trivial: 89 regions, exactly one green-dominant, no false positives. The MASK is the other win: a colour threshold gives whatever pixels passed, a segment gives the object's actual extent, which is what a width measurement needs.

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](stages/scene_hd/14_yoloworld.png)

- **refused** YOLO-World ran in 4.6 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

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
- **source** MediaPipe PoseLandmarker (full), num_poses=4, run in .venv_pose as a separate interpreter -- the same model srl_perception/wearer_tracker_node.py uses, 3.0 s
- **summary** 1 person(s)

| quantity | value | unit |
| --- | ---: | --- |
| people detected | 1 | count |
| landmarks per person | 33 | count |
| inference time | 3.01 | s |
| depth attached to landmarks | no | - |


*body landmarks* -- 8 rows, full data in `data/scene_hd/32_people__body_landmarks.csv`

- **note** This is the wearer-tracking path. The rule the whole safety case rests on: a camera may only ever make the modelled body BIGGER -- fuse() keeps whichever of the tracked and mannequin primitive is CLOSER to the robot, per part, so a body measured further away changes nothing and a lost track falls back rather than shrinking the person.

### 33  The robot in the picture, and what is NOT detected  [by_design]

![self_view](stages/scene_hd/33_self_view.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

