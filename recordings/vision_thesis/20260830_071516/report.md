# Computer vision stages, 2026-08-30T07:15:16

Written by `scripts/cv_pickpose_visuals.py`. The arms were NOT commanded; this program has no publisher, no service client and no Kortex session.

## The arms

Read from `/real/joint_states` (745 distinct source stamps), compared with the saved ideal pick pose.

| arm | reference | joints read | worst deviation | at the pick pose |
| --- | --- | --- | --- | --- |
| left | `config/pick_pose_ideal_left.txt` | 7 of 7 | 75.32 deg | **NO** |
| right | `config/pick_pose_ideal_right.txt` | 7 of 7 | 66.63 deg | **NO** |

<details><summary>left, joint by joint</summary>

| joint | measured (deg) | pick pose (deg) | delta (deg) |
| --- | --- | --- | --- |
| joint_1 | 72.535 | 97.743 | -25.208 |
| joint_2 | 38.856 | 37.529 | +1.327 |
| joint_3 | 159.629 | 134.368 | +25.261 |
| joint_4 | -84.071 | -87.339 | +3.267 |
| joint_5 | -94.143 | -18.818 | -75.324 |
| joint_6 | -72.354 | -28.989 | -43.365 |
| joint_7 | -61.830 | -128.712 | +66.881 |

</details>

<details><summary>right, joint by joint</summary>

| joint | measured (deg) | pick pose (deg) | delta (deg) |
| --- | --- | --- | --- |
| joint_1 | 112.796 | 134.916 | -22.120 |
| joint_2 | -73.140 | -66.576 | -6.565 |
| joint_3 | 92.494 | 25.867 | +66.627 |
| joint_4 | -82.549 | -31.455 | -51.095 |
| joint_5 | 149.000 | 161.696 | -12.697 |
| joint_6 | 8.060 | 71.186 | -63.126 |
| joint_7 | 146.249 | 153.932 | -7.683 |

</details>

## left_gripper

**No frame.** after attempting the driver is running and the arm is serving RTSP, so this is a graph problem: the node is probably on a different ROS_DOMAIN_ID from this terminal (0).; ping ok, port 554 open, port 10000 open, driver running -- it still refuses: only 0 of 4 topics answered on /left_camera within 12 s (missing: c, ck, d, dk), on ROS_DOMAIN_ID=0. Either the camera driver is not up -- `bash scripts/bringup_arm.sh left` -- or this terminal is on a different domain from the one it was started on.  --- then the direct RTSP fallback also failed: no colour frame from rtsp://192.168.1.10/color. The arm must be powered and on the network, and Kinova allows only two simultaneous connections per stream.

## right_gripper

**No frame.** after attempting the driver is running and the arm is serving RTSP, so this is a graph problem: the node is probably on a different ROS_DOMAIN_ID from this terminal (0).; ping ok, port 554 open, port 10000 open, driver running -- it still refuses: only 0 of 4 topics answered on /right_camera within 12 s (missing: c, ck, d, dk), on ROS_DOMAIN_ID=0. Either the camera driver is not up -- `bash scripts/bringup_arm.sh right` -- or this terminal is on a different domain from the one it was started on.  --- then the direct RTSP fallback also failed: no colour frame from rtsp://192.168.1.9/color. The arm must be powered and on the network, and Kinova allows only two simultaneous connections per stream.

## scene_rs

RealSense D435i across the room, depth 480x270@6 + colour 640x480@6, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.

- rotated 180 deg: the camera is mounted upside down and the principal point is rotated WITH the image (cx'=W-1-cx)
- liveness: 12 frames, 12 distinct depth images, 12 distinct frame numbers, 28% of pixels valid

Contact sheet: `sheet_scene_rs.png`

### 01  The frame, as the camera delivered it  [ok]

![raw_colour](stages/scene_rs/01_raw_colour.png)

- **formula** `none -- this is the input every later stage is a function of`
- **source** RealSense D435i across the room, depth 480x270@6 + colour 640x480@6, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 640 x 480 px | focus (variance of Laplacian) 425 | 0.18% of pixels clipped white, 0.00% crushed black

| quantity | value | unit |
| --- | ---: | --- |
| frame width | 640 | px |
| frame height | 480 | px |
| focus, variance of Laplacian | 425.5 | - |
| pixels clipped white | 0.181 | % |
| pixels crushed black | 0 | % |
| mean blue | 105.67 | 0-255 |
| mean green | 116.53 | 0-255 |
| mean red | 100.44 | 0-255 |
| median grey | 117 | 0-255 |
| grey std | 65.41 | 0-255 |

- **note** rotated 180 deg: the camera is mounted upside down and the principal point is rotated WITH the image (cx'=W-1-cx); liveness: 12 frames, 12 distinct depth images, 12 distinct frame numbers, 28% of pixels valid

### 02  Intrinsics and the ray each pixel stands for  [ok]

![intrinsics](stages/scene_rs/02_intrinsics.png)

- **formula** `bearing = ((u - cx)/fx, (v - cy)/fy);  X = bearing * Z`
- **source** RealSense D435i across the room, depth 480x270@6 + colour 640x480@6, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
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
- **source** RealSense D435i across the room, depth 480x270@6 + colour 640x480@6, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** median H 69  S 39  V 126 | 83.7% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 69 | 0-179 |
| median saturation | 39 | 0-255 |
| median value | 126 | 0-255 |
| pixels below S=80 | 83.669 | % |
| pixels below V=40 | 16.058 | % |
| pixels above V=200 | 10.189 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/scene_rs/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 3099 px selected (1.009% of the frame) | of those, median S 88 and median V 47

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 3099 | px |
| fraction of the frame | 1.0088 | % |
| median S of the selected | 88 | 0-255 |
| median V of the selected | 47 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/scene_rs/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 3099 px in, 359 px out, 2740 px removed (88.4%) | 412 separate blobs before, 2 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 3099 | px |
| pixels after | 359 | px |
| pixels removed | 2740 | px |
| removed | 88.416 | % |
| blobs before | 412 | count |
| blobs after | 2 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/scene_rs/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** RealSense D435i across the room, depth 480x270@6 + colour 640x480@6, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 2 components over 40 px | largest 190 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 2 | count |
| largest area | 190 | px |
| total area kept | 359 | px |
| median area | 179.5 | px |


*every component* -- 2 rows, full data in `data/scene_rs/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/scene_rs/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** black 54726 | white 30945 | cyan 10617 | green 5359 | orange 4776 | yellow 128

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 6 | count |
| largest term | black | - |
| its pixel count | 54726 | px |


*pixels per colour term* -- 11 rows, full data in `data/scene_rs/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/scene_rs/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 359 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 5359 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 412 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

### 08  The region-MEAN ratio test, and its near-black guard  [ok]

![region_mean](stages/scene_rs/08_region_mean.png)

- **formula** `green  <=>  mean_G > 1.25 * mean_B  and  mean_G > 1.8 * mean_R,  refused outright when max(B,G,R) < 25`
- **source** srl_perception/prompt_detector.py:_colour_matches
- **summary** 1 of 2 regions pass

| quantity | value | unit |
| --- | ---: | --- |
| regions tested | 2 | count |
| regions passing | 1 | count |
| regions rejected as near-black | 0 | count |
| blue ratio threshold | 1.25 | - |
| red ratio threshold | 1.8 | - |
| near-black floor | 25 | 0-255 |


*region means and verdicts* -- 2 rows, full data in `data/scene_rs/08_region_mean__region_means_and_verdicts.csv`

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
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 7.3 s for this frame
- **summary** 79 regions | largest 76164 px (24.8% of frame), smallest 78 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 79 | count |
| largest region | 76164 | px |
| largest as a fraction of the frame | 24.79 | % |
| smallest region | 78 | px |
| median region | 579 | px |
| inference time | 7.33 | s |
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
- **summary** 79 in, 79 kept, 0 too small, 0 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 79 | count |
| kept | 79 | count |
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
- **summary** 179 px at (320, 196), mean BGR [76.9, 127.0, 29.4]

| quantity | value | unit |
| --- | ---: | --- |
| colour word parsed | green | - |
| regions considered | 79 | count |
| regions matched | 1 | count |
| regions rejected on colour | 78 | count |
| best score | 0.089 | 0-1 |


*matched regions* -- 1 rows, full data in `data/scene_rs/13_segment_match__matched_regions.csv`

- **note** Naming the cube failed at every confidence -- it is ~19 px and dark, and an open-vocabulary detector cannot recognise it as anything. Segmenting it is trivial: 89 regions, exactly one green-dominant, no false positives. The MASK is the other win: a colour threshold gives whatever pixels passed, a segment gives the object's actual extent, which is what a width measurement needs.

### 14  YOLO-World: open-vocabulary naming  [refused]

![yoloworld](stages/scene_rs/14_yoloworld.png)

- **refused** YOLO-World ran in 14.7 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

### 15  The depth frame, and where it has no answer  [ok]

![raw_depth](stages/scene_rs/15_raw_depth.png)

- **formula** `one 16-bit value per pixel, in MILLIMETRES on the wire, divided by 1000 here. 0 means 'no return' and is NOT a distance of zero.`
- **source** RealSense D435i across the room, depth 480x270@6 + colour 640x480@6, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 640 x 480 | 27.9% of pixels have a return | median 3.808 m, 5th-95th 2.974-12.689 m | 73768 px between the 0.15 and 8.00 m gate the pick path uses

| quantity | value | unit |
| --- | ---: | --- |
| depth width | 640 | px |
| depth height | 480 | px |
| pixels with a return | 85727 | px |
| that fraction | 27.91 | % |
| median range | 3.8075 | m |
| 5th percentile | 2.974 | m |
| 95th percentile | 12.689 | m |
| nearest return | 0.36 | m |
| furthest return | 65.535 | m |
| returns inside this camera's working band | 73768 | px |
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
- **source** rgbd_grasp.deproject / servo_pick_left.measure, with fx 603.02 fy 603.13 cx 320.13 cy 247.78
- **summary** 73768 points in the 0.15-8.00 m band | extent X -3.010 to 1.759, Y -1.988 to 2.992, Z 0.360 to 7.996 m

| quantity | value | unit |
| --- | ---: | --- |
| points lifted | 73768 | count |
| fx | 603.022 | px |
| fy | 603.129 | px |
| cx | 320.125 | px |
| cy | 247.782 | px |
| X minimum | -3.0097 | m |
| X maximum | 1.7588 | m |
| Y minimum | -1.9883 | m |
| Y maximum | 2.9925 | m |
| Z minimum | 0.36 | m |
| Z maximum | 7.9962 | m |
| centroid X | -0.1199 | m |
| centroid Y | -0.0875 | m |
| centroid Z | 3.7266 | m |

- **note** This is the whole of 'depth decides WHERE'. Everything after it is geometry on these points, in the CAMERA's frame -- no robot transform is involved yet, which is exactly why the pick measures its error here.

### 18  RANSAC: the support plane  [ok]

![ransac](stages/scene_rs/18_ransac.png)

- **formula** `draw 3 points -> n = (b-a) x (c-a) / |...|,  d = -n.a;  score = #{ |n.P + d| < 6 mm };  keep the best of 400 draws`
- **source** scripts/servo_pick_left.py:fit_plane, imported; fitted on 60000 of 73768 points
- **summary** normal (0.6491, -0.0495, -0.7591), offset 2.6383 m | 3494 of 73768 points are inliers (4.7%) | RMS 3.42 mm | 40.6 deg between the plane normal and the camera's optical axis

| quantity | value | unit |
| --- | ---: | --- |
| normal x | 0.6491 | - |
| normal y | -0.0495 | - |
| normal z | -0.7591 | - |
| offset d | 2.6383 | m |
| points fitted | 60000 | count |
| points scored | 73768 | count |
| inliers | 3494 | count |
| inlier fraction | 4.74 | % |
| residual RMS | 3.416 | mm |
| inlier tolerance | 6 | mm |
| RANSAC iterations | 400 | count |
| angle to the optical axis | 40.61 | deg |

- **note** A plane with too few inliers is not a plane. The pick refuses below 200 inliers rather than returning a geometry whose RMS is NaN -- everything downstream would treat that as a fix and the arm would move on it.

### 19  The PCA refinement, and the NaN it once produced  [ok]

![pca_refine](stages/scene_rs/19_pca_refine.png)

- **formula** `C = cov(Q - mean(Q)) for inliers Q;  n = eigvec(C)[:, argmin];  d = -n . mean(Q)   <-- BOTH halves, together`
- **source** scripts/servo_pick_left.py:fit_plane, imported
- **summary** the normal moved 0.000 deg | RMS 3.42 mm -> 3.42 mm | mixing the refined normal with the RANSAC offset selects 3494 inliers instead of 3494

| quantity | value | unit |
| --- | ---: | --- |
| normal moved | 0 | deg |
| RMS before refinement | 3.416 | mm |
| RMS after refinement | 3.416 | mm |
| inliers with the matched pair | 3494 | count |
| inliers with a mixed normal and offset | 3494 | count |
| inliers lost by mixing | 0 | count |

- **note** That last number is the bug this stage exists to show. The function once returned the REFINED normal with the OLD offset; those describe different planes, the inlier mask could select ZERO points, and mean() of an empty slice is NaN. The pick printed 'plane RMS nan mm' and 'FOUND' in the same breath, then closed 540 mm blind on a centre computed against that plane.

### 20  Height above the plane  [ok]

![height_map](stages/scene_rs/20_height_map.png)

- **formula** `h(P) = n . P + d,  signed;  positive is off the surface, toward the camera`
- **source** servo_pick_left.measure -- the same expression the live pick uses to separate the cube from the table
- **summary** 23185 of 73768 points stand more than 5 mm off the plane (31.43%) | tallest 2267.5 mm

| quantity | value | unit |
| --- | ---: | --- |
| height threshold | 5 | mm |
| points above it | 23185 | count |
| that fraction | 31.43 | % |
| tallest point | 2268 | mm |
| lowest point | -5039 | mm |
| median height | -47.779 | mm |

- **note** The floor is 5 mm because the plane RMS is under 1 mm on this sensor. Set it below the depth noise and the table itself becomes an object; set it far above and a flat item is missed.

### 21  The surface height as a MODE, not a mean  [ok]

![mode_surface](stages/scene_rs/21_mode_surface.png)

- **formula** `bin at 2 mm; take the fullest bin; require it to hold >= 20% of the points; refine as the mean of everything within +/-6 mm of it`
- **source** srl_perception/surface_from_depth.py, applied along the fitted plane normal because this program has no camera-to-world transform; the node applies it to world z
- **summary** modal bin holds 0.9% of 73768 points (floor 20%) | refined -2.6426 m, spread 3.44 mm | a plain MEAN would say -2.9025 m, which is 259.9 mm out | RANSAC's own offset is -2.6383 m, 4.31 mm from this

| quantity | value | unit |
| --- | ---: | --- |
| bin width | 2 | mm |
| points binned | 73768 | count |
| share in the fullest bin | 0.87 | % |
| share required | 20 | % |
| modal bin centre | -2.6426 | m |
| refined estimate | -2.6426 | m |
| spread of the inliers | 3.442 | mm |
| inlier band | 6 | mm |
| a plain mean would say | -2.9025 | m |
| that mean's error | 259.89 | mm |
| RANSAC's own offset | -2.6383 | m |
| the two estimators differ by | 4.315 | mm |

- **note** Two independent estimators of one surface, so they can be compared: RANSAC votes on inliers, this votes on the fullest histogram bin. A mean is dragged up by everything standing on the table and down by every hole, and it MOVES as the objects move -- the 'measurement' would change between trials on a motionless table.

### 22  What is standing on the surface  [ok]

![on_surface](stages/scene_rs/22_on_surface.png)

- **formula** `{ pixels whose lifted point has h > 5 mm }, 8-connected`
- **source** the plane of stage 18 fitted in the colour frame; heights from the aligned depth of stage 16
- **summary** 7500 px, 7291 pts, top 456 mm | 6531 px, 6467 pts, top 162 mm | 3307 px, 3262 pts, top 224 mm | 2059 px, 2016 pts, top 408 mm | 1699 px, 1684 pts, top 1414 mm | 494 px, 470 pts, top 89 mm

| quantity | value | unit |
| --- | ---: | --- |
| pieces standing on the surface | 21 | count |
| height threshold | 5 | mm |
| largest piece | 7500 | px |
| tallest piece | 2268 | mm |


*pieces above the plane* -- 21 rows, full data in `data/scene_rs/22_on_surface__pieces_above_the_plane.csv`

- **note** This is the pre-2020 method on its own -- threshold above a plane, then cluster what is left. It is exactly what fails when two 40 mm cubes sit 20 mm apart: connectivity fuses them into one object 140 mm across. Compare this figure with stage 11.

### 23  The measurement the pick actually acts on  [refused]

![cube_measurement](stages/scene_rs/23_cube_measurement.png)

- **refused** only 0 points are both green-coloured AND more than 5 mm above the plane (the pick needs 60 and returns None below that). 190 points are coloured but ON the plane; 23185 stand off the plane but are not the colour.

### 24  Splitting a cube from the pad it stands on  [ok]

![layers](stages/scene_rs/24_layers.png)

- **formula** `find the MODAL height bin (5 mm bins); cut 12 mm above it; recurse on what is left, twice`
- **source** srl_perception/segment_lift.py:_layers
- **summary** layer 1: 4-91 mm, 4317 points | layer 2: 91-103 mm, 514 points | layer 3: 103-1325 mm, 3708 points

| quantity | value | unit |
| --- | ---: | --- |
| region area | 76164 | px |
| points lifted from it | 19853 | count |
| points above the plane | 8539 | count |
| pixels removed by the 1 px erosion | 4970 | px |
| layers found | 3 | count |
| lowest point | 4.02 | mm |
| highest point | 1325 | mm |


*height layers* -- 3 rows, full data in `data/scene_rs/24_layers__height_layers.csv`

- **note** Appearance decides what is SIDE BY SIDE; height decides what is STACKED. A blue cube on a blue pad is correctly ONE region -- same colour, touching, no edge between them. And the split cannot look for an empty band, because there is none: the cube RESTS on the pad, so its sides fill every bin. What is there is a MODE -- the support's top face is a large flat area and dominates the histogram.

### 25  Rotating calipers, against the two controls  [ok]

![min_width](stages/scene_rs/25_min_width.png)

- **formula** `width = min over theta of [ max(p.u(theta)) - min(p.u(theta)) ],  u(theta) = (cos, sin) in the support plane, swept at 0.25 deg`
- **source** srl_perception/segment_lift.py:_min_width and rgbd_grasp.min_width_frame
- **summary** calipers 1737.5 mm | PCA's short axis 1821.3 mm | axis-aligned box 1817.8 mm | long axis 2621.3 mm, height 1320.6 mm | jaws close on 85 mm, so this is TOO WIDE

| quantity | value | unit |
| --- | ---: | --- |
| instance points | 8539 | count |
| minimum width, rotating calipers | 1737 | mm |
| at this angle in the plane | 76 | deg |
| PCA short axis, THE CONTROL | 1821 | mm |
| axis-aligned box, THE CONTROL | 1818 | mm |
| PCA overstates by | 83.86 | mm |
| long axis | 2621 | mm |
| height | 1321 | mm |
| jaw span | 85 | mm |
| graspable as it lies | no | - |
| sweep step | 0.5 | deg |

- **note** The two controls are here because both were used and both were wrong. The principal axes of a SQUARE are degenerate -- equal variance in both directions -- so SVD returns the diagonal and a 40 mm cube measures 40*sqrt(2) = 53 mm; against an 85 mm jaw that refuses a 60 mm object as ungraspable. The error that matters is not the 13 mm, it is the DIRECTION: a jaw told to close across a diagonal approaches the corner and the object rolls out.

### 26  Dropping masks that are unions of finer ones  [ok]

![nested](stages/scene_rs/26_nested.png)

- **formula** `voxelise each instance at 5 mm; accept smallest first; reject one whose claimed-voxel overlap exceeds 0.55`
- **source** srl_perception/segment_lift.py:_suppress_nested
- **summary** 31 instances in, 23 kept, 8 dropped | overlaps: 1.00, 1.00, 0.78, 0.90, 0.85, 0.87

| quantity | value | unit |
| --- | ---: | --- |
| instances in | 31 | count |
| kept | 23 | count |
| dropped as already explained | 8 | count |
| voxel size | 5 | mm |
| overlap fraction that rejects | 0.55 | - |


*per instance* -- 31 rows, full data in `data/scene_rs/26_nested__per_instance.csv`

- **note** FastSAM returns a HIERARCHY, not a partition: for one picture it hands back the cube, the pad, and a region covering both. All three are legitimate. Lifted, they became three objects in the same space and the merge absorbed a 40 mm cube -- measured EXACT at 40.3 mm against a 40 mm truth -- into a 148 mm blob that was then refused as ungraspable. Volume, not pixels: two masks that overlap in the image can be a metre apart in depth.

### 27  The parallel-jaw grasp  [refused]

![grasp](stages/scene_rs/27_grasp.png)

- **refused** the grasp planner refused, which is an ANSWER: object is 1696 mm across its narrowest axis; the Robotiq 85 opens 75 mm usable. It cannot be grasped as it lies.

### 28  Fiducials: the primary detector  [refused]

![apriltag](stages/scene_rs/28_apriltag.png)

- **refused** no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That is an answer, not a failure: a tag either decodes or it does not, which is exactly why this is the PRIMARY detector and colour is the capped fallback. Detector used: cv2.aruco.ArucoDetector (OpenCV 4.11.0).

### 29  The support surface, as an object  [ok]

![table](stages/scene_rs/29_table.png)

- **formula** `fit the plane, keep its inliers, span them in the plane's own basis; extent taken at the 2nd and 98th percentile so one stray return cannot stretch it`
- **source** the plane of stage 18, expressed as a rectangle in the camera frame
- **summary** 1.725 x 1.735 m of surface, 2.638 m away, flat to 3.42 mm

| quantity | value | unit |
| --- | ---: | --- |
| points on the surface | 3494 | count |
| surface extent along its first axis | 1.7251 | m |
| surface extent along its second axis | 1.7353 | m |
| area spanned | 2.9935 | m2 |
| perpendicular distance from the camera | 2.6383 | m |
| distance to the middle of that extent | 3.5586 | m |
| nearest point on the surface | 2.7232 | m |
| furthest point on the surface | 4.4824 | m |
| tilt from the optical axis | 40.61 | deg |
| flatness, RMS of the inliers | 3.416 | mm |

- **note** This is a MEASURED surface, not the declared one. srl_experiments/work_surface.py has had set_measured() since it was written and nothing ever called it, so a table 20 mm out would have been absorbed silently into every grasp.

### 30  Every object, as an oriented 3-D box  [ok]

![boxes3d](stages/scene_rs/30_boxes3d.png)

- **formula** `centre = mid-extent ACROSS the surface + (foot + top/2) ALONG the normal; axes = (minimum-width direction, its perpendicular, the plane normal); size = the spans along them`
- **source** segment_lift._measure for the horizontal centre and rgbd_grasp.shell_corrected_centre's reasoning for the vertical -- the combination is checked to 2 mm against a constructed cube in --self-test
- **summary** 12 objects, 2.634 to 4.291 m away

| quantity | value | unit |
| --- | ---: | --- |
| objects posed | 12 | count |
| nearest object | 2.6336 | m |
| furthest object | 4.2909 | m |
| largest footprint | 2621 | mm |
| tallest object | 1325 | mm |
| graspable within the 85 mm jaw | 6 | count |


*every object, in the camera frame* -- 12 rows, full data in `data/scene_rs/30_boxes3d__every_object__in_the_camera_frame.csv`

- **note** These coordinates are in the CAMERA's frame. Putting them in the robot's frame needs the camera extrinsic, which on the wrist is measured at ~8.5 deg wrong and rotates with the joint -- 34 mm of error at 0.23 m. That is why the pick servos on an error measured HERE rather than planning through that transform.

### 31  How far apart everything is  [ok]

![distances](stages/scene_rs/31_distances.png)

- **formula** `range = |centre| (the camera is the origin); height = n.c + d; separation = |c_i - c_j|; free gap subtracts each object's half-width`
- **source** the poses of stage 30, all in the camera's own frame
- **summary** 10 objects, nearest 2.634 m, closest pair 0.002 m

| quantity | value | unit |
| --- | ---: | --- |
| objects measured | 10 | count |
| nearest object | 2.6336 | m |
| furthest object | 4.2909 | m |
| closest pair | 0.0015 | m |
| tightest free gap | 0 | m |
| what the origin IS | the scene camera across the room | - |


*each object* -- 10 rows, full data in `data/scene_rs/31_distances__each_object.csv`


*between objects* -- 45 rows, full data in `data/scene_rs/31_distances__between_objects.csv`

- **note** On a WRIST camera the origin is the hand, so 'range' is already the distance the arm has to close -- that is the quantity the servo loop nulls. On the scene camera it is the distance from the camera, and turning that into a distance from the ARM needs the scene-camera-to-robot calibration, which this repository does not have measured.

### 32  People in the picture  [ok]

![people](stages/scene_rs/32_people.png)

- **formula** `markerless pose landmarks; each landmark's range read from the depth at its own pixel (median of a 9x9 window)`
- **source** MediaPipe PoseLandmarker (full), num_poses=4, run in .venv_pose as a separate interpreter -- the same model srl_perception/wearer_tracker_node.py uses, 5.7 s
- **summary** 1 person(s)

| quantity | value | unit |
| --- | ---: | --- |
| people detected | 1 | count |
| landmarks per person | 33 | count |
| inference time | 5.73 | s |
| depth attached to landmarks | yes | - |


*body landmarks* -- 8 rows, full data in `data/scene_rs/32_people__body_landmarks.csv`

- **note** This is the wearer-tracking path. The rule the whole safety case rests on: a camera may only ever make the modelled body BIGGER -- fuse() keeps whichever of the tracked and mannequin primitive is CLOSER to the robot, per part, so a body measured further away changes nothing and a lost track falls back rather than shrinking the person.

### 33  The robot in the picture, and what is NOT detected  [ok]

![self_view](stages/scene_rs/33_self_view.png)

- **formula** `{ pixels with 0 < depth < 0.15 m } -- nearer than anything this camera works on`
- **source** RealSense D435i across the room, depth 480x270@6 + colour 640x480@6, USB 2.1, negotiated against what the device reports it has. Depth is aligned to colour by librealsense and averaged over 12 frames, valid pixels only.
- **summary** 0 px nearer than 0.15 m

| quantity | value | unit |
| --- | ---: | --- |
| pixels nearer than the gate | 0 | px |
| that fraction | 0 | % |
| nearest return of all | 0.36 | m |
| blobs of near return | 0 | count |
| this camera is on the hand | no | - |
| near edge of this camera's band | 0.15 | m |


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
- **summary** 1280 x 720 px | focus (variance of Laplacian) 1162 | 9.15% of pixels clipped white, 1.91% crushed black

| quantity | value | unit |
| --- | ---: | --- |
| frame width | 1280 | px |
| frame height | 720 | px |
| focus, variance of Laplacian | 1162 | - |
| pixels clipped white | 9.148 | % |
| pixels crushed black | 1.907 | % |
| mean blue | 132.6 | 0-255 |
| mean green | 132.38 | 0-255 |
| mean red | 129.2 | 0-255 |
| median grey | 138 | 0-255 |
| grey std | 89.86 | 0-255 |

- **note** NO INTRINSICS: this camera has never been calibrated in this repository, so every stage that needs fx, fy, cx, cy refuses rather than assuming cx = w/2. Calibrate it with scripts/calibrate_scene_camera.py.

### 02  Intrinsics and the ray each pixel stands for  [refused]

![intrinsics](stages/scene_hd/02_intrinsics.png)

- **refused** this camera has no intrinsics in this repository, and this stage is arithmetic on fx, fy, cx, cy. Assuming cx = w/2 is exactly the error that throws a ray by 122 px on the Kinova module -- 94 mm at 1 m, three times the grasp tolerance. Calibrate it: scripts/calibrate_scene_camera.py

### 03  BGR to HSV, the space the colour tests live in  [ok]

![hsv](stages/scene_hd/03_hsv.png)

- **formula** `V = max(B,G,R);  S = (V - min(B,G,R))/V;  H = the sector of the RGB hexagon, 0..179 in OpenCV (not 0..359)`
- **source** HD USB webcam /dev/video6 (HD USB CAMERA: HD USB CAMERA 32e4:0317), MJPG 1280x720
- **summary** median H 75  S 6  V 148 | 81.3% of pixels are below S=80 and cannot satisfy any colour term in this repository

| quantity | value | unit |
| --- | ---: | --- |
| median hue | 75 | 0-179 |
| median saturation | 6 | 0-255 |
| median value | 148 | 0-255 |
| pixels below S=80 | 81.345 | % |
| pixels below V=40 | 21.447 | % |
| pixels above V=200 | 33.336 | % |

- **note** Hue is why colour is done here and not in BGR: brightness moves B, G and R together and leaves H alone, so a shadow does not change what colour something is -- until V collapses, which is what the value floors in each term exist to catch.

### 04  The pick path's own green threshold  [ok]

![green_threshold](stages/scene_hd/04_green_threshold.png)

- **formula** `mask(u,v) = 1 iff  40 <= H <= 85  and  80 <= S <= 255  and  40 <= V <= 255`
- **source** constants from scripts/servo_pick_left.py:measure -- the thresholds the live pick actually runs on
- **summary** 1343 px selected (0.146% of the frame) | of those, median S 122 and median V 92

| quantity | value | unit |
| --- | ---: | --- |
| hue low | 40 | 0-179 |
| hue high | 85 | 0-179 |
| saturation floor | 80 | 0-255 |
| value floor | 40 | 0-255 |
| pixels selected | 1343 | px |
| fraction of the frame | 0.1457 | % |
| median S of the selected | 122 | 0-255 |
| median V of the selected | 92 | 0-255 |

- **note** A threshold is a hard decision with no confidence attached, which is why colour is a FALLBACK in this repository and capped below any AprilTag: it fails by being confidently wrong under changed light, and nothing downstream can tell.

### 05  What the 9x9 opening removes  [ok]

![morphology](stages/scene_hd/05_morphology.png)

- **formula** `open(M) = dilate(erode(M, K), K),  K = ones(9, 9).  Erode deletes anything thinner than the kernel; dilate restores what survived to its original size.`
- **source** 9x9 is the pick path's kernel (servo_pick_left); prompt_detector and colour_shape_detector both use 5x5
- **summary** 1343 px in, 500 px out, 843 px removed (62.8%) | 113 separate blobs before, 1 after

| quantity | value | unit |
| --- | ---: | --- |
| kernel | 9x9 | px |
| pixels before | 1343 | px |
| pixels after | 500 | px |
| pixels removed | 843 | px |
| removed | 62.77 | % |
| blobs before | 113 | count |
| blobs after | 1 | count |

- **note** The kernel size IS a decision about the smallest object that can survive: at this range a 9 px feature is deleted whatever colour it is.

### 06  Connected components, and their statistics  [ok]

![components](stages/scene_hd/06_components.png)

- **formula** `8-connectivity flood label; per label: area, bounding box, centroid = (1/A) * sum of member pixel coordinates`
- **source** HD USB webcam /dev/video6 (HD USB CAMERA: HD USB CAMERA 32e4:0317), MJPG 1280x720
- **summary** 1 components over 40 px | largest 500 px | this is where a MASK becomes a set of candidate OBJECTS

| quantity | value | unit |
| --- | ---: | --- |
| components over 40 px | 1 | count |
| largest area | 500 | px |
| total area kept | 500 | px |
| median area | 500 | px |


*every component* -- 1 rows, full data in `data/scene_hd/06_components__every_component.csv`

- **note** A centroid is not an object's centre: it is the centre of the pixels that happened to pass the threshold, so a partly shadowed cube reports a centroid shifted into its lit half.

### 07  The whole colour vocabulary, swept  [ok]

![vocabulary](stages/scene_hd/07_vocabulary.png)

- **formula** `one inRange per band, OR-ed across bands (red needs two: it wraps the hue circle at 0/179), then a 5x5 opening`
- **source** srl_perception/prompt_detector.py:COLOUR_TERMS -- the colour backend's entire vocabulary, stated so its limit is visible rather than discovered
- **summary** white 299807 | black 191057 | cyan 45796 | red 10137 | green 535 | yellow 440

| quantity | value | unit |
| --- | ---: | --- |
| colour terms tested | 11 | count |
| terms with any pixels | 8 | count |
| largest term | white | - |
| its pixel count | 299807 | px |


*pixels per colour term* -- 11 rows, full data in `data/scene_hd/07_vocabulary__pixels_per_colour_term.csv`


*the three definitions of green* -- 3 rows, full data in `data/scene_hd/07_vocabulary__the_three_definitions_of_green.csv`

- **note** THE THREE GREENS ARE NOT THE SAME: servo_pick_left (the live pick) H40-85 S>=80 V>=40 open 9: 500 px; prompt_detector.COLOUR_TERMS H36-85 S>=80 V>=25 open 5: 535 px; colour_shape_detector.DEFAULT H40-85 S>=80 V>=60 open 5: 505 px. They differ in the value floor (25 / 40 / 60) and in the opening kernel, so the same cube in the same frame gives three different pixel counts depending on which module is asking.

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
- **source** FastSAM-s.pt (~11M parameters) via ultralytics, imgsz=1024 conf=0.25 iou=0.7 on the cpu, 2.5 s for this frame
- **summary** 109 regions | largest 239438 px (26.0% of frame), smallest 61 px

| quantity | value | unit |
| --- | ---: | --- |
| regions returned | 109 | count |
| largest region | 239438 | px |
| largest as a fraction of the frame | 25.98 | % |
| smallest region | 61 | px |
| median region | 1388 | px |
| inference time | 2.47 | s |
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
- **summary** 109 in, 109 kept, 0 too small, 0 too large

| quantity | value | unit |
| --- | ---: | --- |
| regions in | 109 | count |
| kept | 109 | count |
| rejected as noise | 0 | count |
| rejected as too large | 0 | count |
| lower bound | 60 | px |
| upper bound | 322560 | px |
| upper bound as a fraction | 0.35 | - |

- **note** The upper bound is not tidiness. A segmenter returns the table and the room as legitimate regions, and a region covering a third of the frame is never a thing to be picked up.

### 13  Segment everything, then pick the one that matches  [ok]

![segment_match](stages/scene_hd/13_segment_match.png)

- **formula** `for each region: mean BGR over the MASK, then the ratio test of stage 08; score = min(1, area/2000)`
- **source** srl_perception/prompt_detector.py:_segment -- the backend that actually found the target on this rig
- **summary** 598 px at (612, 442), mean BGR [109.2, 141.7, 75.9]

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

- **refused** YOLO-World ran in 11.8 s over classes ['green cube'] at conf 0.05 and found NOTHING. This is the measured behaviour on this rig, not a misconfiguration: it finds a person at 0.725 and a laptop at 0.739 in the same room and cannot see the small dark cube at any confidence. It is why the backend order is segment first, naming second, colour as the floor.

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
- **source** MediaPipe PoseLandmarker (full), num_poses=4, run in .venv_pose as a separate interpreter -- the same model srl_perception/wearer_tracker_node.py uses, 4.7 s
- **summary** 1 person(s)

| quantity | value | unit |
| --- | ---: | --- |
| people detected | 1 | count |
| landmarks per person | 33 | count |
| inference time | 4.66 | s |
| depth attached to landmarks | no | - |


*body landmarks* -- 8 rows, full data in `data/scene_hd/32_people__body_landmarks.csv`

- **note** This is the wearer-tracking path. The rule the whole safety case rests on: a camera may only ever make the modelled body BIGGER -- fuse() keeps whichever of the tracked and mannequin primitive is CLOSER to the robot, per part, so a body measured further away changes nothing and a lost track falls back rather than shrinking the person.

### 33  The robot in the picture, and what is NOT detected  [by_design]

![self_view](stages/scene_hd/33_self_view.png)

- **by_design** the HD USB webcam is a colour camera; it has no depth sensor. This is a by-design absence, not a fault -- the RealSense is the depth scene camera.

