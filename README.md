# Panoroma

A from-scratch C++ implementation of panoramic image stitching, following [Automatic Panoramic Image Stitching using Invariant Features](https://link.springer.com/article/10.1007/s11263-006-0002-3) (Brown & Lowe, IJCV 2007) — with **no OpenCV dependency**. Everything from the matrix containers to SIFT itself is implemented in this repository; the only third-party code is the header-only [CImg](https://cimg.eu/) library used for image loading and display.

## What's Implemented

- **Matrix/image containers** (`Utils/Mat2d`, `Mat3d`, `MatXd`): templated dense matrices with arithmetic, written from scratch
- **Image processing** (`Utils/imgproc`): grayscale conversion, Gaussian blur, up/down-sampling, drawing utilities
- **SIFT** (`SIFT.cpp`, `GaussianPyramid.cpp`): scale-space extrema detection over a Gaussian pyramid, keypoint refinement, orientation assignment, and 128-D descriptor extraction
- **Feature matching** (`main.cpp`): brute-force descriptor matching with Lowe's ratio test (0.75)
- **RANSAC homography** (`RANSAC.cpp`): robust homography estimation from the putative matches, re-estimated on all inliers
- **Warping & compositing**: the second image is warped into the first image's frame and composited into the output panorama

The full Brown & Lowe pipeline also includes bundle adjustment, gain compensation, and multi-band blending for many-image panoramas — this project focuses on the core two-image geometry (SIFT → ratio-test matching → RANSAC homography → warp), with simple overlay compositing. Anti-aliased blending is left as a TODO.

## Build & Run

Open `Panoroma.xcodeproj` in Xcode (the code is plain C++14, so it can also be compiled directly with any modern compiler — just add the `Panoroma/` and `Panoroma/Utils` sources and include `3rdparty/CImg.h`).

The demo in `main.cpp` stitches the bundled `test1.jpeg` / `test2.jpeg` pair: it visualizes the SIFT matches between the two views, prints the estimated homography, and shows the stitched result.

## Notes

- CImg's display backend requires X11 on Linux/macOS (GDI on Windows).
- The current pipeline operates on grayscale images.
