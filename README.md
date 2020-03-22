# Sensor Fusion Self-Driving Car Course - Cluster with ROI

Construct Bounding-Boxes around Clound points that represents same objects and then show those clustered cloud points on a top-view.

<img width="999" alt="cluster_with_roi" src="https://user-images.githubusercontent.com/12381733/77242694-6964a180-6c44-11ea-8c74-1d1110b688ac.png">

### Project Status:

![issue_badge](https://img.shields.io/badge/build-Passing-green) ![issue_badge](https://img.shields.io/badge/UdacityRubric-Passing-green)

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Project Overview

### Most of functions in this projects are also used in this project, see more detail at there.

* [SFND_Camera_Lidar_To_Camera](https://github.com/kimsooyoung/SFND_Camera_Lidar_To_Camera)

### Main Workflow

#### 1. Implement projection from Lidar measuremnet space to Camera image plane space through Homogeneous transformation.

#### 2. Clustering projected Lidar points using specific ROI

By providing a factor `shrinkFactor` which denotes the amount of resizing in %, a smaller box is created from the original bounding box. 

```c++
vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
{
    // shrink current bounding box slightly to avoid having too many outlier points around the edges
    cv::Rect smallerBox;
    smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
    smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
    smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
    smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);
```

By applying this method, projected Lidar points are concentrated on the central area of the preceding vehicle whereas Lidar points close to the edges are ignored.

<img width="600" alt="shrinkfactor" src="https://user-images.githubusercontent.com/12381733/77242856-72567280-6c46-11ea-8f26-baae8cacb279.png">

This parameter `shrinkFactor` is mostly set to 5-10 %

In in the figure below, a set of Lidar points in the upper right corner of the green ROI that actually belong to the red truck are associated with the blue vehicle. 

<img width="596" alt="avoid-error" src="https://user-images.githubusercontent.com/12381733/77242947-5ef7d700-6c47-11ea-816e-a3d0978e7efa.png">


Few lines added in order to avoid such an errorness.

```c++
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
                // it2->lidarPoints.push_back(*it1);
                // lidarPoints.erase(it1);
                // it1--;
                // break;
            }
        } // eof loop over all bounding boxes
        if(enclosingBoxes.size() == 1){
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }
```


#### 3. showLidarTopview

Same function that already had done in previous project [SFND_Camera_Lidar_To_Camera](https://github.com/kimsooyoung/SFND_Camera_Lidar_To_Camera)

---
 ### Reference
* [Udacity Sensor Fusion Nanodegree](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313)

