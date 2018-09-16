# TRACKING

The goal of the project is to reverse-follow a person. To this end, the robot should be able to detect the person and then "reverse-follow" only them. It should be able to track this person, ignoring anyone else.

At each frame from the camera, our detection method (currently YoLo) outputs bounding boxes. There is a lot of noise in the detection step. Lots of false positives, could be multiple people in the scene. We don't know which one of these bounding boxes may be true. For this we require an algorithm to track our target.
Once we acquire a target at startup, There are some simple methods for detecting our target:
- Use **color and surface normal histograms** of the contents inside various detected bounding boxes and match them to that or original target detected on startup. Pick the bounding box that is closest.
- Use **feature matching**. Some algorithm like [SIFT](http://aishack.in/tutorials/sift-scale-invariant-feature-transform-introduction/). This will pick out features in an target image and try to find these features in the full image. Pick the bounding box with most matches.

These methods are simplistic and have problems. Color histograms only match color, so any other object or person with similar color in the image with give a high match. Feature extraction will work when the image is rotated, scaled or skewed, but works best with static images. When objects are moving, the perspective may change, obscuring features that were detected on startup. It may also be computationally heavy and is almost the same as running 2 detectin algorithms.

## SORT (SIMPLE ONLINE AND REALTIME TRACKING)
Another approach is to use the location of the bounding box. Given the fact that the images are from a continuous video stream, the location of the target cannot change too much. So choose the bounding box that is closest in location to the detected object. If an object is moving, we can do one better, we can estimate the velocity of the target and then choose a bounding box that is close to the predicted position. In 2017, an algorithm called [SORT](https://arxiv.org/abs/1602.00763) ([code](https://github.com/abewley/sort)) was introduced by [Alex Bewly](http://alex.bewley.ai/), et al. based on these concepts.
The algorithm works based on the detected position of the bounding boxes and uses algorithms like kalman filters to predict velocity and (). It does not account for special cases like short/long term occlusion, yet gets high scores on benchmarks.

Here are some quotes from the paper:
> appearance features beyond the detection component are ignored in tracking and only the bounding box position and size are used for both motion estimation and data association

> The state of each target is modelled as:
  x = [u, v, s, r, u,˙ v,˙ s˙]^T,
where u and v represent the horizontal and vertical pixel location of the centre of the target, while the scale s and r represent the scale (area) and the aspect ratio of the target’s bounding box respectively

> When a detection is associated to a target, the detected bounding box is used to update the target state where the velocity components are solved optimally via a Kalman filter framework [14]. If no detection is associated to the target, its state is simply predicted without correction using the linear velocity model.

#### Drawbacks of SORT
While SORT works very well to track objects, it does have it's drawbacks, specially considering our application. It works by assigning an ID to each detection that it's tracking. If the target is not occluded or otherwise lost, the tracking is excellent. However, if the target is lost, then SORT does not try to re-identify the target. A new ID will be assigned to the target when it is detected by our object detection algorithm. See this in action in the posted video [here](https://motchallenge.net/movies/ETH-Linthescher-SORT.mp4)

In our application, when our target to follow is lost, we'd like to re-acquire them, or re-identify our original target. Additional measures will need to be taken to re-identify the original target. We might use options like color histograms or feature matching as described in section 1.