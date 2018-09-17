<div align="center">
<!-- <img src="docs/mace-logo.png" width="400" alt="MACE" /> -->
</div>

**Horizontal Plane Extractor** (or **HoPE** for short) is a ROS package for
extracting horizontal planes given point cloud input. The planes can be the ground or a tabletop surface,
with which robotic tasks such as navigation or manipulation can be performed with ease.

## Installation
This package requests following dependences:
* ROS (tested on both Indigo and Kinetic)
* PCL 1.7
* OpenCV (tested on 2.4 and 3.3)
* Boost
* Eigen
On a fresh Ubuntu 14.04 or 16.04 System, you can install aformentioned packages by installing ros-<distro>-desktop

After these prerequests have been met, clone this repository into your /home/$USER directory, and do:
```
cd hope
mkdir build & cd build
cmake ..
make -j

```


## Performance
[MACE Model Zoo](https://github.com/XiaoMi/mace-models) contains
several common neural networks and models which will be built daily against a list of mobile
phones. The benchmark results can be found in [the CI result page](https://gitlab.com/llhe/mace-models/pipelines)
(choose the latest passed pipeline, click *release* step and you will see the benchmark results).
To get the comparison results with other frameworks, you can take a look at
[MobileAIBench](https://github.com/XiaoMi/mobile-ai-bench) project.

## Basic Usage


## Advanced Usage
Any kind of contribution is welcome. For bug reports, feature requests,
please just open an issue without any hesitation. For code contributions, it's
strongly suggested to open an issue for discussion first. For more details,
please refer to [the contribution guide](https://mace.readthedocs.io/en/latest/development/contributing.html).

## License
[Apache License 2.0](LICENSE).

## Acknowledgement
MACE depends on several open source projects located in the
[third_party](third_party) directory. Particularly, we learned a lot from
the following projects during the development:
* [Qualcomm Hexagon NN Offload Framework](https://source.codeaurora.org/quic/hexagon_nn/nnlib): the Hexagon DSP runtime
  depends on this library.
* [TensorFlow](https://github.com/tensorflow/tensorflow),
  [Caffe](https://github.com/BVLC/caffe),
  [SNPE](https://developer.qualcomm.com/software/snapdragon-neural-processing-engine-ai),
  [ARM ComputeLibrary](https://github.com/ARM-software/ComputeLibrary),
  [ncnn](https://github.com/Tencent/ncnn) and many others: we learned many best
  practices from these projects.

Finally, we also thank the Qualcomm, Pinecone and MediaTek engineering teams for
their help.
