rosbuild_add_executable(main2 Main.cpp)
target_link_libraries(main2 Detector SegmentationProcessor Identifier Transformer EKF MultiscaleHog)

