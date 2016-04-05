# DaVinci
This project registers stl's to the daVinci robot using monocular vision

## Code overview
The most important pieces of code are
- 'vtk_registration/src/tools/camCalibration.py'
    - Running this will generate camera intrinsic and distortion matrices
- 'vtk_registration/src/tools/daVinciCalibration.py'
    - Running this will use the camera data generated by the function above to register the robot to the camera
- 'vtk_registration/src/vtk_registration.py'
    - Running this will use both of the files generated above and allows you to register an STL to the daVinci robot
