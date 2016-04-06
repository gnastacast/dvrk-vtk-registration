# DaVinci
This project registers stl's to the daVinci robot using monocular vision

## Code overview
The most important pieces of code are
- `vtk_registration/src/tools/camCalibration.py`
    - Running this will generate camera intrinsic and distortion matrices
- `vtk_registration/src/tools/daVinciCalibration.py`
    - Running this will use the camera data generated by the function above to register the robot to the camera
    - If no arguments are provided, camera parameters are pulled from the `vtk_registration/src/defaults/` directory.
- `vtk_registration/src/vtk_registration.py`
    - Running this will use both of the files generated above and allows you to register an STL to the daVinci robot
    - If no arguments are provided, resources are pulled from the `vtk_registration/src/defaults/` directory.

```
vtk_registration/
├── src/
│   ├── defaults/
│   ├── launch/
│   ├── code/
|   │   ├── amoeba_annealer.py
|   │   ├── controller.py
|   │   ├── model.py
|   │   ├── views
|   |   │   ├── main_view.py 
|   |   │   ├── main_view.ui
|   |   │   └── ui_main_view.py
|   │   ├── vtk_registration.py
│   │   └── vtkTools.py
│   ├── tools/
|   │   ├── camCalibration.py
|   │   └── daVinciCalibration.py
├── CMakeLists.txt
└── package.xml
```

