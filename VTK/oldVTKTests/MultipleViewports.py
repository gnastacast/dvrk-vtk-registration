import vtk
 
def main():
    '''One render window, multiple viewports'''
    iren_list = []
    rw = vtk.vtkRenderWindow()
    rw.SetSize(640*2, 480)
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(rw)
    # Define viewport ranges
    xmins=[0,.5]
    xmaxs=[0.5,1]
    ymins=[0]
    ymaxs=[1]
    for i in range(2):
        ren = vtk.vtkRenderer()
        rw.AddRenderer(ren)
        ren.SetViewport(xmins[i],ymins[0],xmaxs[i],ymaxs[0])
 
        #Create a sphere
        sphereSource = vtk.vtkSphereSource()
        sphereSource.SetCenter(0.0, 0.0, 0.0)
        sphereSource.SetRadius(5)
 
        #Create a mapper and actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphereSource.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        ren.AddActor(actor)
        ren.ResetCamera()
 
    rw.Render()
    rw.SetWindowName('RW: Multiple ViewPorts')
    iren.Start()
 
 
if __name__ == '__main__':
    main()