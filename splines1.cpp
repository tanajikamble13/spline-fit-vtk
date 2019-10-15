//// https://stackoverflow.com/questions/54111430/plot-splines-in-pcl-viewer?rq=1
/// how to draw splines 
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types_conversion.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/console/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>

#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkParametricSpline.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkParametricFunctionSource.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkPlaneSource.h>
int
main (int argc, char** argv)
{
/*
// Load the point cloud 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_curve(new pcl::PointCloud<pcl::PointXYZ>);
if (pcl::io::loadPCDFile<pcl::PointXYZ>("line.pcd", *cloud_curve) == -1) 
{
    PCL_ERROR("Couldn't read the pcd file.\n");
    exit(1); 
}

vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New (); 
    planeSource->SetXResolution (4); 
    planeSource->SetYResolution (4); 
    planeSource->SetOrigin (-1, -1, 0); 
    planeSource->SetPoint1 (1, -1, 0); 
    planeSource->SetPoint2 (-1, 1, 0); 

    vtkSmartPointer<vtkPolyDataMapper> planMapper = vtkSmartPointer<vtkPolyDataMapper>::New (); 
    planMapper->SetInputConnection (planeSource->GetOutputPort ()); 

    vtkSmartPointer<vtkActor> planeActor = vtkSmartPointer<vtkActor>::New (); 
    planeActor->SetMapper (planMapper); 
    planeActor->GetProperty ()->SetRepresentationToWireframe (); 
    planeActor->GetProperty ()->SetColor (1, 0, 0); 



// Visualisze curve points in pcl Viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer( "Simple Cloud Viewer"));
viewer->setBackgroundColor(0, 0, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cloud_curve, 255 , 0, 0);
viewer->addPointCloud(cloud_curve, handler, "cloud");
viewer->initCameraParameters();
viewer->getRenderWindow ()->GetRenderers()->GetFirstRenderer()->AddActor(planeActor);
//viewer->getRenderWindow ()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100));
}
*/
// Visualisze splines in vtk Viewer

double p0[3] = {-5.91152, 4.65316, 28063.7};
double p1[3] = { -6.2126, 3.8805, 28034.9};
double p2[3] = {-6.54915, 4.6517, 28007};
double p3[3] = {-6.91234, 4.91234, 27997};
double p4[3] = {-7.51234, 5.51234, 27989};
// Create a vtkPoints object and store the points in it
vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
points->InsertNextPoint(p0);
points->InsertNextPoint(p1);
points->InsertNextPoint(p2);
points->InsertNextPoint(p3);
points->InsertNextPoint(p4);
// Setup render window, renderer, and interactor
vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
renderWindow->SetSize(1000,2000); //(width, height)
vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
spline->SetPoints(points);
vtkSmartPointer<vtkParametricFunctionSource> functionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
functionSource->SetParametricFunction(spline);
functionSource->Update();


// Evaluate every 50 distance units along the line
  std::cout << "Spline interpolation:" << std::endl;
  double dt = .25;
  for( double t = dt ; t <= 1. - dt; t += dt )
    {
    std::cout << "t: " << t << " value = " << spline->Evaluate(t) << std::endl;
    }
// Setup actor and mapper
vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
mapper->SetInputConnection(functionSource->GetOutputPort());
vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
actor->SetMapper(mapper);
actor->GetProperty()->SetColor(1,0,0);
renderWindow->AddRenderer(renderer);
renderWindowInteractor->SetRenderWindow(renderWindow);
renderer->AddActor(actor);
renderWindow->Render();
vtkSmartPointer<vtkInteractorStyleSwitch> style = vtkSmartPointer<vtkInteractorStyleSwitch>::New();
style->SetCurrentStyleToTrackballCamera();
renderWindowInteractor->SetInteractorStyle( style );
renderWindowInteractor->Start();

}
