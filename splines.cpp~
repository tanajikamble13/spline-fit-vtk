#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkSphereSource.h>
#include <vtkGlyph3DMapper.h>
#include <vtkVersion.h>

int main(int, char *[])
{       
        std::cout << vtkVersion::GetVTKSourceVersion() << std::endl;
        std::cout << vtkVersion::GetVTKMajorVersion() << std::endl;
        std::cout << vtkVersion::GetVTKMinorVersion() << std::endl;
	double p0[3] = { 1.0, 0.0, 0.0 };
	double p1[3] = { 3.0, 2.0, 0.0 };
	double p2[3] = { 5.0, 0.0, 0.0 };
        double p3[3] = { 7.0, 0.0, 0.0 };
        double p4[3] = { 8.0, 2.0, 0.0 };
 
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(p0);
	points->InsertNextPoint(p1);
	points->InsertNextPoint(p2);
        points->InsertNextPoint(p3);
        points->InsertNextPoint(p4);

	vtkSmartPointer<vtkParametricSpline> spline =
		vtkSmartPointer<vtkParametricSpline>::New();
	spline->SetPoints(points);

	vtkSmartPointer<vtkPoints> outPoints =
		vtkSmartPointer<vtkPoints>::New();
	int pointsCnt = 11;
	double step = 1.0 / (pointsCnt-1);
	for (double i = 0; i <= 1; i = i + step)
	{
		double u[] = {i, 0, 0};
		double p[3];
		spline->Evaluate(u, p, NULL);
		outPoints->InsertNextPoint(p);
	}

	vtkSmartPointer<vtkParametricSpline> outSpline =
		vtkSmartPointer<vtkParametricSpline>::New();
	outSpline->SetPoints(outPoints);

	vtkSmartPointer<vtkParametricFunctionSource> functionSource =
		vtkSmartPointer<vtkParametricFunctionSource>::New();
	functionSource->SetParametricFunction(outSpline);
	functionSource->Update();

	vtkSmartPointer<vtkPolyDataMapper> splineMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	splineMapper->SetInputConnection(functionSource->GetOutputPort());

	vtkSmartPointer<vtkActor> splineActor =
		vtkSmartPointer<vtkActor>::New();
	splineActor->SetMapper(splineMapper);

	vtkSmartPointer<vtkSphereSource> sphereSource =
		vtkSmartPointer<vtkSphereSource>::New();
	sphereSource->SetPhiResolution(21);
	sphereSource->SetThetaResolution(21);
	sphereSource->SetRadius(.05);

	vtkSmartPointer<vtkPolyData> splinePointsData =
		vtkSmartPointer<vtkPolyData>::New();
	splinePointsData->SetPoints(outPoints);

	vtkSmartPointer<vtkGlyph3DMapper> splinePointsMapper =
		vtkSmartPointer<vtkGlyph3DMapper>::New();
	splinePointsMapper->SetInputData(splinePointsData);
	splinePointsMapper->SetSourceConnection(sphereSource->GetOutputPort());

	vtkSmartPointer<vtkActor> pointsActor =
		vtkSmartPointer<vtkActor>::New();
	pointsActor->SetMapper(splinePointsMapper);
	pointsActor->GetProperty()->SetColor(1, 0, 0);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->SetSize(600, 600);
	renderWindow->AddRenderer(renderer);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderer->AddActor(splineActor);
	renderer->AddActor(pointsActor);

	renderWindow->Render();
	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

