//#include "Vec3.h"
#include "Utilities.h"
#include <algorithm>
#include <random>

Vec3 getColor(const Ray& ray, const std::vector<std::unique_ptr<Shapes>>& shapes,const Vec3 & backcolor, int depth)
{
	hit_point point;
	if (depth <= 0) return Vec3(0.0, 0.0, 0.0);
	for (const auto& shape : shapes)
	{
		if (shape->hit(ray,0.001,INFINITY,point))
		{
			Ray scattered(Vec3(0,0,0),Vec3(0,0,0));
			Vec3 attenuation;
			Vec3 emitted = point.mat_ptr->emitted(point.u, point.v, point.point);

			if (!point.mat_ptr->scatter(ray, point, attenuation, scattered))
				return emitted;

			return emitted + attenuation * getColor(scattered, shapes, backcolor, depth - 1);

		}
	}
	return backcolor;
}


int main() 
{
	int x, y;
	std::cout << "INPUTS FOR MAIN\n"
		"Dimension(x y) : takes x, y as parameterand creates x by y pixel Scene\n"
		"Camera position(x y z) : position camera at x, y, z\n"
		"Camera target(x y z) : position camera target at x, y, z\n"
		"Ambient light(r g b): ambient light of environment\n"
		"Ground color(r g b): color of ground of environment\n"
		"if included\n"
		"Adding light source\n"
		"	Light source number(x): number of light source\n"
		"	light source color(r g b): specify color of light\n"
		"	light source position(x y z): position of light source\n"
		"Adding sphere :\n"
		"	Spherenumber(x) : number of Sphere\n"
		"	Sphere material(0 - 4) specify sphere material\n"
		"	Sphere position(x y z) position sphere at x y z\n"
		"	Sphere radius(x) specify radius of sphere\n"
		"	color of diffuse light(r g b) in interval 0-1: color of sphere for lambertian material\n";
	std::cout << "Enter dimensions(x y): ";
	std::cin >> x >> y;
	Scene scene(x,y);

	std::cout << "Enter camera position(x y z): ";
	double x1, y1, z1;
	std::cin >> x1>>y1>>z1;
	std::cout << "Enter camera target(x y z): ";
	double x2,y2,z2;	
	std::cin >> x2>> y2>>z2;
	Camera camera(Vec3(x1, y1, z1), Vec3(x2, y2, z2),x*1.0/y);

	auto lightSource = std::make_shared<diffuse_light>(Vec3(0.3, 0.7, 0.3));
	std::vector<std::unique_ptr<Shapes>> shapes;

	std::cout << "Enter ambient light(r g b) in interval 0-1: ";
	double ambientR, ambientG, ambientB;
	std::cin >> ambientR >> ambientG >> ambientB;
	Vec3 backolor(ambientR, ambientG, ambientB);

	double groundR, groundG, groundB;
	std::cout << "Enter ground color(r g b) in interval 0-1: ";
	std::cin >> groundR >> groundB >> groundB;
	auto lamb = std::make_shared<lambertian>(Vec3(groundR, groundB, groundB));
	shapes.push_back(std::make_unique<Sphere>(Vec3(0, -10001, -1), 10000, lamb));
	int lightSourceNumber;
	std::cout << "Enter light source number";
	std::cin >> lightSourceNumber;

	for (size_t i = 0; i < lightSourceNumber; i++)
	{
		std::cout << "Enter light source " <<(i + 1)<< " Color(r g b) in interval 0-1: ";
		double lightR, lightG, lightB;
		std::cin >> lightR >> lightG >> lightB;
		auto lightSource = std::make_shared<diffuse_light>(Vec3(lightR, lightG, lightB));

		std::cout << "Enter light source" << i + 1 << " position (x y z): ";
		double lightX, lightY, lightZ;
		std::cin >> lightX >> lightY >> lightZ;
		shapes.push_back(std::make_unique<Sphere>(Vec3(lightX, lightY, lightZ), 1, lightSource));
	}

	int sphereNumber;
	std::cout << "Enter Sphere number:";
	std::cin >> sphereNumber;


	auto metaf = std::make_shared<metal>(Vec3(0.8, 0.8, 0.8),0.8);
	auto meta = std::make_shared<metal>(Vec3(0.8, 0.8, 0.8), 0.0);
	auto dielectic = std::make_shared<dielec>(5.5);
	auto dielectic2 = std::make_shared<dielec>(0.2);

	
	for (int i = 0; i < sphereNumber; i++)
	{
		int mat;
		std::cout << "Choose material: lambertian(0),metal(1),metal fuzzy(2), dielectic(3), dielectic low index(4): ";
		std::cin >> mat;
		double x3, y3, z3;
		std::cout << "Enter Sphere position(x y z):";
		std::cin >> x3>> y3>> z3;
		std::cout << "Enter Sphere radius(r):";
		double r;
		std::cin >> r;
		switch (mat)
		{
		case 0:
		{
			std::cout << "Specify color of diffuse light(r g b) in interval 0-1: ";
			double diffuseX, diffuseY, diffuseZ;
			std::cin >> diffuseX >> diffuseY >> diffuseZ;
			lamb = std::make_shared<lambertian>(Vec3(diffuseX, diffuseY, diffuseZ));
			shapes.push_back(std::make_unique<Sphere>(Vec3(x3, y3, z3), r, lamb));
			break;
		}
		case 1:
			shapes.push_back(std::make_unique<Sphere>(Vec3(x3, y3, z3), r, meta));
			break;
		case 2:
			shapes.push_back(std::make_unique<Sphere>(Vec3(x3, y3, z3), r, metaf));
			break;
		case 3:
			shapes.push_back(std::make_unique<Sphere>(Vec3(x3, y3, z3), r, dielectic));
			break;
		case 4:
			shapes.push_back(std::make_unique<Sphere>(Vec3(x3, y3, z3), r, dielectic2));
			break;
		default:
			break;
		}
	}
	std::sort(shapes.begin(), shapes.end(), [&](const std::unique_ptr<Shapes>& a, const std::unique_ptr<Shapes>& b)
		{
			return (a->getCenter() - camera.position).length() < (b->getCenter()-camera.position).length();
		});

	for (int i = 0; i < y; i++)
	{
		for (int j = 0;  j < x;  j++)
		{
			Vec3 color(0.0, 0.0, 0.0);

			for (int k = 0; k < 100; k++) {
				Ray ray = camera.rayToWorld((j * 1.0+random_double(-1.0,1.0)) / (x - 1),( i * 1.0+random_double(-1.0,1.0))/ (y - 1));
				
				color += getColor(ray,shapes,backolor,50);
			}
			
			scene.set(j, y - i - 1, color);
		}

	}
	/*outputs to ppm file use converter*/
	std::ofstream output;
	output.open("./render.ppm", std::ios::out | std::ios::trunc);
	if (!output.is_open())
		return 1;

	write_to_PPM(scene, output,50);
	output.close();
	return 0;
}