#pragma once
#ifndef UTILITIES_H
#define UTILITIES_H
#include "Vec3.h"
#include <fstream>
#include <iostream>
#include <vector>

const double pi = 3.1415926535897932385;

struct Scene
{
	Vec3 * pixels;
	double ratio;
	int x, y;
	Scene(int x,int y) : x(x),y(y), ratio(y*1.0/x),pixels(new Vec3[x*y]) {}
	~Scene()
	{
		delete[] pixels;
	}
	Vec3& get(unsigned x, unsigned y) const
	{
		return pixels[y * this->x + x];
	}
	Vec3& get(unsigned x) const
	{
		return pixels[x];
	}
	void set(unsigned a, unsigned b, Vec3& point)
	{
		pixels[b * x + a] = point;
	}
};
struct Ray
{
	Vec3 origin, direction;
	double time;
	Ray(const Vec3& origin, const Vec3& direction, double time = 0) : origin(origin), direction(direction), time(time) {}
	Vec3 at(double t) const
	{
		return origin + direction * t;
	}
};
struct Camera
{
	Vec3 position, left_bottom, horizontal, vertical,forward,right,up;
	double focal_length,lens_radius;
	Camera(const Vec3& pos,const Vec3& target, double ratio,double aperture = 1, double focal_length = 1) : position(pos), focal_length(focal_length)
	{
		up = Vec3(0, 1, 0);
		forward = normalize((target - position));
		right = normalize(crossProduct(forward,up));
		up = normalize(crossProduct(forward,-right));

		horizontal = right * ratio;
		vertical = up * 1;
		left_bottom = position + forward * focal_length - horizontal * 0.5 - vertical*0.5;
		lens_radius = aperture / 2;
	}
	Ray rayToWorld(double x,double y)const
	{
		return Ray(position, normalize(left_bottom+ x * horizontal + y * vertical - position));
	}

};
class material;
struct hit_point {
	Vec3 point;
	Vec3 normal;
	double root;
	double u;
	double v;
	std::shared_ptr<material> mat_ptr;
	bool isFront;

	inline void setNormal(const Ray& r, const Vec3& point_normal) {
		isFront = dotProduct(r.direction, point_normal) < 0;
		normal = isFront ? point_normal : -point_normal;
	}
};
struct material
{
	virtual bool scatter(const Ray& r_in, const hit_point& rec, Vec3& attenuation, Ray& scattered) const = 0;
	virtual Vec3 emitted(double u, double v, const Vec3& p) const {
		return Vec3(0, 0, 0);
	}
};



struct lambertian :material
{
public:
	Vec3 color;
	lambertian(const Vec3& a) : color(a) {}

	virtual bool scatter(const Ray& ray, const hit_point& rec, Vec3& attenuation, Ray& scattered) const override
	{
		auto scatterDirection = rec.normal + random_unit_vector();
		if (scatterDirection.nearZero()) scatterDirection = rec.normal;
		scattered = Ray(rec.point, scatterDirection);
		attenuation = color;
		return true;
	}
};

struct metal :material
{
public:
	Vec3 color;
	double fuzziness;
	metal(const Vec3& a,double f) : color(a), fuzziness(f < 1 ? f : 1) {}

	virtual bool scatter(
		const Ray& r_in, const hit_point& rec, Vec3& attenuation, Ray& scattered) const override
	{
		Vec3 reflection = reflect(normalize(r_in.direction), rec.normal);
		scattered = Ray(rec.point, reflection+fuzziness*random_in_unit_sphere());
		attenuation = color;
		return (dotProduct(scattered.direction, rec.normal) > 0);
	}
};
double reflectance(double cosine, double ref_idx) {
	auto r0 = (1 - ref_idx) / (1 + ref_idx);
	r0 = r0 * r0;
	return r0 + (1 - r0) * pow((1 - cosine), 5);
}
struct dielec :material
{
	double ir;
	dielec(double index_of_refraction) : ir(index_of_refraction) {}

	virtual bool scatter(const Ray& r_in, const hit_point& rec, Vec3& attenuation, Ray& scattered) const override
	{
		attenuation = Vec3(1.0, 1.0, 1.0);
		double refraction_ratio = rec.isFront ? (1.0 / ir) : ir;

		Vec3 unit_direction = normalize(r_in.direction);
		double cos_theta = fmin(dotProduct(-unit_direction, rec.normal), 1.0);
		double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

		bool cannot_refract = refraction_ratio * sin_theta > 1.0;
		Vec3 direction;

		if  (cannot_refract || reflectance(cos_theta, refraction_ratio) > random_double(0.0,1.0))
			direction = reflect(unit_direction, rec.normal);
		else
			direction = refract(unit_direction, rec.normal, refraction_ratio);

		scattered = Ray(rec.point, direction);
		Vec3 refracted = refract(unit_direction, rec.normal, refraction_ratio);

		scattered = Ray(rec.point, refracted);

		return true;
	}
};

struct Shapes
{
	virtual bool hit(const Ray& ray,double min_time,double max_time,hit_point & point) const
	{
		return 0;
	}
	virtual Vec3 getCenter() const
	{
		return Vec3();
	}
};

struct Sphere : Shapes
{
public:
	Vec3 center;
	double radius;
	std::shared_ptr<material>  mat_ptr;
	Sphere(const Vec3& center, double r,const std::shared_ptr<material> m) :center(center), radius(r),mat_ptr(m){}
	virtual bool hit(const Ray& ray,double min_time,double max_time,hit_point & point) const override
	{
		Vec3 oc = ray.origin - center;
		double a = dotProduct(ray.direction, ray.direction);
		double b = dotProduct(oc, ray.direction);
		double c = dotProduct(oc, oc) - radius * radius;
		double d = b * b -  a * c;
		if (d < 0)
		{
			return false;
		}
		double root = (-b-std::sqrt(d))/a;
		if (root < min_time || max_time < root)
		{
			root = (-b + std::sqrt(d)) / a;
			if (root<min_time || root> max_time) return false;
		}
		point.root = root;
		point.point = ray.at(root);
		point.setNormal(ray, (point.point - center) / radius);
		get_sphere_uv((point.point - center) / radius,point.u,point.v);
		point.mat_ptr = mat_ptr;
		return true;
		
	}
	virtual Vec3 getCenter() const override
	{
		return center;
	}
	static void get_sphere_uv(const Vec3& p, double& u, double& v) {
		auto theta = acos(-p.y());
		auto phi = atan2(-p.z(), p.x()) + pi;

		u = phi / (2 * pi);
		v = theta / pi;
	}
};

struct Cylinder :Shapes
{
	Vec3 bottom_center, up_center;
	double radius, height;
	std::shared_ptr<material> mat;
	Cylinder(const Vec3& bottom, const Vec3& up, double r, std::shared_ptr<material> m) :bottom_center(bottom), up_center(up),
		radius(r), height((up - bottom).length()),mat(m) {}
	virtual bool hit(const Ray& ray, double min_time, double max_time, hit_point& point) const override
	{
		double x = 1 - dotProduct(up_center - bottom_center, up_center - bottom_center);
		Vec3 oc = ray.origin - bottom_center;
		double a = dotProduct(ray.direction, ray.direction)*x;
		double b = dotProduct(oc, ray.direction)*x;
		double c = dotProduct(oc, oc)*x - radius * radius;
		double d = b * b - a * c;
		if (d < 0)
		{
			return false;
		}
		double root = (-b - std::sqrt(d)) / a;
		if (root < min_time || max_time < root)
		{
			root = (-b + std::sqrt(d)) / a;
			if (root<min_time || root> max_time) return false;
		}
		point.root = root;
		point.point = ray.at(root);
		double h = dotProduct(point.point - bottom_center, up_center - bottom_center);
		Vec3 center = (up_center - bottom_center) * h / height;
		point.setNormal(ray, (point.point - center) / radius);
		get_sphere_uv((point.point - center) / radius, point.u, point.v);
		point.mat_ptr = mat;
		return true;

	}
	virtual Vec3 getCenter() const override
	{
		return bottom_center;
	}
	static void get_sphere_uv(const Vec3& p, double& u, double& v) {
		auto theta = acos(-p.y());
		auto phi = atan2(-p.z(), p.x()) + pi;

		u = phi / (2 * pi);
		v = theta / pi;
	}
};

struct xy_rect : Shapes {
	std::shared_ptr<material> mat;
	double x0, x1, y0, y1, k;
	xy_rect() {}

	xy_rect(double x0, double x1, double y0, double y1, double k,
		std::shared_ptr<material> mat)
		: x0(x0), x1(x1), y0(y0), y1(y1), k(k), mat(mat) {};

	virtual bool hit(const Ray& r, double t_min, double t_max, hit_point& rec) const override
	{
		auto t = (k - r.origin.z()) / r.direction.z();
		if (t < t_min || t > t_max)
			return false;
		auto x = r.origin.x() + t * r.direction.x();
		auto y = r.origin.y() + t * r.direction.y();
		if (x < x0 || x > x1 || y < y0 || y > y1)
			return false;
		rec.u = (x - x0) / (x1 - x0);
		rec.v = (y - y0) / (y1 - y0);
		rec.root = t;
		auto outward_normal = Vec3(0, 0, 1);
		rec.setNormal(r, outward_normal);
		rec.mat_ptr = mat;
		rec.point = r.at(t);
		return true;
	}
	virtual Vec3 getCenter() const override
	{
		return Vec3((x0 + x1) / 2, (y0 + y1) / 2, k);
	}
};
struct texture {
	virtual Vec3 value(double u, double v, const Vec3& p) const = 0;
};

struct solid_color : texture {
	Vec3 color_value;
	solid_color() {}
	solid_color(const Vec3& c) : color_value(c) {}

	solid_color(double red, double green, double blue)
		: solid_color(Vec3(red, green, blue)) {}

	virtual Vec3 value(double u, double v, const Vec3& p) const override {
		return color_value;
	}
};
struct diffuse_light :material
{
	std::shared_ptr<texture> emit;
	diffuse_light(std::shared_ptr<texture> a) : emit(a) {}
	diffuse_light(Vec3 c) : emit(std::make_shared<solid_color>(c)) {}

	virtual bool scatter(const Ray& r_in, const hit_point& rec, Vec3& attenuation, Ray& scattered) const override {
		return false;
	}

	virtual Vec3 emitted(double u, double v, const Vec3& p) const override {
		return emit->value(u, v, p);
	}
};
inline double aligner(double x, double min, double max) {
	return x<min?min: (x>max ?max: x);
}
void write_to_PPM(const Scene& scene, std::ostream& output)
{
	output << "P3\n" << scene.x << ' ' << scene.y << "\n255\n";

	const int total_pixels =scene.x * scene.y;
	for (unsigned i = 0; i < total_pixels; ++i)
	{
		Vec3 v = scene.get(i) * 255.99;

		int ir = static_cast<int>(v.r());
		int ig = static_cast<int>(v.g());
		int ib = static_cast<int>(v.b());

		output << ir << ' ' << ig << ' ' << ib << '\n';
	}
	std::cout << "Outputting" << std::endl;
}
void write_to_PPM(const Scene& scene, std::ostream& output,int randomCount)
{
	output << "P3\n" << scene.x << ' ' << scene.y << "\n255\n";

	const int total_pixels = scene.x * scene.y;
	for (unsigned i = 0; i < total_pixels; ++i)
	{
		auto r = scene.get(i).r();
		auto g = scene.get(i).g();
		auto b = scene.get(i).b();
		r = std::sqrt(r*1.0/randomCount);
		g = std::sqrt(g * 1.0 / randomCount);
		b = std::sqrt(b * 1.0 / randomCount);


		int ir = static_cast<int>(256*aligner(r,0.0,0.999));
		int ig = static_cast<int>(256*aligner(g, 0.0, 0.999));
		int ib = static_cast<int>(256*aligner(b, 0.0, 0.999));

		output << ir << ' ' << ig << ' ' << ib << '\n';
	}
	std::cout << "Outputting" << std::endl;
}

#endif