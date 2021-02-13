#pragma once
#ifndef VEC_3_H
#define VEC_3_H
#include <iostream>
#include <cmath>
#include <random>
inline double random_double(double x,double y) {
	static std::uniform_real_distribution<double> distribution(x, y);
	static std::mt19937 generator;
	return distribution(generator);
}
class Vec3
{
public:
	Vec3() : e{ 0,0,0 } {};
	Vec3(double e1, double e2, double e3)
	{
		e[0] = e1;
		e[1] = e2;
		e[2] = e3;
	}
	double x() const
	{
		return e[0];
	}
	double y() const
	{
		return e[1];
	}
	double z() const
	{
		return e[2];
	}
	double r() const
	{
		return e[0];
	}
	double g() const
	{
		return e[1];
	}
	double b() const
	{
		return e[2];
	}
	Vec3 operator-() const
	{
		return Vec3(-e[0], -e[1], -e[2]);
	}
	double& operator[](int i) { return e[i];  }
	double operator[](int i) const { return e[i]; }
	Vec3& operator*=(double e1)
	{
		e[0] *= e1;
		e[1] *= e1;
		e[2] *= e1;
		return *this;
	}
	Vec3& operator*=(Vec3 & e1) 
	{
		this->e[0] += e1[0];
		this->e[1] += e1[1];
		this->e[2] += e1[2];
		return *this;
	}
	Vec3& operator+=(const Vec3& v) {
		e[0] += v.e[0];
		e[1] += v.e[1];
		e[2] += v.e[2];
		return *this;
	}
	double length() const
	{
		return std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
	}

	Vec3& operator/=(const double t)
	{
		return *this *= 1 / t;
	}
	inline static Vec3 random(double min, double max) {
		return Vec3(random_double(min, max), random_double(min, max), random_double(min, max));
	}
	bool nearZero() const {
		const auto epsilon = 1e-8;
		return (fabs(e[0]) < epsilon) && (fabs(e[1]) < epsilon) && (fabs(e[2]) < epsilon);
	}

private:
	double e[3];
};

Vec3 normalize(Vec3 p)
{
	return p /= p.length();
}
Vec3 crossProduct(const Vec3 & A,const Vec3 & B)
{
	Vec3 P;
	P[0] = A[1] * B[2] - A[2] * B[1];
	P[1] = A[2] * B[0] - A[0] * B[2];
	P[2] = A[0] * B[1] - A[1] * B[0];
	return P;
}
double dotProduct(const Vec3& A,const Vec3& B)
{
	return A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
}

inline std::ostream& operator<<(std::ostream& out, const Vec3& v) {
	return out << v.x() << ' ' << v.y() << ' ' << v.z();
}

inline Vec3  operator+(const Vec3& u, const Vec3& v) {
	return Vec3(u.x() + v.x(), u.y() + v.y(), u.z() + v.z());
}

inline Vec3 operator-(const Vec3& u,const  Vec3& v) {
	return Vec3(u.x() - v.x(), u.y() - v.y(), u.z() - v.z());
}

inline Vec3  operator*(const Vec3& u, const Vec3& v) {
	return Vec3(u.x() * v.x(), u.y() * v.y(), u.z() * v.z());
}

inline Vec3 operator*(double t, const Vec3& v) {
	return Vec3(t * v.x(), t * v.y(), t * v.z());
}

inline Vec3 operator*(const Vec3& v, double t) {
	return t * v;
}

inline Vec3 operator/(const Vec3 & v, double t) {
	return (1 / t) * v;
}

Vec3 random_in_unit_sphere() {
	while (true) {
		auto p = Vec3::random(-1, 1);
		if (p.length()*p.length() >= 1) continue;
		return p;
	}
}
inline Vec3 random_in_unit_disk() {
	while (true) {
		auto p = Vec3(random_double(-1, 1), random_double(-1, 1), 0);
		if (p.length()*p.length() >= 1) continue;
		return p;
	}
}

Vec3 random_unit_vector() {
	return normalize(random_in_unit_sphere());
}
Vec3 reflect(const Vec3& v, const Vec3& n) {
	return v - 2 * dotProduct(v, n) * n;
}

Vec3 refract(const Vec3& uv, const Vec3& n, double etai_over_etat) {
	auto cos_theta = fmin(dotProduct(-uv, n), 1.0);
	Vec3 r_out_perp = etai_over_etat * (uv + cos_theta * n);
	Vec3 r_out_parallel = -sqrt(fabs(1.0 - r_out_perp.length()*r_out_perp.length())) * n;
	return r_out_perp + r_out_parallel;
}
#endif // !VEC_3
