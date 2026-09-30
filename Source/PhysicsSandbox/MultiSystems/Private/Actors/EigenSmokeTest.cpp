#include <Eigen/Dense>
#include <Eigen/SVD>

void EigenSmokeTest()
{
	// -------------------------------------------------------------------------
	// Fixed-size test
	// -------------------------------------------------------------------------

	Eigen::Matrix2d A;

	A <<
		1.0, 2.0,
		3.0, 4.0;

	Eigen::Vector2d X;

	X <<
		1.0,
		2.0;

	const Eigen::Vector2d Y =
		A * X;

	Eigen::JacobiSVD<Eigen::Matrix2d> SVD(
		A,
		Eigen::ComputeFullU |
		Eigen::ComputeFullV
	);

	const Eigen::Vector2d SingularValues =
		SVD.singularValues();

	// -------------------------------------------------------------------------
	// Dynamic-size test
	// -------------------------------------------------------------------------

	Eigen::MatrixXd DynamicMatrix(3, 2);

	DynamicMatrix <<
		1.0, 2.0,
		3.0, 4.0,
		5.0, 6.0;

	Eigen::VectorXd DynamicVector(2);

	DynamicVector <<
		1.0,
		2.0;

	const Eigen::VectorXd DynamicResult =
		DynamicMatrix * DynamicVector;

	Eigen::JacobiSVD<Eigen::MatrixXd> DynamicSVD(
		DynamicMatrix,
		Eigen::ComputeFullU |
		Eigen::ComputeFullV
	);

	const Eigen::VectorXd DynamicSingularValues =
		DynamicSVD.singularValues();
}