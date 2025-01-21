#include "dynamic_mapping/LidarCameraProjector.h"
#include <Eigen/Core>
#include <iostream>
#include <dynamic_mapping/helpers.h>
#include <fstream>

void LidarCameraProjector::projectPtsToCameraFrame(const Eigen::Matrix3Xd& pts, const Eigen::Isometry3d& transform) {


  Eigen::Matrix4Xd homogenousPts = pts.colwise().homogeneous(); // P_W 4*n

  const Eigen::Matrix4d& homogeneousTransform = transform.matrix(); // T_WC 4*4
  Eigen::Matrix<double, 3, 4> projection = cameraProjection_ * homogeneousTransform; // 
  projectedPoints_ = projection * homogenousPts;// 3*4 times 4*n

  // Debugging
  Eigen::Matrix4Xd transformedPts = transform.matrix() * homogenousPts; // 4 x N
  
  //writeMatrixToCSV(transformedPts, "transformedPts.csv");
  //writeMatrixToCSV(projectedPoints_, "projectedPoints_.csv");

}

std::vector<int> LidarCameraProjector::filterVisiblePoints(const Eigen::Matrix3Xd& pclPts, cv::Size maskShape) {


  Eigen::ArrayXXd u = projectedPoints_.row(0).array() / projectedPoints_.row(2).array();
  Eigen::ArrayXXd v = projectedPoints_.row(1).array() / projectedPoints_.row(2).array();
  Eigen::ArrayXXd z = projectedPoints_.row(2).array();
  int h = maskShape.height;
  int w = maskShape.width;
  auto inliers = (u >= 0) && (u < w) && (v >= 0) && (v < h) && (z > 0);

  //std::cout << "h: " << h << ", w: " << w << std::endl;
  //std::cout << "u >= 0: " << (u >= 0).count() << " / " << u.size() << std::endl;
  //std::cout << "u < w: " << (u < w).count() << " / " << u.size() << std::endl;
  //std::cout << "v >= 0: " << (v >= 0).count() << " / " << v.size() << std::endl;
  //std::cout << "v < h: " << (v < h).count() << " / " << v.size() << std::endl;
  //std::cout << "z > 0: " << (z > 0).count() << " / " << z.size() << std::endl;
  //std::cout << "(u >= 0)&& (v >= 0) && (z > 0)" << ((u >= 0) && (v >= 0) && (z > 0)).count() << " / " << z.size() << std::endl;
  //std::cout << " (u >= 0) && (u < w) " << ( (u >= 0) && (u < w)).count() << " / " << z.size() << std::endl;
  //std::cout << " (u >= 0) && (u < w) && (v >= 0) " << ( (u >= 0) && (u < w) && (v >= 0)).count() << " / " << z.size() << std::endl;
  //std::cout << " (u >= 0) && (u < w) && (v >= 0) && (v < h) " << ( (u >= 0) && (u < w) && (v >= 0) && (v < h)).count() << " / " << z.size() << std::endl;
  //std::cout << " (u >= 0) && (u < w) && (v >= 0) && (v < h) && (z > 0) " << ( (u >= 0) && (u < w) && (v >= 0) && (v < h) && (z > 0)).count() << " / " << z.size() << std::endl;
  //std::cout << "u min: " << u.minCoeff() << ", max: " << u.maxCoeff() << std::endl;
  //std::cout << "v min: " << v.minCoeff() << ", max: " << v.maxCoeff() << std::endl;
  //std::cout << "z min: " << z.minCoeff() << ", max: " << z.maxCoeff() << std::endl;
  //writeUVZToCSV(u, v, z, "uvz.csv");


  std::vector<int> inlierIdx;
  for (int i = 0; i < inliers.size(); i++) {
    if (inliers(i)) {
      inlierIdx.push_back(i);
      // Delete column from projected points
    }
  }
  auto ptsInCamera = projectedPoints_(Eigen::all, inlierIdx);
  visiblePoints_ = pclPts(Eigen::all, inlierIdx);
  pixelPoints_ = ptsInCamera.array().rowwise() / ptsInCamera.row(2).array();

  Eigen::ArrayXXd zFiltered = z(Eigen::all, inlierIdx);
  pixelPoints_.row(2) = zFiltered; // error here
  return inlierIdx;
}

// Function to write u, v, and z to a CSV file
void LidarCameraProjector::writeUVZToCSV(const Eigen::ArrayXXd& u, const Eigen::ArrayXXd& v, const Eigen::ArrayXXd& z, const std::string& filename) {
    if (u.rows() != v.rows() || u.rows() != z.rows() || u.cols() != v.cols() || u.cols() != z.cols()) {
        std::cerr << "Error: Dimensions of u, v, and z must match." << std::endl;
        return;
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    // Write u, v, and z values to the CSV file
    for (int i = 0; i < u.rows(); ++i) {
        for (int j = 0; j < u.cols(); ++j) {
            file << u(i, j) << "," << v(i, j) << "," << z(i, j);
            if (j < u.cols() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();

    if (file.good()) {
        std::cout << "u, v, and z successfully written to " << filename << std::endl;
    } else {
        std::cerr << "Error: Failed to write to " << filename << std::endl;
    }
}

void LidarCameraProjector::writeMatrixToCSV(const Eigen::Matrix4Xd& matrix, const std::string& filename) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    // Iterate over the rows and columns of the matrix
    for (int row = 0; row < matrix.rows(); ++row) {
        for (int col = 0; col < matrix.cols(); ++col) {
            file << matrix(row, col);
            // Add a comma after each element except the last in the row
            if (col < matrix.cols() - 1) {
                file << ",";
            }
        }
        // End the row with a newline
        file << "\n";
    }

    file.close();

    if (file.good()) {
        std::cout << "Matrix successfully written to " << filename << std::endl;
    } else {
        std::cerr << "Error: Failed to write to " << filename << std::endl;
    }
}
void LidarCameraProjector::writeMatrixToCSV(const Eigen::Matrix3Xd& matrix, const std::string& filename) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    // Iterate over the rows and columns of the matrix
    for (int row = 0; row < matrix.rows(); ++row) {
        for (int col = 0; col < matrix.cols(); ++col) {
            file << matrix(row, col);
            // Add a comma after each element except the last in the row
            if (col < matrix.cols() - 1) {
                file << ",";
            }
        }
        // End the row with a newline
        file << "\n";
    }

    file.close();

    if (file.good()) {
        std::cout << "Matrix successfully written to " << filename << std::endl;
    } else {
        std::cerr << "Error: Failed to write to " << filename << std::endl;
    }
}