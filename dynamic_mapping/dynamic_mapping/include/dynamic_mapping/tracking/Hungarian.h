#ifndef DYNAMIC_MAPPING_TRACKING_HUNGARIAN_H_
#define DYNAMIC_MAPPING_TRACKING_HUNGARIAN_H_
#include <Eigen/Core>
#include <unordered_map>
/**
 * @brief Implements the Hungarian (or Munkres) algorithm for optimal assignment.
 */
class HungarianSolver {
 public:
  void solve(const Eigen::MatrixXd& cost);
  std::unordered_map<int, int> getAssignments() { return assignments_; };

 private:
  enum Step { DONE = 0, REDUCE_ROWS, STAR_ZEROS, COVER_ZEROS, PRIME_ZEROS, MAKE_AUGMENTING_PATH, RELAX_CONSTRAINTS };
  enum Mark { NONE = 0, STAR, PRIME };
  void padMatrix(Eigen::MatrixXd& m);
  void reduce();
  void starZeros();
  void coverZeros();
  void primeZeros();
  void makeAugmentingPath();
  void relaxConstraints();
  bool findUncoveredZero(int& row, int& col);
  int findMarkInRow(int row, Mark mark);
  int findMarkInCol(int col, Mark mark);
  void clearPrimes();
  Eigen::MatrixXd m_;
  Eigen::MatrixXi marks_;
  Eigen::MatrixX2i path_;
  Eigen::VectorXi rowCovered_, colCovered_;
  Step state_;
  int pathStartRow_{0}, pathStartCol_{0};
  std::unordered_map<int, int> assignments_;
};
#endif