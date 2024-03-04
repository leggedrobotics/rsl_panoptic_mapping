#include "dynamic_mapping/tracking/Hungarian.h"
#include <iostream>

void HungarianSolver::solve(const Eigen::MatrixXd& cost) {
  m_ = cost;
  padMatrix(m_);
  marks_.resizeLike(m_);
  marks_.fill(0);
  rowCovered_.resize(m_.rows());
  rowCovered_.fill(0);
  colCovered_.resize(m_.cols());
  colCovered_.fill(0);
  path_.resize(m_.rows() * 2, Eigen::NoChange);
  path_.fill(-1);

  state_ = REDUCE_ROWS;
  while (state_ != DONE) {
    switch (state_) {
      case REDUCE_ROWS:
        reduce();
        break;
      case STAR_ZEROS:
        starZeros();
        break;
      case COVER_ZEROS:
        coverZeros();
        break;
      case PRIME_ZEROS:
        primeZeros();
        break;
      case MAKE_AUGMENTING_PATH:
        makeAugmentingPath();
        break;
      case RELAX_CONSTRAINTS:
        relaxConstraints();
        break;
      default:
        break;
    }
  }
  for (int i = 0; i < cost.rows(); i++) {
    for (int j = 0; j < cost.cols(); j++) {
      if (marks_(i, j) == STAR) {
        assignments_.insert(std::pair<int, int>(i, j));
      }
    }
  }
}

void HungarianSolver::padMatrix(Eigen::MatrixXd& m) {
  if (m.rows() != m.cols()) {
    int dim = std::max(m.rows(), m.cols());
    Eigen::MatrixXd tmp = m;
    m.resize(dim, dim);
    for (int i = 0; i < dim; i++) {
      for (int j = 0; j < dim; j++) {
        if (i >= tmp.rows() || j >= tmp.cols()) {
          m(i, j) = 0;
        } else {
          m(i, j) = tmp(i, j);
        }
      }
    }
  }
}

void HungarianSolver::reduce() {
  for (int i = 0; i < m_.rows(); i++) {
    double minElem = m_.row(i).minCoeff();
    Eigen::VectorXd minRow(m_.rows());
    minRow.fill(minElem);
    m_.row(i) -= minRow;
  }
  state_ = STAR_ZEROS;
}

void HungarianSolver::starZeros() {
  for (int i = 0; i < m_.rows(); i++) {
    if (rowCovered_(i)) continue;
    for (int j = 0; j < m_.cols(); j++) {
      if (colCovered_(j)) continue;
      if (m_(i, j) == 0) {
        marks_(i, j) = rowCovered_(i) = colCovered_(j) = STAR;
        break;
      }
    }
  }
  rowCovered_.fill(0);
  colCovered_.fill(0);
  state_ = COVER_ZEROS;
}

void HungarianSolver::coverZeros() {
  for (int i = 0; i < m_.cols(); i++) {
    if (marks_.col(i).any()) {
      colCovered_(i) = 1;
    }
  }
  if (colCovered_.sum() == m_.cols()) {
    state_ = DONE;
  } else {
    state_ = PRIME_ZEROS;
  }
}

void HungarianSolver::primeZeros() {
  bool done = false;
  while (!done) {
    int row, col;
    if (!findUncoveredZero(row, col)) {
      done = true;
      state_ = RELAX_CONSTRAINTS;
    } else {
      marks_(row, col) = PRIME;
      int starCol = findMarkInRow(row, STAR);
      if (starCol >= 0) {
        rowCovered_(row) = 1;
        colCovered_(starCol) = 0;
      } else {
        pathStartRow_ = row;
        pathStartCol_ = col;
        state_ = MAKE_AUGMENTING_PATH;
        done = true;
      }
    }
  }
}

void HungarianSolver::makeAugmentingPath() {
  bool done = false;
  int count = 0;
  path_(count, 0) = pathStartRow_;
  path_(count, 1) = pathStartCol_;
  while (!done) {
    int starRow = findMarkInCol(path_(count, 1), STAR);
    if (starRow >= 0) {
      count++;
      if (count >= path_.rows()) std::cout << "count greater than assigned size" << std::endl;
      path_(count, 0) = starRow;
      path_(count, 1) = path_(count - 1, 1);
    } else {
      done = true;
    }
    if (!done) {
      int primeCol = findMarkInRow(path_(count, 0), PRIME);
      count++;
      if (count >= path_.rows()) std::cout << "count greater than assigned size" << std::endl;
      path_(count, 0) = path_(count - 1, 0);
      path_(count, 1) = primeCol;
    }
  }
  for (int i = 0; i <= count; i++) {
    int r = path_(i, 0);
    int c = path_(i, 1);
    if (marks_(r, c) == STAR) {
      marks_(r, c) = NONE;
    } else {
      marks_(r, c) = STAR;
    }
  }
  rowCovered_.fill(0);
  colCovered_.fill(0);
  clearPrimes();

  state_ = COVER_ZEROS;
}

void HungarianSolver::relaxConstraints() {
  double minVal = std::numeric_limits<double>::max();
  for (int i = 0; i < m_.rows(); i++) {
    if (rowCovered_(i)) continue;
    for (int j = 0; j < m_.cols(); j++) {
      if (colCovered_(j)) continue;
      minVal = std::min(minVal, m_(i, j));
    }
  }

  for (int row = 0; row < m_.rows(); row++) {
    for (int col = 0; col < m_.cols(); col++) {
      if (rowCovered_(row)) {
        m_(row, col) += minVal;
      }
      if (!colCovered_(col)) {
        m_(row, col) -= minVal;
      }
    }
  }

  state_ = PRIME_ZEROS;
}
bool HungarianSolver::findUncoveredZero(int& row, int& col) {
  for (int i = 0; i < m_.rows(); i++) {
    if (rowCovered_(i)) continue;
    for (int j = 0; j < m_.cols(); j++) {
      if (colCovered_(j)) continue;
      if (m_(i, j) == 0) {
        row = i;
        col = j;
        return true;
      }
    }
  }
  return false;
}

int HungarianSolver::findMarkInRow(int row, Mark mark) {
  for (int i = 0; i < marks_.cols(); i++) {
    if (marks_(row, i) == mark) {
      return i;
    }
  }
  return -1;
}
int HungarianSolver::findMarkInCol(int col, Mark mark) {
  for (int i = 0; i < marks_.rows(); i++) {
    if (marks_(i, col) == mark) {
      return i;
    }
  }
  return -1;
}

void HungarianSolver::clearPrimes() {
  for (int i = 0; i < m_.rows(); i++) {
    for (int j = 0; j < m_.cols(); j++) {
      if (marks_(i, j) == PRIME) {
        marks_(i, j) = NONE;
      }
    }
  }
}