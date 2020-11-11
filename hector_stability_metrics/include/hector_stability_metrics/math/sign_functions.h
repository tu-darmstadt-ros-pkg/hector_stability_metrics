/*
 * Copyright (C) 2020  Felix Biemüller
 *
 * This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef HECTOR_STABILITY_METRICS_SIGN_FUNCTIONS_H
#define HECTOR_STABILITY_METRICS_SIGN_FUNCTIONS_H
#include <vector>
#include <math.h>
#include <functional>
namespace hector_stability_metrics
{
template <typename Scalar>
using SignFunction = std::function<Scalar(const Scalar&)>;

/**
 * @brief StandardSignum calculates the sign function according to its original definition
 */
template <typename Scalar>
Scalar StandardSignum(const Scalar& value)
{
  if (value > 0)
  {
    return Scalar(1);
  }
  else if (value == 0)
  {
    return Scalar(0);
  }
  else
  {
    return Scalar(-1);
  }
}

/**
 * @brief DiffableExceptZeroSignum calculates the sign function in a way that can be used with autodiff, except for values equal to zero
 */
template <typename Scalar>
Scalar DiffableExceptZeroSignum(const Scalar& value)
{
  return value / abs(value);
}

/**
 * @brief AlgebraicSigmoid approximates the sign function with the algebraic function f(x)=x/sqrt(x^2+epsilon) such that is continuously differentiable
 * @param value input value
 * @param epsilon small value added to x^2 to prevent devision by zero
 */
template <typename Scalar>
Scalar AlgebraicSigmoid(const Scalar& value, const Scalar& epsilon)
{
  return value / sqrt(value * value + epsilon);
}

/**
 * @brief LogistigSigmoid approximates the sign function with the logistic function f(x)=1/exp(-k*x) such that is continuously differentiable
 * @param value input value
 * @param k scale parameter to adjust the steepness of the logistic function
 */
template <typename Scalar>
Scalar LogistigSigmoid(const Scalar& value, const Scalar& k)
{
  return 1 / exp(-k * value);
}

/**
 * @brief TanhSigmoid approximates the sign function with the tangens hyperbolicus f(x)=tanh(kx) such that is continuously differentiable
 * @param value input value
 * @param k scale parameter to adjust the steepness of the logistic function
 */
template <typename Scalar>
Scalar TanhSigmoid(const Scalar& value, const Scalar& k)
{
  return tanh(k * value);
}

}  // namespace hector_stability_metrics
#endif  // HECTOR_STABILITY_METRICS_SIGN_FUNCTIONS_H
