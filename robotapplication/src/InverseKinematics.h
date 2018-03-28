/*
 * inverseKinematics.h
 *
 *  Created on: Apr 5, 2017
 *      Author: thomas
 */

#ifndef SRC_INVERSEKINEMATICS_H_
#define SRC_INVERSEKINEMATICS_H_

#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include "Matrix.hpp"

const double PI = std::acos(-1);

class InverseKinematics
{
  public:
    static InverseKinematics& getInstance()
    {
        static InverseKinematics instance;
        return instance;
    }

    static double toRadians(double aDegrees)
    {
        return aDegrees * PI / 180.0;
    }

    static double toDegrees(double radians)
    {
        return radians * 180.0 / PI;
    }

    template <std::size_t W, class T>
    static Matrix<3, 1, T> calculatePosition(const Matrix<W, 1, T>& phis, const Matrix<W, 1, T>& armLenghts)
    {
        Matrix<3, 1, T> temp(0);
        double totalAngle = 0;
        double baseAngle = phis.at(0, 0);
        for (std::size_t i = 1; i < W; ++i)
        {
            totalAngle += phis.at(i, 0);
            temp[0][0] += armLenghts.at(i, 0) * std::cos(totalAngle) * std::cos(baseAngle);
            temp[1][0] += armLenghts.at(i, 0) * std::cos(totalAngle) * std::sin(baseAngle);
            temp[2][0] += armLenghts.at(i, 0) * std::sin(totalAngle);
        }
        return temp;
    }

    template <std::size_t H, class T>
    static Matrix<H, 3, T> calculateJacobijnse(const Matrix<H, 1, T>& phis, const Matrix<H, 1, T>& arm)
    {
        Matrix<H, 3, T> temp(0);
        double totalAngle = 0;
        double baseAngle = phis.at(0, 0);
        for (std::size_t i = 1; i < H; ++i)
        {
            totalAngle += phis.at(i, 0);
            temp[0][0] += arm.at(i, 0) * std::cos(totalAngle) * -std::sin(baseAngle);  // x1
            temp[1][0] += arm.at(i, 0) * std::cos(totalAngle) * std::cos(baseAngle);   // y1
            temp[2][0] += 0;                                                           // z1
            temp[0][1] += arm.at(i, 0) * -std::sin(totalAngle) * std::cos(baseAngle);  // x2
            temp[1][1] += arm.at(i, 0) * -std::sin(totalAngle) * std::sin(baseAngle);  // y2
            temp[2][1] += arm.at(i, 0) * std::cos(totalAngle);                         // z2
            if (i >= 2)
            {
                temp[0][2] += arm.at(i, 0) * -std::sin(totalAngle) * std::cos(baseAngle);  // x3
                temp[1][2] += arm.at(i, 0) * -std::sin(totalAngle) * std::sin(baseAngle);  // y3
                temp[2][2] += arm.at(i, 0) * std::cos(totalAngle);                         // z3
            }
            if (i > 3)
            {
                temp[0][3] += arm.at(i, 0) * -std::sin(totalAngle) * std::cos(baseAngle);  // x4
                temp[1][3] += arm.at(i, 0) * -std::sin(totalAngle) * std::sin(baseAngle);  // y4
                temp[2][3] += arm.at(i, 0) * std::cos(totalAngle);                         // z4
            }
        }
        return temp;
    }

    template <std::size_t H>
    bool inSolutionSpace(const Matrix<H, 1, double>& phis, const Matrix<H, 2, double>& solutionSpace)
    {
        for (std::size_t i = 0; i < H; ++i)
        {
            if (toDegrees(phis.at(i, 0)) < solutionSpace.at(i, 0) || toDegrees(phis.at(i, 0)) > solutionSpace.at(i, 1))
            {
                return false;
            }
        }
        return true;
    }

    template <std::size_t H>
    void limitToSolutionSpace(Matrix<H, 1, double>& phis, const Matrix<H, 2, double>& solutionSpace)
    {
        for (std::size_t i = 0; i < H; ++i)
        {
            if (toDegrees(phis.at(i, 0)) < solutionSpace.at(i, 0))
            {
                phis.at(i, 0) = toRadians(solutionSpace.at(i, 0));
            }
            else if (toDegrees(phis.at(i, 0)) > solutionSpace.at(i, 1))
            {
                phis.at(i, 0) = toRadians(solutionSpace.at(i, 1));
            }
        }
    }

    template <std::size_t W>
    Matrix<3, 1, double> calculateInverse(const Matrix<3, 1, double>& goal, Matrix<W, 1, double>& phis,
                                          const Matrix<W, 1, double>& armLengths,
                                          const Matrix<W, 2, double>& solutionSpace, double precision = 0.1,
                                          double beta = 0.1)
    {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        Matrix<3, 1, double> currentPosition = calculatePosition(phis, armLengths);
        std::cout << "start pos:" << currentPosition << std::endl;
        std::cout << "start phis: " << phis << std::endl;

        for (int i = 0; i < 10000; i++)
        {
            if (currentPosition.approxEqual(goal, precision))
            {
                std::cout << "approx equal" << std::endl;
                break;
            }

            Matrix<W, 3, double> jacobijnse = calculateJacobijnse(phis, armLengths);
            auto identiteit = jacobijnse.identity();
            auto transposedJacobijnse = jacobijnse.transpose();
            auto inverseJacobijnse = transposedJacobijnse * ((jacobijnse * transposedJacobijnse).inverse());

            Matrix<3, 1, double> deltaE = (goal - currentPosition) * beta;
            auto deltaFie = inverseJacobijnse * deltaE;
            Matrix<3, 1, double> newEndPos = calculatePosition(phis, armLengths);

            Matrix<3, 1, double> check = (newEndPos - (currentPosition + deltaE));

            if (check[0][0] > precision && check[1][0] > precision && check[2][0] > precision)
            {
                beta /= 1.8;
                continue;
            }
            else
            {
                phis += deltaFie;
                currentPosition = newEndPos;
                beta *= 1.1;
            }
        }

        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::cout << "Nieuwe phis: ";
        for (std::size_t i = 0; i < phis.getHeight(); ++i)
        {
            std::cout << std::to_string(toDegrees(phis.at(i, 0))) + ",";
        }

        std::cout << std::endl
                  << "Calculation completed in "
                  << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "microseconds"
                  << std::endl;
        std::cout << "nieuwe endpos:" << currentPosition << std::endl;

        if (!inSolutionSpace(phis, solutionSpace))
        {
            std::cout << "Some angles are outside solution space, closest possible valid values will be used. "
                      << std::endl;
        }

        return currentPosition;
    }

    virtual ~InverseKinematics()
    {
    }

  private:
    InverseKinematics()
    {
    }
};

#endif /* SRC_INVERSEKINEMATICS_H_ */
