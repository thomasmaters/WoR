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
            //            std::cout << "temp " << i << ": " << temp << std::endl;
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
                // std::cout << "kaas" << std::endl;
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

    template <std::size_t W, std::size_t H, class T>
    static void swapRows(Matrix<W, H, T>& matrix, std::size_t row, std::size_t otherRow)
    {
        matrix[row].swap(matrix[otherRow]);
    }

    template <std::size_t W, std::size_t H, class T>
    static std::size_t getHighestRow(Matrix<W, H, T>& matrix, std::size_t row)
    {
        T highestValue = matrix.at(row, row);
        std::size_t highestIndex = row;
        for (std::size_t i = row; i < H; ++i)
        {
            T curValue = matrix.at(i, row);
            if (curValue == 1)
            {
                return i;
            }
            else if (curValue < highestValue && curValue != 0)
            {
                highestIndex = i;
                highestValue = curValue;
            }
        }
        return highestIndex;
    }

    template <std::size_t W, std::size_t H, class T>
    static double pivaltFactor(Matrix<W, H, T>& matrix, std::size_t x, std::size_t y)
    {
        double value = matrix.at(x, y);
        if (value == 0 || (x == y && value == 1))
        {
            return 0;
        }
        else if (x == y && value != 1)
        {
            return (value - 1) / value;
        }
        else
        {
            return value;
        }
    }

    template <std::size_t W, class T>
    static std::array<T, W> multiplyArray(std::array<T, W> lhs, T factor)
    {
        for (std::size_t a = 0; a < W; a++)
        {
            lhs[a] = lhs[a] * factor;
        }
        return lhs;
    }

    template <std::size_t W, class T>
    static void subtractArray(std::array<T, W>& lhs, const std::array<T, W>& rhs)
    {
        for (std::size_t a = 0; a < W; a++)
        {
            lhs[a] -= rhs[a];
        }
    }

    template <std::size_t W, std::size_t H, class T>
    static Matrix<W, H, T> gaussEliminatie(Matrix<W, H, T> matrix, Matrix<W, H, T> identity)
    {
        if (W != H)
        {
            throw std::runtime_error("Error: Can't get inverse of non square Matrix.");
        }

        for (std::size_t row = 0; row < H; ++row)
        {
            std::size_t highestRow = getHighestRow(matrix, row);
            if (row != highestRow)
            {
                swapRows(matrix, row, highestRow);
                swapRows(identity, row, highestRow);
            }

            double pivatFactor = pivaltFactor(matrix, row, row);
            std::array<T, W> factorRow = matrix.at(row);

            subtractArray(factorRow, multiplyArray(matrix.at(row), pivatFactor));
            subtractArray(matrix[row], multiplyArray(matrix.at(row), pivatFactor));
            subtractArray(identity[row], multiplyArray(identity.at(row), pivatFactor));

            for (std::size_t subRow = 0; subRow < H; ++subRow)
            {
                double afactor = pivaltFactor(matrix, subRow, row);

                subtractArray(matrix[subRow], multiplyArray(factorRow, afactor));
                subtractArray(identity[subRow], multiplyArray(identity.at(row), afactor));
            }
            if (matrix.at(row, row) == 0)
            {
                //	            throw std::runtime_error(
                //	                "Determinant = 0, matrix is not invertible.");
            }
        }
        return identity;
    }

    template <std::size_t W>
    Matrix<3, 1, double> calculateInverse(const Matrix<3, 1, double>& goal, Matrix<W, 1, double>& phis,
                                          const Matrix<W, 1, double>& armLengths, double precision = 0.1,
                                          double beta = 0.1)
    {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        Matrix<3, 1, double> e = calculatePosition(phis, armLengths);
        std::cout << "start pos:" << e << std::endl;

        for (int i = 0; i < 10000; i++)
        {
            if (e.approxEqual(goal, precision))
            {
                std::cout << "approx equal" << std::endl;
                break;
            }
            //        while (!(e.approxEqual(goal, precision)))
            //        {
            Matrix<W, 3, double> jacobijnse = calculateJacobijnse(phis, armLengths);
            auto identiteit = jacobijnse.identity();
            auto transposedJacobijnse = jacobijnse.transpose();
            auto inverseJacobijnse = transposedJacobijnse * ((jacobijnse * transposedJacobijnse).inverse());
            ;
            //            if (W == 3)
            //            {
            //            std::cout << "jaco:" << jacobijnse << std::endl;
            //                inverseJacobijnse = jacobijnse.inverse();
            //                std::cout << "injaco" << inverseJacobijnse << std::endl;
            //                //                inverseJacobijnse =
            //                //                    gaussEliminatie(transposedJacobijnse * jacobijnse, identiteit) *
            //                //                    transposedJacobijnse;
            //                //                std::cout << "injaco2" << inverseJacobijnse << std::endl;
            //            }
            //            else
            //            {
            //            }

            //            std::cout << "jacobijnse" << jacobijnse * transposedJacobijnse << std::endl;
            //            std::cout << "inverse" << (jacobijnse * transposedJacobijnse).inverse() << std::endl;
            //	        Matrix<3, 4, double> inverseJacobijnse = transposedJacobijnse * (jacobijnse *
            // transposedJacobijnse).inverse();
            //            std::cout << "injaco" << inverseJacobijnse << std::endl;

            Matrix<3, 1, double> deltaE = (goal - e) * beta;
            auto deltaFie = inverseJacobijnse * deltaE;
            phis += deltaFie;

            if (phis.at(1, 0) <= 0)
            {
                phis[1][0] = toRadians(45);
            }

            if (phis.at(2, 0) >= 0)
            {
                phis[2][0] = toRadians(-45);
            }

            e = calculatePosition(phis, armLengths);
        }

        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::cout << "Nieuwe phis: ";
        for (std::size_t i = 0; i < phis.getWidth(); ++i)
        {
            std::cout << std::to_string(toDegrees(phis.at(0, i))) + ",";
        }

        std::cout << std::endl
                  << "Calculation completed in "
                  << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "microseconds"
                  << std::endl;
        std::cout << "nieuwe endpos:" << e << std::endl;

        return e;
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
