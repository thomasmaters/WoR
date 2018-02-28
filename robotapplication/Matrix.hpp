/*
 * Matrix.hpp
 *
 *  Created on: 9 Feb 2017
 *      Author: Thomas
 */

#ifndef MATRIX_HPP_
#define MATRIX_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <vector>

template <std::size_t W, std::size_t H, class T = int> class Matrix
{
  public:
    static_assert(std::is_arithmetic<T>::value,
                  "Value T must be integral T or a floating-point T.");

    Matrix(T defaultValue = 0);

    Matrix(const std::initializer_list<T>& list);

    Matrix(const std::initializer_list<std::initializer_list<T>>& list);

    Matrix(const std::array<std::array<T, W>, H>& array);

    Matrix(const Matrix<W, H, T>& other);

    std::array<T, W> at(std::size_t height) const;

    T at(std::size_t height, std::size_t width) const;

    std::array<T, W>& operator[](const std::size_t idx);

    const Matrix<W, H, T>& operator=(const Matrix<W, H, T>& other);

    /*
     * Can't assign a matrix of a different size, this returned.
     */
    template <std::size_t aWidth, std::size_t aHeight, class aT>
    const Matrix operator=(const Matrix<aWidth, aHeight, aT>& other);

    /*
     * Compares 2 matricies. Always returns false by comparing matricies with
     * different sizes.
     */
    template <std::size_t aWidth, std::size_t aHeight, class aT>
    bool operator==(const Matrix<aWidth, aHeight, aT>& other);

    /*
     *Returns the amount of elements in the matrix.
     */
    inline std::size_t size() const
    {
        return W * H;
    }

    inline std::size_t getWidth() const
    {
        return W;
    }

    inline std::size_t getHeight() const
    {
        return H;
    }

    /************************************************************
     * Matrix functions.
     ************************************************************/

    Matrix<H, W, T> transpose() const;

    Matrix<W, H, T> identity() const;

    template <std::size_t aW, std::size_t aH, class aT>
    bool approxEqual(const Matrix<aW, aH, aT>& rhs, double precision) const;

    template <class aT = T> Matrix<W, H, T> pointWiseMultiply(const T value);

    std::string to_string() const;

    /**
     * @brief use gauss elimination to calculate the inverse of a square matrix
     * private function sort rows
     * 1 vind de correcte bovenste rij dit is de eerste rij die met 1 begint en
     * anders de grootste
     * 2 swap deze rij naar de bovenste rij
     * private function devide_row_to_one
     * 3 maak van de bovenste rij het eerste element 1 door de rij te delen door
     * het eerste element
     * private function substract cols_to_zero
     * 4 maak van de overige rijen het eerste element 0 door de rij_x - eerste
     * rij x het eerste element van de rij_x
     * uses a for loop based on height
     * 5 herhaal bovenenste rij wordt volgende rij en kijken naar het volgende
     * element
     * @return the inverse of the current matrix
     */
    Matrix<W, H, T> inverse();

  private:
    Matrix<W, H, T> sort_rows(Matrix<W, H, T>& lhs, Matrix<W, H, T>& rhs,
                              std::size_t index);

    Matrix<W, H, T> substract_cols_to_zero(Matrix<W, H, T>& lhs,
                                           Matrix<W, H, T>& rhs,
                                           std::size_t index);

    Matrix<W, H, T> divide_row_to_one(Matrix<W, H, T>& lhs,
                                      Matrix<W, H, T>& rhs, std::size_t row);

  public:
    /************************************************************
     * Scalar operations.
     ************************************************************/

    Matrix<W, H, T>& operator*=(const T& scalar);

    Matrix<W, H, T> operator*(const T& scalar) const;

    template <class aT = T> Matrix<W, H, T>& operator/=(const aT& scalar);

    template <class aT = T> Matrix<W, H, T> operator/(const aT& scalar) const;

    /************************************************************
     * Pointwise operations.
     ************************************************************/

    Matrix<W, H, T>& operator+=(const T& value);

    Matrix<W, H, T> operator+(const T& value);

    template <class aT = T> Matrix<W, H, T>& operator-=(const aT& value);

    template <class aT = T> Matrix<W, H, T> operator-(const aT& value) const;

    /************************************************************
     * Matrix operations.
     ************************************************************/
    template <std::size_t aWidth, std::size_t aHeight>
    Matrix<aWidth, H, T>
    operator*(const Matrix<aWidth, aHeight, T>& other) const;

    Matrix<W, H, T>& operator+=(const Matrix<W, H, T>& other);

    Matrix<W, H, T> operator+(const Matrix<W, H, T>& other);

    Matrix<W, H, T>& operator-=(const Matrix<W, H, T>& other);

    Matrix<W, H, T> operator-(Matrix<W, H, T>& other);

    virtual ~Matrix();

  private:
    std::array<std::array<T, W>, H> innerMatrix;
};

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T>::Matrix(T defaultValue)
{
    for (std::size_t i = 0; i < H; ++i) {
        innerMatrix.at(i).fill(defaultValue);
    }
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T>::Matrix(const std::initializer_list<T>& list)
{
    if (list.size() != W * H) {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        throw std::runtime_error("Error: Can't initialize Matrix, "
                                 "because initalizer list size does "
                                 "not match template size.");
    }

    auto it = list.begin();
    for (size_t i = 0; i < H; ++i) {
        for (size_t j = 0; j < W; ++j) {
            innerMatrix.at(i % H).at(j) = *it;
            ++it;
        }
    }
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T>::Matrix(
    const std::initializer_list<std::initializer_list<T>>& list)
{
    if (list.size() != H) {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        throw std::runtime_error("Error: Can't initialize Matrix, "
                                 "because initalizer list size does "
                                 "not match template size.");
    }

    auto it = list.begin();
    for (size_t i = 0; i < list.size(); ++i) {
        if (it->size() != W) {
            std::cout << __PRETTY_FUNCTION__ << std::endl;
            throw std::runtime_error("Error: Can't initialize Matrix, "
                                     "because initalizer list size does "
                                     "not match template size.");
        }

        auto itt = (*it).begin();
        for (size_t j = 0; j < (*it).size(); ++j) {
            innerMatrix.at(i).at(j) = *itt;
            ++itt;
        }
        ++it;
    }
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T>::Matrix(const std::array<std::array<T, W>, H>& array)
    : innerMatrix(array)
{
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T>::Matrix(const Matrix<W, H, T>& other)
    : innerMatrix(other.innerMatrix)
{
    //        std::cout << __PRETTY_FUNCTION__ << std::endl;
}

template <std::size_t W, std::size_t H, class T>
std::array<T, W> Matrix<W, H, T>::at(std::size_t height) const
{
    return innerMatrix.at(height);
}

template <std::size_t W, std::size_t H, class T>
T Matrix<W, H, T>::at(std::size_t height, std::size_t width) const
{
    return innerMatrix.at(height).at(width);
}

template <std::size_t W, std::size_t H, class T>
std::array<T, W>& Matrix<W, H, T>::operator[](const std::size_t idx)
{
    return innerMatrix.at(idx);
}

template <std::size_t W, std::size_t H, class T>
const Matrix<W, H, T>& Matrix<W, H, T>::operator=(const Matrix<W, H, T>& other)
{
    innerMatrix = other.innerMatrix;

    return *this;
}

template <std::size_t W, std::size_t H, class T>
template <std::size_t aWidth, std::size_t aHeight, class aT>
const Matrix<W, H, T> Matrix<W, H, T>::
operator=(const Matrix<aWidth, aHeight, aT>& other)
{
    return *this;
}

template <std::size_t W, std::size_t H, class T>
template <std::size_t aWidth, std::size_t aHeight, class aT>
bool Matrix<W, H, T>::operator==(const Matrix<aWidth, aHeight, aT>& other)
{
    if (&other == this) {
        return true;
    }

    return W == aWidth && H == aHeight && innerMatrix == other.innerMatrix;
}

template <std::size_t W, std::size_t H, class T>
std::ostream& operator<<(std::ostream& stream, const Matrix<W, H, T>& other)
{
    return stream << other.to_string();
}

template <std::size_t W, std::size_t H, class T>
Matrix<H, W, T> Matrix<W, H, T>::transpose() const
{
    Matrix<H, W, T> temp = Matrix<H, W, T>();
    for (std::size_t i = 0; i < H; ++i) {
        for (std::size_t j = 0; j < W; ++j) {
            temp[j][i] = innerMatrix[i][j];
        }
    }
    return temp;
}

template <std::size_t W, std::size_t H, class T>
template <std::size_t aW, std::size_t aH, class aT>
bool Matrix<W, H, T>::approxEqual(const Matrix<aW, aH, aT>& rhs,
                                  double precision) const
{
    for (std::size_t i = 0; i < H; ++i) {
        for (std::size_t j = 0; j < W; ++j) {
            if (std::abs(at(i, j) - rhs.at(i, j)) > precision)
                return false;
        }
    }
    return true;
}

template <std::size_t W, std::size_t H, class T>
template <class aT>
Matrix<W, H, T> Matrix<W, H, T>::pointWiseMultiply(const T value)
{
    for (std::size_t i = 0; i < H; ++i) {
        for (std::size_t j = 0; j < W; ++j) {
            innerMatrix[j][i] = value * innerMatrix[i][j];
        }
    }
    return *this;
}

template <std::size_t W, std::size_t H, class T>
std::string Matrix<W, H, T>::to_string() const
{
    std::string temp =
        "[Matrix]<" + std::to_string(W) + "," + std::to_string(H) + ">\n{\n";
    for (std::size_t i = 0; i < H; ++i) {
        for (std::size_t j = 0; j < W; ++j) {
            temp += std::to_string(innerMatrix[i][j]) + ",";
        }
        temp += "\n";
    }
    temp += "}";
    return temp;
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T>& Matrix<W, H, T>::operator*=(const T& scalar)
{
    for (std::size_t i = 0; i < H; ++i) {
        for (std::size_t j = 0; j < W; ++j) {
            innerMatrix.at(i).at(j) *= scalar;
        }
    }
    return *this;
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T> Matrix<W, H, T>::operator*(const T& scalar) const
{
    Matrix<W, H, T> temp(*this);
    return temp *= scalar;
}

template <std::size_t W, std::size_t H, class T>
template <class aT>
Matrix<W, H, T>& Matrix<W, H, T>::operator/=(const aT& scalar)
{
    for (std::size_t i = 0; i < H; ++i) {
        for (std::size_t j = 0; j < W; ++j) {
            innerMatrix[i][j] /= scalar;
        }
    }
    return *this;
}

template <std::size_t W, std::size_t H, class T>
template <class aT>
Matrix<W, H, T> Matrix<W, H, T>::operator/(const aT& scalar) const
{
    Matrix<W, H, T> temp = *this;
    return temp /= scalar;
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T>& Matrix<W, H, T>::operator+=(const T& value)
{
    for (std::size_t i = 0; i < H; ++i) {
        for (std::size_t j = 0; j < W; ++j) {
            innerMatrix[i][j] += value;
        }
    }
    return *this;
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T> Matrix<W, H, T>::operator+(const T& value)
{
    Matrix<W, H, T> temp = *this;
    return temp += value;
}

template <std::size_t W, std::size_t H, class T>
template <class aT>
Matrix<W, H, T>& Matrix<W, H, T>::operator-=(const aT& value)
{
    for (std::size_t i = 0; i < H; ++i) {
        for (std::size_t j = 0; j < W; ++j) {
            innerMatrix[i][j] -= value;
        }
    }
    return *this;
}

template <std::size_t W, std::size_t H, class T>
template <class aT>
Matrix<W, H, T> Matrix<W, H, T>::operator-(const aT& value) const
{
    Matrix<W, H, T> temp = *this;
    return temp -= value;
}

template <std::size_t W, std::size_t H, class T>
template <std::size_t aWidth, std::size_t aHeight>
Matrix<aWidth, H, T> Matrix<W, H, T>::
operator*(const Matrix<aWidth, aHeight, T>& other) const
{
    if (W != aHeight) {
        throw std::runtime_error("Error: Can't multiply matrixes, because "
                                 "their size doesn't allow "
                                 "that.");
    }

    Matrix<aWidth, aHeight, T> temp = other;
    Matrix<aWidth, H, T> tempMatrix = Matrix<aWidth, H, T>();

    for (std::size_t i = 0; i < H; ++i) {
        for (std::size_t j = 0; j < aWidth; ++j) {
            for (std::size_t k = 0; k < W; ++k) {
                tempMatrix[i][j] += innerMatrix[i][k] * temp[k][j];
            }
        }
    }
    return tempMatrix;
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T>& Matrix<W, H, T>::operator+=(const Matrix<W, H, T>& other)
{
    for (std::size_t i = 0; i < H; ++i) {
        for (std::size_t j = 0; j < W; ++j) {
            innerMatrix[i][j] += other.at(i, j);
        }
    }
    return *this;
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T> Matrix<W, H, T>::operator+(const Matrix<W, H, T>& other)
{
    Matrix<W, H, T> temp(*this);
    return temp += other;
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T>& Matrix<W, H, T>::operator-=(const Matrix<W, H, T>& other)
{
    for (std::size_t i = 0; i < H; ++i) {
        for (std::size_t j = 0; j < W; ++j) {
            innerMatrix[i][j] -= other.at(i, j);
        }
    }
    return *this;
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T> Matrix<W, H, T>::operator-(Matrix<W, H, T>& other)
{
    Matrix<W, H, T> temp(*this);
    return temp -= other;
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T> Matrix<W, H, T>::identity() const
{

    if (W != H) {
        throw std::runtime_error("Error: Can't create identity matrix"
                                 "Because the width is not equal to height");
    }

    Matrix<W, H, T> identity;

    for (std::size_t i = 0; i < H; ++i) {
        identity[i][i] = 1;
    }
    return identity;
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T> Matrix<W, H, T>::sort_rows(Matrix<W, H, T>& lhs,
                                           Matrix<W, H, T>& rhs,
                                           std::size_t index)
{
    std::size_t j = index;
    for (std::size_t i = index; i < H; ++i) {

        std::array<T, W> ref = lhs[j];
        std::array<T, W> ref2 = lhs[i];

        std::array<T, W> iden = rhs[j];
        std::array<T, W> iden2 = rhs[i];

        if (ref2[index] == 1 || ref2[index] == -1) {
            lhs[j].swap(ref2);
            lhs[i].swap(ref);

            rhs[j].swap(iden2);
            rhs[i].swap(iden);
            ++j;
        } else if (ref[index] > ref2[index]) {
            lhs[i].swap(ref);
            lhs[j].swap(ref2);

            rhs[i].swap(iden);
            rhs[j].swap(iden2);
            ++j;
        }
    }
    return lhs;
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T> Matrix<W, H, T>::substract_cols_to_zero(Matrix<W, H, T>& lhs,
                                                        Matrix<W, H, T>& rhs,
                                                        std::size_t index)
{

    std::array<T, W> row = lhs[index];
    std::array<T, W> rowi = rhs[index];
    for (std::size_t i = 0; i < H; ++i) {
        if (row != lhs[i]) {
            double factor = lhs[i][index];
            for (std::size_t j = 0; j < W; ++j) {
                double subs = (row[j] * factor);
                double subsi = (rowi[j] * factor);
                lhs[i][j] -= subs;
                rhs[i][j] -= subsi;
            }
        }
    }
    return lhs;
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T> Matrix<W, H, T>::divide_row_to_one(Matrix<W, H, T>& lhs,
                                                   Matrix<W, H, T>& rhs,
                                                   std::size_t row)
{
    double factor = (1 / lhs[row][row]);

    for (std::size_t i = 0; i < W; ++i) {
        lhs[row][i] *= factor;
        rhs[row][i] *= factor;
    }
    return lhs;
}

template <std::size_t W, std::size_t H, class T>
Matrix<W, H, T> Matrix<W, H, T>::inverse()
{
    if (W != H) {
        throw std::runtime_error("Current implementation only supports "
                                 "matrices with the same height as width ");
    }

    Matrix<W, H, T> identity = this->identity();
    Matrix<W, H, T> temp = *(this);
    for (std::size_t i = 0; i < H; ++i) {
        sort_rows(temp, identity, i);
        divide_row_to_one(temp, identity, i);
        substract_cols_to_zero(temp, identity, i);
    }

    return identity;
}

template <std::size_t W, std::size_t H, class T> Matrix<W, H, T>::~Matrix()
{
}

#endif /* MATRIX_HPP_ */
