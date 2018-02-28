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
#include <cassert>

template <std::size_t H /* amount of rows */, std::size_t W /* amount of columns */, typename T>
class Matrix
{
  public:
    static_assert(std::is_arithmetic<T>::value, "Value T must be integral T or a floating-point T.");
    static_assert(H > 0 && W > 0, "H (height) and W (width) must both be greater than 0");

    explicit Matrix(T defaultValue = 0);
    // cppcheck-suppress noExplicitConstructor
    Matrix(const std::initializer_list<T>& list);
    // cppcheck-suppress noExplicitConstructor
    Matrix(const std::initializer_list<std::initializer_list<T>>& list);

    explicit Matrix(const std::array<std::array<T, W>, H>& array);
    // cppcheck-suppress noExplicitConstructor
    Matrix(const Matrix<H, W, T>& rhs);

    /************************************************
     * Access operators
     ************************************************/

    /**
     * Gets a reference of a row in the matrix.
     * @param height Row to get a copy of.
     * @return Returns a std::array of representing the row in the matrix.
     */
    std::array<T, W>& at(std::size_t height);
    const std::array<T, W>& at(std::size_t height) const;

    /**
     * Gets a reference of a value at a position in the matrix.
     * @param height Row.
     * @param width Column.
     * @return A reference of the value at the location in the matrix.
     */
    T& at(std::size_t height, std::size_t width);
    const T& at(std::size_t height, std::size_t width) const;

    /**
     * Gets a reference to a row in the matrix.
     * @param idx Row to get reference of.
     * @return Returns a reference to a std::array of a row in the matrix.
     */
    std::array<T, W>& operator[](const std::size_t idx);

    /**
     * Gets a const reference to a row in the matrix.
     * @param idx Row to get const reference of.
     * @return Returns a const reference to a std::array of a row in the matrix.
     */
    const std::array<T, W>& operator[](const std::size_t idx) const;

    /************************************************
     * Other operators
     ************************************************/

    /**
     * Assigns a matrices to another matrix of the same size and type.
     * @param other Matrix to assign to this.
     * @return Returns this.
     */
    Matrix<H, W, T>& operator=(const Matrix<H, W, T>& rhs);

    /**
     * Compares 2 matrices. Always returns false by comparing matrices with different sizes.
     * @param rhs Matrix to compare against this.
     * @return True if the matrices are equal to each other, returns False otherwise.
     */
    bool operator==(const Matrix<H, W, T>& rhs) const;

    /**
     *@return Returns the amount of elements in the matrix (W * H).
     */
    inline static std::size_t size()
    {
        return W * H;
    }

    /**
     * @return Returns the amount of columns.
     */
    inline static std::size_t getWidth()
    {
        return W;
    }

    /**
     * @return Returns the amount of rows.
     */
    inline static std::size_t getHeight()
    {
        return H;
    }

    /************************************************************
     * Matrix functions.
     ************************************************************/

    /**
     * Gets the transpose of the matrix.
     * @return Returns a copy to the transpose of the this matrix.
     */
    Matrix<W, H, T> transpose() const;

    /**
     * Get an identity matrix, if the With and Height of the matrix are equal, throws runtime exception otherwise.
     * @return Returns an identiy matrix.
     */
    static Matrix<H, W, T> identity();

    /**
     * Get a string representation of the this matrix.
     * @return Returns string representation of the matrix.
     */
    std::string to_string() const;

    /**
     * Compares two matrices with a precision. If the rhs matrix is a different type, it will be cast to the type of the
     * this matrix.
     * @param rhs Matrix to compare this matrix against.
     * @param precision Double representing the precision to compare the matrices with.
     * @return Returns True if the rhs matrix is more precise(<) then the this matrix.
     */
    template <typename aT = T>
    bool approxEqual(const Matrix<H, W, aT>& rhs, double precision) const;

    /**
     * @brief Use gauss elimination to calculate the inverse of a square matrix
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
    /**
     * Use gauss elimination to calculate the inverse of a square matrix.
     * @return Inverse matrix.
     */
    Matrix<H, W, T> inverse();

    /************************************************************
     * Scalar operations.
     ************************************************************/

    /**
     * Multiplies matrix with a scalar.
     * @param scalar Scalar to  multiply the matrix with.
     * @return Returns a reference to the this matrix.
     */
    template <typename aT = T>
    Matrix<H, W, T>& operator*=(const aT& scalar);

    /**
     * Multiplies matrix with a scalar.
     * @param scalar Scalar to  multiply the matrix with.
     * @return Returns a matrix with the result.
     */
    template <typename aT = T>
    Matrix<H, W, T> operator*(const aT& scalar) const;

    /**
     * Divides matrix with a scalar.
     * @param scalar Scalar to  divide the matrix with.
     * @return Returns a reference to the this matrix.
     */
    template <typename aT = T>
    Matrix<H, W, T>& operator/=(const aT& scalar);

    /**
     * Divides matrix with a scalar.
     * @param scalar Scalar to  divide the matrix with.
     * @return Returns a matrix with the result.
     */
    template <typename aT = T>
    Matrix<H, W, T> operator/(const aT& scalar) const;

    /************************************************************
     * Matrix operations.
     ************************************************************/

    /**
     * Multiplies two matrices, if the Width of the this matrix is equal to the Height of the rhs matrix, throws runtime
     * exception otherwise.
     * @param rhs Matrix to multiply with the this matrix.
     * @return Returns a matrix with the result.
     */
    template <std::size_t aH, std::size_t aW>
    Matrix<H, aW, T> operator*(const Matrix<aH, aW, T>& rhs) const;

    /**
     * Adds two matrices together.
     * @param rhs Matrix to add to the this matrix.
     * @return Returns a reference to the this matrix.
     */
    Matrix<H, W, T>& operator+=(const Matrix<H, W, T>& rhs);

    /**
     * Adds two matrices together.
     * @param rhs Matrix to add to the this matrix.
     * @return Returns a matrix with the result.
     */
    Matrix<H, W, T> operator+(const Matrix<H, W, T>& rhs) const;

    /**
     * Subtracts two matrices.
     * @param rhs Matrix to subtract of the this matrix.
     * @return Returns a reference to the this matrix.
     */
    Matrix<H, W, T>& operator-=(const Matrix<H, W, T>& rhs);

    /**
     * Subtracts two matrices.
     * @param rhs Matrix to subtract of the this matrix.
     * @return Returns a matrix with the result.
     */
    Matrix<H, W, T> operator-(const Matrix<H, W, T>& rhs) const;

    virtual ~Matrix();

  private:
    /**
     * Find the correct top row, this is a row starting with a 1 otherwise the one with the highest value.
     * @param lhs Matrix to get the inverse of.
     * @param rhs Identity matrix.
     * @param index Current row.
     * @return Correctly sorted matrix.
     */
    static Matrix<H, W, T> sort_rows(Matrix<H, W, T>& lhs, Matrix<H, W, T>& rhs, std::size_t index);

    /**
     * Makes the other rows to 0.
     * @param lhs Matrix to get the inverse of.
     * @param rhs Identity matrix.
     * @param index Current row.
     * @return Returns a matrix with rows values set to 0.
     */
    static Matrix<H, W, T> substract_cols_to_zero(Matrix<H, W, T>& lhs, Matrix<H, W, T>& rhs, std::size_t index);

    /**
     * Makes the first value of the top row to 1.
     * @param lhs Matrix to get the inverse of.
     * @param rhs Identity matrix.
     * @param index Current row.
     * @return Correctly sorted matrix.
     */
    static Matrix<H, W, T> divide_row_to_one(Matrix<H, W, T>& lhs, Matrix<H, W, T>& rhs, std::size_t row);

    std::array<std::array<T, W>, H> innerMatrix;  ///< Storage container for the matrix.
};

template <std::size_t W, std::size_t H, class T>
std::ostream& operator<<(std::ostream& stream, const Matrix<H, W, T>& rhs)
{
    return stream << rhs.to_string();
}

template <std::size_t H, std::size_t W, class T>
Matrix<H, W, T>::~Matrix()
{
}

#include "Matrix.inc"

#endif /* MATRIX_HPP_ */
