/************************************************************
 * Constructors.
 ************************************************************/

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T>::Matrix(T defaultValue)
{
    for (std::size_t i = 0; i < H; ++i)
    {
        innerMatrix.at(i).fill(defaultValue);
    }
}

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T>::Matrix(const std::initializer_list<T>& list)
{
    assert(list.size() == H * W);

    auto row_iter = list.begin();
    for (std::size_t row = 0; row < H; ++row)
    {
        for (std::size_t column = 0; column < W; ++column)
        {
            innerMatrix.at(row).at(column) = *row_iter;
            ++row_iter;
        }
    }
}

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T>::Matrix(const std::initializer_list<std::initializer_list<T>>& list)
{
    // Check the arguments, the static assert assures that there is at least 1 M an N!
    assert(list.size() == H && (*list.begin()).size() == W);

    auto row_iter = list.begin();
    for (std::size_t row = 0; row < list.size(); ++row)
    {
        auto column_iter = (*row_iter).begin();
        for (std::size_t column = 0; column < (*row_iter).size(); ++column)
        {
            innerMatrix.at(row).at(column) = *column_iter;
            ++column_iter;
        }
        ++row_iter;
    }
}

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T>::Matrix(const std::array<std::array<T, W>, H>& array) : innerMatrix(array)
{
}

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T>::Matrix(const Matrix<H, W, T>& rhs) : innerMatrix(rhs.innerMatrix)
{
}

/************************************************************
 * Acces operators.
 ************************************************************/

template <std::size_t H, std::size_t W, typename T>
std::array<T, W>& Matrix<H, W, T>::at(std::size_t height)
{
    return innerMatrix.at(height);
}

template <std::size_t H, std::size_t W, typename T>
const std::array<T, W>& Matrix<H, W, T>::at(std::size_t height) const
{
    return innerMatrix.at(height);
}

template <std::size_t H, std::size_t W, typename T>
T& Matrix<H, W, T>::at(std::size_t height, std::size_t width)
{
    return innerMatrix.at(height).at(width);
}

template <std::size_t H, std::size_t W, typename T>
const T& Matrix<H, W, T>::at(std::size_t height, std::size_t width) const
{
    return innerMatrix.at(height).at(width);
}

template <std::size_t H, std::size_t W, typename T>
std::array<T, W>& Matrix<H, W, T>::operator[](const std::size_t idx)
{
    return innerMatrix.at(idx);
}

template <std::size_t H, std::size_t W, typename T>
const std::array<T, W>& Matrix<H, W, T>::operator[](const std::size_t idx) const
{
    return innerMatrix.at(idx);
}

/************************************************************
 * Other operations.
 ************************************************************/

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T>& Matrix<H, W, T>::operator=(const Matrix<H, W, T>& rhs)
{
    if (this != &rhs)
    {
        innerMatrix = rhs.innerMatrix;
    }

    return *this;
}

template <std::size_t H, std::size_t W, typename T>
bool Matrix<H, W, T>::operator==(const Matrix<H, W, T>& rhs) const
{
    return innerMatrix == rhs.innerMatrix;
}

/************************************************************
 * Matrix functions.
 ************************************************************/

template <std::size_t H, std::size_t W, typename T>
Matrix<W, H, T> Matrix<H, W, T>::transpose() const
{
    Matrix<W, H, T> temp = Matrix<W, H, T>();
    for (std::size_t i = 0; i < H; ++i)
    {
        for (std::size_t j = 0; j < W; ++j)
        {
            temp[j][i] = innerMatrix[i][j];
        }
    }
    return temp;
}

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T> Matrix<H, W, T>::identity()
{
    if (H != W)
    {
        throw std::runtime_error("Current implementation only supports "
                                 "matrices with the same height as width ");
    }
    Matrix<H, W, T> identity;

    for (std::size_t i = 0; i < H; ++i)
    {
        identity[i][i] = 1;
    }
    return identity;
}

template <std::size_t H, std::size_t W, typename T>
std::string Matrix<H, W, T>::to_string() const
{
    std::string temp = "[Matrix]<" + std::to_string(W) + "," + std::to_string(H) + ">\n{\n";
    for (std::size_t i = 0; i < H; ++i)
    {
        for (std::size_t j = 0; j < W; ++j)
        {
            temp += std::to_string(innerMatrix[i][j]) + ",";
        }
        temp += "\n";
    }
    temp += "}";
    return temp;
}

template <std::size_t H, std::size_t W, typename T>
template <typename aT>
bool Matrix<H, W, T>::approxEqual(const Matrix<H, W, aT>& rhs, double precision) const
{
    for (std::size_t i = 0; i < H; ++i)
    {
        for (std::size_t j = 0; j < W; ++j)
        {
            if (std::abs(at(i, j) - rhs.at(i, j)) > precision)
                return false;
        }
    }
    return true;
}

/************************************************************
 * Inverse functions.
 ************************************************************/

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T> Matrix<H, W, T>::sort_rows(Matrix<H, W, T>& lhs, Matrix<H, W, T>& rhs, std::size_t index)
{
    std::size_t j = index;
    for (std::size_t i = index; i < H; ++i)
    {
        std::array<T, W> ref = lhs[j];
        std::array<T, W> ref2 = lhs[i];

        std::array<T, W> iden = rhs[j];
        std::array<T, W> iden2 = rhs[i];

        if (ref2[index] == 1 || ref2[index] == -1)
        {
            lhs[j].swap(ref2);
            lhs[i].swap(ref);

            rhs[j].swap(iden2);
            rhs[i].swap(iden);
            ++j;
        }
        else if (ref[index] > ref2[index])
        {
            lhs[i].swap(ref);
            lhs[j].swap(ref2);

            rhs[i].swap(iden);
            rhs[j].swap(iden2);
            ++j;
        }
    }
    return lhs;
}

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T> Matrix<H, W, T>::substract_cols_to_zero(Matrix<H, W, T>& lhs, Matrix<H, W, T>& rhs, std::size_t index)
{
    std::array<T, W> row = lhs[index];
    std::array<T, W> rowi = rhs[index];
    for (std::size_t i = 0; i < H; ++i)
    {
        if (row != lhs[i])
        {
            double factor = lhs[i][index];
            for (std::size_t j = 0; j < W; ++j)
            {
                lhs[i][j] -= (row[j] * factor);
                rhs[i][j] -= (rowi[j] * factor);
            }
        }
    }
    return lhs;
}

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T> Matrix<H, W, T>::divide_row_to_one(Matrix<H, W, T>& lhs, Matrix<H, W, T>& rhs, std::size_t row)
{
    double factor = (1 / lhs[row][row]);

    for (std::size_t i = 0; i < W; ++i)
    {
        lhs[row][i] *= factor;
        rhs[row][i] *= factor;
    }
    return lhs;
}

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T> Matrix<H, W, T>::inverse()
{
    if (W != H)
    {
        throw std::runtime_error("Current implementation only supports "
                                 "matrices with the same height as width ");
    }

    Matrix<H, W, T> identity = this->identity();
    Matrix<H, W, T> temp = *(this);
    for (std::size_t i = 0; i < H; ++i)
    {
        sort_rows(temp, identity, i);
        divide_row_to_one(temp, identity, i);
        substract_cols_to_zero(temp, identity, i);
    }

    return identity;
}

/************************************************************
 * Scalar operators.
 ************************************************************/

template <std::size_t H, std::size_t W, typename T>
template <typename aT>
Matrix<H, W, T>& Matrix<H, W, T>::operator*=(const aT& scalar)
{
    for (std::size_t i = 0; i < H; ++i)
    {
        for (std::size_t j = 0; j < W; ++j)
        {
            innerMatrix.at(i).at(j) *= scalar;
        }
    }
    return *this;
}

template <std::size_t H, std::size_t W, typename T>
template <typename aT>
Matrix<H, W, T> Matrix<H, W, T>::operator*(const aT& scalar) const
{
    Matrix<H, W, T> temp(*this);
    return temp *= scalar;
}

template <std::size_t H, std::size_t W, typename T>
template <typename aT>
Matrix<H, W, T>& Matrix<H, W, T>::operator/=(const aT& scalar)
{
    for (std::size_t i = 0; i < H; ++i)
    {
        for (std::size_t j = 0; j < W; ++j)
        {
            innerMatrix[i][j] /= scalar;
        }
    }
    return *this;
}

template <std::size_t H, std::size_t W, typename T>
template <typename aT>
Matrix<H, W, T> Matrix<H, W, T>::operator/(const aT& scalar) const
{
    Matrix<H, W, T> temp = *this;
    return temp /= scalar;
}

/************************************************************
 * Matrix operations.
 ************************************************************/

template <std::size_t H, std::size_t W, typename T>
template <std::size_t aH, std::size_t aW>
Matrix<H, aW, T> Matrix<H, W, T>::operator*(const Matrix<aH, aW, T>& rhs) const
{
    if (W != aH)
    {
        throw std::runtime_error("Error: Can't multiply matrixes, because "
                                 "their size doesn't allow "
                                 "that.");
    }

    Matrix<H, aW, T> tempMatrix = Matrix<H, aW, T>();

    for (std::size_t i = 0; i < H; ++i)
    {
        for (std::size_t j = 0; j < aW; ++j)
        {
            for (std::size_t k = 0; k < W; ++k)
            {
                tempMatrix[i][j] += innerMatrix[i][k] * rhs[k][j];
            }
        }
    }
    return tempMatrix;
}

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T>& Matrix<H, W, T>::operator+=(const Matrix<H, W, T>& rhs)
{
    for (std::size_t i = 0; i < H; ++i)
    {
        for (std::size_t j = 0; j < W; ++j)
        {
            innerMatrix[i][j] += rhs.at(i, j);
        }
    }
    return *this;
}

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T> Matrix<H, W, T>::operator+(const Matrix<H, W, T>& rhs) const
{
    Matrix<H, W, T> temp(*this);
    return temp += rhs;
}

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T>& Matrix<H, W, T>::operator-=(const Matrix<H, W, T>& rhs)
{
    for (std::size_t i = 0; i < H; ++i)
    {
        for (std::size_t j = 0; j < W; ++j)
        {
            innerMatrix[i][j] -= rhs.at(i, j);
        }
    }
    return *this;
}

template <std::size_t H, std::size_t W, typename T>
Matrix<H, W, T> Matrix<H, W, T>::operator-(const Matrix<H, W, T>& rhs) const
{
    Matrix<H, W, T> temp(*this);
    return temp -= rhs;
}


