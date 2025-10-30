struct alignas(64) Matrix
{
    float data[16][16];
};

Matrix mult(const Matrix& lhs, const Matrix& rhs)
{
    Matrix result;
    for (int i = 0; i < 16; ++i) {
        for (int k = 0; k < 16; ++k) {
            float sum = 0;
            for (int n = 0; n < 16; ++n) {
                sum += lhs.data[i][n] * rhs.data[n][k];
            }
            result.data[i][k] = sum;
        }
    }
    return result;
}

Matrix lhs;

Matrix mult_static(const Matrix& rhs)
{
    return mult(lhs, rhs);
}
