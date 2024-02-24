#ifndef VECTOR_H
#define VECTOR_H

template <int N>
class VectorXD {
private:
    double c_arr[N];
public:
    VectorXD();
    VectorXD(double x, double y); // by default its assumed to be a 2 column vector.
    int size() const; // return size
    void setIndex(int i, double val);
    double getIndex(int i);

    VectorXD<N> mult(double X); // Multiply by a constant
    VectorXD<N> div(double X); // Divide by a constant
    VectorXD<N> add_by_vect(VectorXD<N> Vect); // Add by vector of same size
    VectorXD<N> sub_by_vect(VectorXD<N> Vect); // Minus by vector of same size
    VectorXD<N> rotate(double rad); // rotates counterclockwise by some angle in radians.
};

#endif // VECTOR_H