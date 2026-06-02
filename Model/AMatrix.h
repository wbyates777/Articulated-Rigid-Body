/* AMatrix Template Class
 
 $$$$$$$$$$$$$$$$$$$$$$$$
 $   AMatrix.h - defs   $
 $$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 A simple, disposable 2D structure to hold values
 
*/

#ifndef __AMATRIX_H__
#define __AMATRIX_H__


#include <iostream>
#include <vector>



template <typename T>
class AMatrix
{

public:
    AMatrix(void) : m_rows(0), m_cols(0), m_data() {}
    AMatrix(int rows, int cols)  : m_rows(rows), m_cols(cols), m_data(rows * cols) {}
     AMatrix(int rows, int cols, const T& val) : m_rows(rows), m_cols(cols), m_data(rows * cols, val) {}
    ~AMatrix(void)=default;
    
    void
    clear( void )
    {
        m_rows = m_cols = 0;
        m_data.clear();
    }
    
    void 
    set(int rows, int cols, const T& val = T()) 
    {
        m_rows = rows; m_cols = cols;
        m_data.assign(rows * cols, val);
    }
    
    void 
    set(const T& val)
    {
        std::fill(m_data.begin(), m_data.end(), val);
    }

    class RowProxy 
    {
        public:
            RowProxy(T* ptr) : row_ptr(ptr) {}
            T& operator[](int c) { return row_ptr[c]; }
        private:
            T* row_ptr;
    };

    class ConstRowProxy 
    {
        public:
            ConstRowProxy(const T* ptr) : row_ptr(ptr) {}
            const T& operator[](int c) const { return row_ptr[c]; }
        private:
            const T* row_ptr;
    };
    
    RowProxy 
    operator[](int r)   {  return RowProxy(&m_data[r * m_cols]); }

    ConstRowProxy 
    operator[](int r) const  {  return ConstRowProxy(&m_data[r * m_cols]); }
    
    BScalar&
    operator[](int r, int c) { return &m_data[(r * m_cols) + c]; }
    
    const BScalar&
    operator[](int r, int c) const { return &m_data[(r * m_cols) + c]; }
    
    int 
    rows( void ) const { return (int) m_rows; }
    
    int 
    cols( void ) const { return (int) m_cols; }

    T* 
    data( void ) { return m_data.data(); }
    
    const T* 
    data( void ) const { return m_data.data(); }
    
    bool 
    operator!=( const AMatrix<T> &rhs ) const { return !(*this == rhs); }
    
    bool 
    operator==( const AMatrix<T> &rhs ) const { return m_rows == rhs.m_rows && m_cols == rhs.m_cols && m_data == rhs.m_data; } 
    
private:
   

    int m_rows;
    int m_cols;
    std::vector<T> m_data;
};




template <class T>
AMatrix<T> transpose( const AMatrix<T>& m ) 
{
    AMatrix<T> retVal(m.cols(), m.rows());
    for (int i = 0; i < m.rows(); ++i)
    {
        for (int j = 0; j < m.cols(); ++j)
        {
            retVal[j][i] = m[i][j];
        }
    }
    return retVal;
}

template <class T>
std::ostream&
operator<<( std::ostream& ostr, const AMatrix<T>& m ) 
{
    ostr << m.rows() << ' ' << m.cols() << '\n';
    for (int i = 0; i < m.rows(); ++i)
    {
        for (int j = 0; j < m.cols(); ++j)
        {
            ostr << m[i][j] << ' ';
        }
        ostr << '\n';
    }
    ostr << '\n';
    return ostr;
}

template <class T>
std::istream&
operator>>( std::istream& istr, AMatrix<T>& m )
{
    int r = 0, c = 0;
    istr >> r;
    istr >> c;
    
    m.set(r,c);    
    for (int i = 0; i < m.rows(); ++i)
    {
        for (int j = 0; j < m.cols(); ++j)
        {
            istr >> m[i][j];
        }
    }
    return istr;
}


typedef AMatrix<BScalar> BMatrix;

#endif // __AMATRIX_H__ 


