#include <iostream>
#include <ctime>
#define XROW 8
#define XCOL 8
#define KROW 2
#define KCOL 2
#define SROW 2
#define SCOL 2
using namespace std;

/*
    创建一个新的二维数组，返回它的指针
    row, col: 新数组的行列数
*/
int** new2d(int row, int col)
{
    int** res = new int*[row];
    for (int i = 0; i < row; i++)
    {
        res[i] = new int[col];
        for (int j = 0; j < col; j++)
        {
            res[i][j] = rand() % 3;
            cout << res[i][j] << " ";
        }
        cout << endl;
    }
    return res;
}

/*
    卷积操作，返回输出数组
    x, xrow, xcol: 输入数组和它的行列数
    kernel, krow, kcol: 卷积核和它的行列数
    yrow, ycol: 输出数组的行列数
    srow, scol: 行列的步长
*/
int** conv2d(int** x, int xrow, int xcol, int** kernel, int krow, int kcol, int* yrow, int* ycol, int srow=1, int scol=1)
{
    *yrow = (xrow - krow) / srow + 1;
    *ycol = (xcol - kcol) / scol + 1;
    int** y = new int*[*yrow];
    for (int i = 0; i < *yrow; i++)
    {
        y[i] = new int[*ycol];
        for (int j = 0; j < *ycol; j++)
        {
            y[i][j] = 0;
            for (int ki = 0; ki < krow; ki++)
                for (int kj = 0; kj < kcol; kj++)
                    y[i][j] += x[i * srow + ki][j * scol + kj] * kernel[ki][kj];
        }
    }
    return y;
}

/*
    池化操作，返回输出数组
    x, xrow, xcol: 输入数组和它的行列数
    krow, kcol: 池化过滤器的行列数
    yrow, ycol: 输出数组的行列数
    srow, scol: 行列的步长
*/
float** avgpool(int** x, int xrow, int xcol, int frow, int fcol, int* yrow, int* ycol, int srow=1, int scol=1)
{
    *yrow = (xrow - frow) / srow + 1;
    *ycol = (xcol - fcol) / scol + 1;
    int num = frow * fcol;
    float** y = new float*[*yrow];
    for (int i = 0; i < *yrow; i++)
    {
        y[i] = new float[*ycol];
        for (int j = 0; j < *ycol; j++)
        {
            y[i][j] = 0;
            for (int fi = 0; fi < frow; fi++)
                for (int fj = 0; fj < fcol; fj++)
                    y[i][j] += x[i * srow + fi][j * scol + fj];
            y[i][j] /= num;
        }
    }
    return y;
}

void delete2d(int** x, int row)
{
    for (int i = 0; i < row; i++) delete[] x[i];
    delete[] x;
}

void delete2d(float** x, int row)
{
    for (int i = 0; i < row; i++) delete[] x[i];
    delete[] x;
}

int main()
{
    srand(time(NULL));
    cout << "INPUT:" << endl;
    int** x = new2d(XROW, XCOL);
    cout << "KERNEL:" << endl;
    int** k = new2d(KROW, KCOL);

    int yrow, ycol;
    int** y = conv2d(x, XROW, XCOL, k, KROW, KCOL, &yrow, &ycol, SROW, SCOL);
    cout << "CONV:" << endl;
    for (int i = 0; i < yrow; i++)
    {
        for (int j = 0; j < ycol; j++) cout << y[i][j] << " ";
        cout << endl;
    }

    int zrow, zcol;
    float** z = avgpool(y, yrow, ycol, KROW, KCOL, &zrow, &zcol, SROW, SCOL);
    cout << "POOL:" << endl;
    for (int i = 0; i < zrow; i++)
    {
        for (int j = 0; j < zcol; j++) cout << z[i][j] << " ";
        cout << endl;
    }
    
    delete2d(x, XROW);
    delete2d(k, KROW);
    delete2d(y, yrow);
    delete2d(z, zrow);
    return 0;
}