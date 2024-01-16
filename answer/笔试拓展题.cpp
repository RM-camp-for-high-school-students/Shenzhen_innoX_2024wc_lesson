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

int main()
{
    srand(time(NULL));
    cout << "INPUT:" << endl;
    int** x = new int*[XROW];
    for (int i = 0; i < XROW; i++)
    {
        x[i] = new int[XCOL];
        for (int j = 0; j < XCOL; j++)
        {
            x[i][j] = rand() % 3;
            cout << x[i][j] << " ";
        }
        cout << endl;
    }

    cout << "KERNEL:" << endl;
    int** k = new int*[KROW];
    for (int i = 0; i < KROW; i++)
    {
        k[i] = new int[KCOL];
        for (int j = 0; j < KCOL; j++)
        {
            k[i][j] = rand() % 3;
            cout << k[i][j] << " ";
        }
        cout << endl;
    }

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
    
    for (int i = 0; i < XROW; i++) delete[] x[i];
    delete[] x;
    for (int i = 0; i < KROW; i++) delete[] k[i];
    delete[] k;
    for (int i = 0; i < yrow; i++) delete[] y[i];
    delete[] y;
    for (int i = 0; i < zrow; i++) delete[] z[i];
    delete[] z;
    return 0;
}