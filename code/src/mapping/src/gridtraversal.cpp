#include <mapping/gridtraversal.hpp>
#include <cmath>
using namespace std;

GridTraversal::GridTraversal(double x0, double y0, double x1, double y1)
{
    mX0 = x0;
    mY0 = y0;
    mX1 = x1;
    mY1 = y1;

    init();
}

void GridTraversal::get(int &i, int &j)
{
    i = mI;
    j = mJ;
}

bool GridTraversal::next(int &i, int &j)
{
    if ((mTMaxX < mTMaxY && mTDeltaX != 0.0) || mTDeltaY == 0.0) {
        mTMaxX += mTDeltaX;
        mI += mStepX;
    } else {
        mTMaxY += mTDeltaY;
        mJ += mStepY;
    }

    i = mI;
    j = mJ;

    bool hasNext = (((mStepX == 1 ? mI < mI1 : mI > mI1) && (mStepY == 1 ? mJ < mJ1 + 1 : mJ > mJ1 - 1))
                 || ((mStepY == 1 ? mJ < mJ1 : mJ > mJ1) && (mStepX == 1 ? mI < mI1 + 1 : mI > mI1 - 1)));
    return hasNext;
}

double tMax(double val, int step)
{
    double t = floor(val + 1) - val;
    return (step == 1 || t == 1 ? t : 1.0 - t);
}

void GridTraversal::init()
{
    const double eps = 0.001;
    mI = int(mX0 + eps);
    mJ = int(mY0 + eps);
    mI1 = int(mX1 + eps);
    mJ1 = int(mY1 + eps);

    mStepX = mStepY = 0;
    mTDeltaX = mTDeltaY = mTMaxX = mTMaxY = 0.0;

    if (mX1 != mX0 || mY1 != mY0) {
        mStepX = (mX1 >= mX0 ? 1 : -1);
        mStepY = (mY1 >= mY0 ? 1 : -1);

        if (mX1 == mX0) {
            mTDeltaY = 1.0;
            mTMaxY = tMax(mY0, mStepY);
        } else if (mY1 == mY0) {
            mTDeltaX = 1.0;
            mTMaxX = tMax(mX0, mStepX);
        } else {
            double m = (mY1 - mY0) / (mX1 - mX0);
            double m2 = m * m;
            mTDeltaX = sqrt(m2 + 1);
            mTDeltaY = sqrt(1.0 / m2 + 1.0);
            mTMaxX = tMax(mX0, mStepX) * mTDeltaX;
            mTMaxY = tMax(mY0, mStepY) * mTDeltaY;
        }
    }
}
