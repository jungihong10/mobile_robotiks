#ifndef GRIDTRAVERSAL_H
#define GRIDTRAVERSAL_H

/// The GridTraversal class provides an algorithm for traversing a 2D grid
/// using the traversal algorithm by Amanatides & Woo.
/// @see http://www.cse.chalmers.se/edu/year/2010/course/TDA361/grid.pdf
class GridTraversal
{
public:
    /// Initialize the grid traversal with the start (x0, y0) and end point (x1, y1).
    /// Note that the points are expected in grid coordinates!
    GridTraversal(double x0, double y0, double x1, double y1);

    /// Get the indices (i, j) of the current cell.
    void get(int &i, int &j);
    /// Traverses one cell further and passes the indices (i, j) of this cell.
    /// Returns false once the final cell is reached.
    bool next(int &i, int &j);

private:
    void init();

    int mI;
    int mJ;
    int mI1;
    int mJ1;
    int mStepX;
    int mStepY;
    double mTDeltaX;
    double mTDeltaY;
    double mTMaxX;
    double mTMaxY;
    double mX0;
    double mX1;
    double mY0;
    double mY1;
};

#endif // GRIDTRAVERSAL_H

