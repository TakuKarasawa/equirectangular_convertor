#ifndef PROJECTION_MODE_H_
#define PROJECTION_MODE_H_

#include <iostream>

// ====================== Mode ======================
// Equidistance Projection          : 等距離射影
// Stereographic Projection         : 立体射影
// Inverse Stereographic Projection : 立体射影逆変換
// Orthographic Projection          : 正射影
// Inverse Orthographic Projection  : 正射影逆変換
// ==================================================

enum ProjectionMode
{
    Equidistance,
    Stereographic,
    InverseStereographic,
    Orthographic,
    InverseOrthographic
};

#endif  // PROJECTION_MODE_H_
