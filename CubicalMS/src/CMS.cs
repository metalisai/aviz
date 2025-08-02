using System;
using AnimLib;

public class CMS {

    // converts normalized face-space coordinates to cube-space coordinates
    public static Vector3 FacePosToCubePos(int face, Vector2 pos) {
        return face switch {
            0 => new Vector3(pos.x, pos.y, 1.0f),
            1 => new Vector3(1.0f, pos.y, pos.x),
            2 => new Vector3(pos.x, 1.0f, pos.y),
            3 => new Vector3(0.0f, pos.y, pos.x),
            4 => new Vector3(pos.x, 0.0f, pos.y),
            5 => new Vector3(pos.x, pos.y, 0.0f),
            _ => throw new Exception("Invalid face")
        };
    }

    // projects a world-space 3D direction vector onto a 2D face
    public static Vector2 ProjectOnFace(int face, Vector3 v) {
        // 0: x=x, y=y
        // 1: x=z, y=y
        // 2: x=x, y=-z
        // 3: x=z, y=y
        // 4: x=x, y=z
        // 5: x=-x, y=y
        return face switch {
            0 => new Vector2(v.x, v.y).Normalized,
            1 => new Vector2(v.z, v.y).Normalized,
            2 => new Vector2(v.x, v.z).Normalized,
            3 => new Vector2(v.z, v.y).Normalized,
            4 => new Vector2(v.x, v.z).Normalized,
            5 => new Vector2(v.x, v.y).Normalized,
            _ => throw new Exception("Invalid face")
        };
    }

    public static readonly (int x, int y, int z)[] cornerOffsetsI = new (int, int, int)[] {
        (0, 0, 0),
        (1, 0, 0),
        (1, 1, 0),
        (0, 1, 0),
        (0, 0, 1),
        (1, 0, 1),
        (1, 1, 1),
        (0, 1, 1),
    };

    public static readonly int[,] faceCubeCornersXX = new int[,] {
        { 4, 5, 6, 7 },
        { 5, 1, 2, 6 },
        { 7, 6, 2, 3 },
        { 4, 0, 3, 7 },
        { 4, 5, 1, 0 },
        { 0, 1, 2, 3 },
    };

    public static readonly Vector3[] faceCenters = new Vector3[] {
        new Vector3(0.5f, 0.5f, 1.0f), // face 0
        new Vector3(1.0f, 0.5f, 0.5f), // face 1
        new Vector3(0.5f, 1.0f, 0.5f), // face 2
        new Vector3(0.0f, 0.5f, 0.5f), // face 3
        new Vector3(0.5f, 0.0f, 0.5f), // face 4
        new Vector3(0.5f, 0.5f, 0.0f), // face 5
    };

    // map face idx to cube corners
    public static readonly int[,] faceCubeCorners = new int[,] {
        { 4, 5, 6, 7 },
        { 1, 5, 6, 2 },
        { 3, 2, 6, 7 },
        { 0, 4, 7, 3 },
        { 0, 1, 5, 4 },
        { 0, 1, 2, 3 },
    };

    // latest
    public static readonly int[,] faceEdgeToCubeEdgeXX = new int[6, 4] {
        { 0, 1, 2, 3},
        { 4, 5, 6, 1},
        { 2, 6, 7, 8},
        { 9,10, 8, 3},
        { 0, 4, 11, 9},
        {11, 5, 7, 10}
    };

    // map face edges to cube edges
    public static readonly int[,] faceEdgeToCubeEdge = new int[6, 4] {
        { 0, 3, 2, 1},
        { 4, 5, 6, 1},
        { 7, 8, 2, 6},
        { 9,10, 8, 3},
        {11, 9, 0, 4},
        {11,10, 7, 5}
    };

    // [cubeEdge, i] -> (face, quadEdge)
    public static readonly (int face, int edge)[,] cubeEdgeToFaceEdge = new (int, int)[12, 2] {
        { (0, 0), (4, 2) },
        { (0, 3), (1, 3) },
        { (0, 2), (2, 2) },
        { (0, 1), (3, 3) },
        { (1, 0), (4, 3) },
        { (1, 1), (5, 3) },
        { (1, 2), (2, 3) },
        { (2, 0), (5, 2) },
        { (2, 1), (3, 2) },
        { (3, 0), (4, 1) },
        { (3, 1), (5, 1) },
        { (4, 0), (5, 0) },
    };

    public static readonly bool[][] edgeFlipTable = new bool[][] {
        new bool[]{ false, false, true, true },
        new bool[]{ true, false, false, true },
        new bool[]{ false, true, true, false },
        new bool[]{ true, false, false, true },
        new bool[]{ false, true, true, false },
        new bool[]{ false, false, true, true },
    };

    public static readonly (int face, int edge)[,] cubeEdgeToFaceEdgeXX = new (int, int)[12, 2] {
        { (0, 0), (4, 0) }, //
        { (0, 1), (1, 3) }, //
        { (0, 2), (2, 0) }, //
        { (0, 3), (3, 3) }, //
        { (1, 0), (4, 1) }, //
        { (1, 1), (5, 1) }, //
        { (1, 2), (2, 1) }, //
        { (2, 2), (5, 2) }, //
        { (2, 3), (3, 2) }, //
        { (3, 0), (4, 3) }, //
        { (3, 1), (5, 3) }, //
        { (4, 2), (5, 0) }, //
    };

    public static Vector2 CubePosToFacePosXX(int face, Vector3 pos, float size) {
        switch (face) {
            case 0:
            case 5:
                return new Vector2(pos.x, pos.y);
            case 1:
            case 3:
                return new Vector2(size - pos.z, pos.y);
            case 2:
            case 4:
                return new Vector2(pos.x, size - pos.z);
            default: throw new ArgumentException("Invalid face");
        }
    }

    public static Vector2 CubeDirToFaceDirXX(int face, Vector3 dir) {
        switch (face) {
            case 0:
            case 5:
                return new Vector2(dir.x, dir.y).Normalized;
            case 1:
            case 3:
                return new Vector2(-dir.z, dir.y).Normalized;
            case 2:
            case 4:
                return new Vector2(dir.x, -dir.z).Normalized;
            default: throw new ArgumentException("Invalid face");
        }
    }

    public static Vector3 FacePosToCubePosXX(int face, Vector2 pos, float size) {
        switch(face) {
            case 0:
                return new Vector3(pos.x, pos.y, size);
            case 5:
                return new Vector3(pos.x, pos.y, 0);
            case 1:
                return new Vector3(size, pos.y, size - pos.x);
            case 3:
                return new Vector3(0, pos.y, size - pos.x);
            case 2:
                return new Vector3(pos.x, size, size - pos.y);
            case 4:
                return new Vector3(pos.x, 0, size - pos.y);
            default: throw new ArgumentException("Invalid face");
        }
    }

    public static readonly int[] cubeEdgeDir = new int[12] {
        0, 1, 0, 1, 2, 1, 2, 0, 2, 2, 1, 0
    };

    // NOTE: these are always towards positive direction
    public static readonly (int s, int e)[] cubeEdgeCorners = new (int, int)[12] {
        ( 4, 5), // x
        ( 5, 6), // y
        ( 7, 6), // x
        ( 4, 7), // y
        ( 1, 5), // z
        ( 1, 2), // y
        ( 2, 6), // z
        ( 3, 2), // x 
        ( 3, 7), // z
        ( 0, 4), // z
        ( 0, 3), // y
        ( 0, 1), // x
    };

    public static Vector3[] cornerOffsets = new Vector3[] {
        new Vector3(0, 0, 0),
        new Vector3(1, 0, 0),
        new Vector3(1, 1, 0),
        new Vector3(0, 1, 0),
        new Vector3(0, 0, 1),
        new Vector3(1, 0, 1),
        new Vector3(1, 1, 1),
        new Vector3(0, 1, 1),
    };

    // unwrapped quad offsets
    public static readonly Vector3[] quadOffsets = new Vector3[] {
        new Vector3(0, 0, 0),
        new Vector3(1, 0, 0),
        new Vector3(0, 1, 0),
        new Vector3(-1, 0, 0),
        new Vector3(0, -1, 0),
        new Vector3(2, 0, 0),
    };

    // corners relative to lower left corner
    public static readonly Vector2[] quadCorners = new Vector2[] {
        new Vector2(0, 0),
        new Vector2(1, 0),
        new Vector2(1, 1),
        new Vector2(0, 1),
    };

    public static readonly (int, int)[] quadEdgeCorners = new (int, int)[] {
        (0, 1),
        (0, 3),
        (3, 2),
        (1, 2),
    };

    // NOTE: these are set up such that first is cw and second ccw (when rotating 90 degrees)
    public static readonly int [,] quadSegmentsXX = new int[16, 4] {
        { -1, -1, -1, -1},
        {  3,  0, -1, -1},
        {  0,  1, -1, -1},
        {  3,  1, -1, -1},
        {  1,  2, -1, -1},
        {  0,  3,  2,  1}, // TODO
        {  0,  2, -1, -1},
        {  3,  2, -1, -1},
        {  2,  3, -1, -1},
        {  2,  0, -1, -1},
        {  1,  0,  3,  2}, // TODO
        {  2,  1, -1, -1},
        {  1,  3, -1, -1},
        {  1,  0, -1, -1},
        {  0,  3, -1, -1},
        { -1, -1, -1, -1},
    };

    // segment connections, indices are to quadEdges
    // lhs is always the internal side
    public static readonly int [,] quadSegments = new int[16, 4] {
        { -1, -1, -1, -1},
        {  0,  1, -1, -1},
        {  3,  0, -1, -1},
        {  3,  1, -1, -1},
        {  2,  3, -1, -1},
        {  0,  3,  2,  1},
        {  2,  0, -1, -1},
        {  2,  1, -1, -1},
        {  1,  2, -1, -1},
        {  0,  2, -1, -1},
        {  1,  0,  3,  2},
        {  3,  2, -1, -1},
        {  1,  3, -1, -1},
        {  0,  3, -1, -1},
        {  1,  0, -1, -1},
        { -1, -1, -1, -1},
    };

    // lhs is always the internal side
    public static readonly int[,] alternativeSegments = new int[2, 4] {
        { 0, 1, 2, 3}, // case 5
        { 3, 0, 1, 2}, // case 10
    };

    // the 12 internal faces of an internal octree cell
    // each face is shared by 2 cells
    public static readonly (int cell, int face)[,] internalFaceCells = new (int, int)[12, 2] {
        { (0, 0), (4, 5) }, // z
        { (1, 0), (5, 5) }, // z
        { (2, 0), (6, 5) }, // z
        { (3, 0), (7, 5) }, // z
        { (4, 1), (5, 3) }, // x
        { (0, 1), (1, 3) }, // x
        { (3, 1), (2, 3) }, // x
        { (7, 1), (6, 3) }, // x
        { (4, 2), (7, 4) }, // y 
        { (5, 2), (6, 4) }, // y
        { (1, 2), (2, 4) }, // y
        { (0, 2), (3, 4) }, // y
    };

    // each of the 4 cells share a common edge
    // this maps the face edges to the 2 cube edges that share it
    // [face, quadEdge] => (cellChildIdx, cellEdgeIdx)
    public static readonly (int cell, int edge)[,,] faceEdges = new (int, int)[,,] {
        {{ (4, 1), (5, 3) }, { (5, 2), (6, 0) }, { (6, 3), (7, 1) }, { (7, 0), (4, 2)}},
        {{ (5, 5), (1, 1) }, { (1, 6), (2, 4) }, { (2, 1), (6, 5) }, { (6, 4), (5, 6)}},
        {{ (7, 6), (6, 8) }, { (6, 7), (2, 2) }, { (2, 8), (3, 6) }, { (3, 2), (7, 7)}},
        {{ (4,10), (0, 3) }, { (0, 8), (3, 9) }, { (3, 3), (7,10) }, { (7, 9), (4, 8) }},
        {{ (4, 4), (5, 9) }, { (5,11), (1, 0) }, { (1, 9), (0, 4) }, { (0, 0), (4, 11)}},
        {{ (0, 5), (1,10) }, { (1, 7), (2,11) }, { (2,10), (3, 5) }, { (3,11), (0, 7)}},
    };

    // quadtree internal edge traversal lookup table
    // for each face map to 4 cells along with the internal edge
    public static readonly (int cell, int edge)[,] faceCells = new (int,int)[6,4] {
        { (4, 6), (5,  8), (6,  9), (7,  4)},
        { (5, 7), (1,  2), (2,  0), (6, 11)},
        { (7, 5), (6, 10), (2,  3), (3,  1)},
        { (4, 7), (0,  2), (3,  0), (7, 11)},
        { (4, 5), (5, 10), (1,  3), (0,  1)},
        { (0, 6), (1,  8), (2,  9), (3,  4)},
    };

    public  static readonly int[,] edgeCells = new int[12,2] {
        { 4, 5 },
        { 5, 6 },
        { 7, 6 },
        { 4, 7 },
        { 1, 5 },
        { 1, 2 },
        { 2, 6 },
        { 3, 2 },
        { 3, 7 },
        { 0, 4 },
        { 0, 3 },
        { 0, 1 },
    };
}
