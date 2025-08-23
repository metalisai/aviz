using System;
using System.Linq;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using MathNet.Numerics.LinearAlgebra;

using V3 = System.Numerics.Vector3;
using V2 = System.Numerics.Vector2;
using Debug = AnimLib.Debug;
using AnimLib;

namespace FasterMarchingSquares;

// Cubical marching squares algorithm

//   y
// | 
// | / z
// |/    x
// ------ 

// cube corners and children placement
//  6    7
//   ----   
//  /---/|
// 2|  3|/ 5   (4 not visible)
//  -----      note that the corners are just bit patterns of 0..8
//  0   1         with bit 0 being x, bit 1 y and bit 2 z

// Faces
// 0 - x==0 face
// 1 - x==1 face   face direction = faceId>>1
// 2 - y==0 face      0 - x
// 3 - y==1 face      1 - y
// 4 - z==0 face      2 - z
// 5 - z==1 face

// Face corners and edges
//  2 _____ 3
//   |  2  |       just like cube corners, the corner index
//   |1 0 3|         has bit pattern of coordinates
//  0------- 1      the edges are indexed from bottom clockwise
///   axis1        faces 0,1 - yz     faces 2,3 - zx     faces 4,5 - xy


// Cube edges
//   Cube edges use same indexing order as face corners
//     Thus the 12 edges are grouped into 3 groups of 4. 4 for each direction.
//     (edgeIdx>>2) is edge direction (see direction indexing in Faces)
//   All cube edges always point in positive direction
//     this makes edge subdivision easy and consistent.

// Face children
//
// y
// ---------
// | 2 | 3 |
// |-------|    Same indexing as face corners
// | 0 | 1 |
// --------- x

// Coordinate systems
//   The octree doesn't know anything about global coordinates.
//     Integer coordinates are used to track cell coorinates within octree.
//     These coordinates are indices into hermite data.
//     Edge interections are indexed by dir,dim1,dim2,dim3
//     dim3 always spans the direction of "dir" for sequential memory layout.
//     Cell corners are indexed by x,y,z integers.
//     When new vertices are created, they are created with normalized coordinates within their cell.
//     Global floating point coordinates are resolved after CMS.
//
// The algorithm could've been implemented with less lookup tables.
//    The alternative would be to just duplicate faces and keep track of
//    shared faces. I think the other option would be better now, but initially
//    I started with having edges as reference types as well.

public static class Lookups {
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static I3 CornerOffsetI(int corner, int size) {
        return new I3((corner&1)!=0?size:0, (corner&2)!=0?size:0, (corner&4)!=0?size:0);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static (I3 s, I3 e) EdgeOffset(int edge, int size) {
        int axis = edge >> 2;
        int idx = edge & 0x3;
        int start = axis switch {
            0 => ((idx<<1)&2) | ((idx<<1)&4), // 0, b0, b1
            1 => ((idx>>1)&1) | ((idx<<2)&4), // b1, 0, b0
            2 or _ => idx, // b0, b1, 0
        };
        int end = start | (0x1 << axis);
        return (CornerOffsetI(start, size), CornerOffsetI(end, size));
    }

    public static readonly byte[,] faceEdgeToCubeEdge = new byte[,] {
        { 4,  8, 5, 10 },
        { 6,  9, 7, 11 },
        { 8,  0, 9,  2 },
        {10,  1,11,  3 },
        { 0,  4, 1,  6 },
        { 2,  5, 3,  7 },
    };

    public static readonly byte[,] faceCubeCorners = new byte[,] {
        { 0, 2, 4, 6 },
        { 1, 3, 5, 7 },
        { 0, 4, 1, 5 },
        { 2, 6, 3, 7 },
        { 0, 1, 2, 3 },
        { 4, 5, 6, 7 },
    };

    public static readonly int [,] quadSegments = new int[18, 4] {
        { -1, -1, -1, -1},
        {  1,  0, -1, -1},
        {  0,  3, -1, -1},
        {  1,  3, -1, -1},
        {  2,  1, -1, -1},
        {  2,  0, -1, -1},
        {  0,  3,  2,  1}, // 6a
        {  2,  3, -1, -1},
        {  3,  2, -1, -1},
        {  1,  0,  3,  2}, // 9a
        {  0,  2, -1, -1},
        {  1,  2, -1, -1},
        {  3,  1, -1, -1},
        {  3,  0, -1, -1},
        {  0,  1, -1, -1},
        { -1, -1, -1, -1},
        {  0,  1,  2,  3}, // 6b
        {  1,  2,  3,  0}, // 9b
    };

    public static readonly byte[,] childOuterFaceMap = new byte[,] {
        {0b000, 0}, {0b010, 1}, {0b100, 2}, {0b110, 3},
        {0b001, 0}, {0b011, 1}, {0b101, 2}, {0b111, 3},
        {0b000, 0}, {0b001, 2}, {0b100, 1}, {0b101, 3},
        {0b010, 0}, {0b011, 2}, {0b110, 1}, {0b111, 3},
        {0b000, 0}, {0b001, 1}, {0b010, 2}, {0b011, 3},
        {0b100, 0}, {0b101, 1}, {0b110, 2}, {0b111, 3}
    };

    public static readonly byte[,] childInnerFaceMap = new byte[,] {
        {0, 1}, {1, 0}, {2, 1}, {3, 0},
        {4, 1}, {5, 0}, {6, 1}, {7, 0},
        {0, 3}, {2, 2}, {4, 3}, {6, 2},
        {1, 3}, {3, 2}, {5, 3}, {7, 2},
        {0, 5}, {4, 4}, {1, 5}, {5, 4},
        {2, 5}, {6, 4}, {3, 5}, {7, 4},
    };

    public static readonly byte[,] faceInnerEdgeMap = new byte[,] {
        {2, 2}, {3, 4}, {2, 3}, {1, 4},
        {0, 2}, {3, 5}, {0, 3}, {1, 5},
        {2, 4}, {3, 0}, {2, 5}, {1, 0},
        {0, 4}, {3, 1}, {0, 5}, {1, 1},
        {2, 0}, {3, 2}, {2, 1}, {1, 2},
        {0, 0}, {3, 3}, {0, 1}, {1, 3},
    };

    public static readonly byte[,] faceOuterEdgeMap = new byte[,] {
        {0, 4, 2}, {1, 2, 0}, {0, 4, 3}, {3, 3, 0},
        {1, 2, 1}, {2, 5, 2}, {2, 5, 3}, {3, 3, 1},
        {0, 0, 2}, {1, 4, 0}, {0, 0, 3}, {3, 5, 0},
        {1, 4, 1}, {2, 1, 2}, {2, 1, 3}, {3, 5, 1},
        {0, 2, 2}, {1, 0, 0}, {0, 2, 3}, {3, 1, 0},
        {1, 0, 1}, {2, 3, 2}, {2, 3, 3}, {3, 1, 1},
    };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2? CubeDirToFaceDir(int face, V3 dir) {
        V2 ret;
        switch (face>>1) {
            case 0:
                ret = V2.Normalize(new V2(dir.Y, dir.Z));
                break;
            case 1:
                ret = V2.Normalize(new V2(dir.Z, dir.X));
                break;
            case 2:
            default:
                ret = V2.Normalize(new V2(dir.X, dir.Y));
                break;
        }
        return ret.LengthSquared() < 1e-6 ? null : ret;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 CubePosToFacePos(int face, V3 pos) {
        switch (face>>1) {
            case 0:
                return new V2(pos.Y, pos.Z);
            case 1:
                return new V2(pos.Z, pos.X);
            case 2:
            default:
                return new V2(pos.X, pos.Y);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 FacePosToCubePos(int face, V2 pos) {
        switch(face) {
            case 0:
                return new V3(0.0f, pos.X, pos.Y);
            case 1:
                return new V3(1.0f, pos.X, pos.Y);
            case 2:
                return new V3(pos.Y, 0.0f, pos.X);
            case 3:
                return new V3(pos.Y, 1.0f, pos.X);
            case 4:
                return new V3(pos.X, pos.Y, 0.0f);
            case 5:
            default:
                return new V3(pos.X, pos.Y, 1.0f);
        }
    }
}

public abstract class CMSVertex {
    public abstract override bool Equals(object obj);
    public abstract override int GetHashCode();
}

public class CMSEdgeVertex : CMSVertex {
    public EdgeCoord edge { get; set; }
    public CMSEdgeVertex(EdgeCoord edge) {
        this.edge = edge;
    }

    public override bool Equals(object obj) => obj is CMSEdgeVertex ev ? this.edge == ev.edge : false;
    public override int GetHashCode() => edge.GetHashCode();
}

public class CMSNewVertex : CMSVertex {
    public V3 position;
    public I3 cellMin;
    public int cellSize;

    public CMSNewVertex(V3 position, I3 cellMin, int cellSize)
    {
        this.position = position;
        this.cellMin = cellMin;
        this.cellSize = cellSize;
    }

    public override bool Equals(object obj) => ReferenceEquals(this, obj);
    public override int GetHashCode() => System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(this);
}

public record struct CMSSegment(CMSVertex v1, CMSVertex v2);

public struct EdgeCoord {
    // all this packing seems crazy, but the whole algorithms is ~10% faster
    //   with this packing
    public ulong xyz;

    public int x {
        get => (int)(xyz & 0xFFFF);
        set {
            Debug.Assert(value <= 0xFFFF);
            xyz = (xyz&0xFFFFFFFFFFFF0000UL) | ((ulong)(uint)value);
        }
    }

    public int y {
        get => (int)((xyz>>16) & 0xFFFF);
        set {
            Debug.Assert(value <= 0xFFFF);
            xyz = (xyz&0xFFFFFFFF0000FFFF) | (((ulong)(uint)value) << 16);
        }
    }

    public int z {
        get => (int)((xyz>>32) & 0xFFFF);
        set {
            Debug.Assert(value <= 0xFFFF);
            xyz = (xyz&0xFFFF0000FFFFFFFF) | (((ulong)(uint)value) << 32);
        }
    }

    // this puts hermite grid size limit at 16k
    // 16K^3 grid would take 4TB of memory, so probably ok?
    public int count {
        get => (int)((xyz>>48) & 0x3FFF);
        set {
            Debug.Assert(value <= 0x3FFF);
            xyz = (xyz&0xC000FFFFFFFFFFFF) | (((ulong)(uint)value) << 48);
        }
    }

    public int dir {
        get => (int)((xyz>>62) & 0x3);
        set {
            xyz = (xyz&0x3FFFFFFFFFFFFFFF) | (((ulong)(uint)value) << 62);
        }
    }

    public EdgeCoord() {
    }

    public EdgeCoord(I3 loc, int count, int dir) {
        this.x = loc.x;
        this.y = loc.y;
        this.z = loc.z;
        this.count = count;
        this.dir = (byte)dir;
    }

    public (EdgeCoord, EdgeCoord) Subdivide() {
        Debug.Assert(count > 0 && (count&1) == 0);
        var hc = count>>1;
        var first = new EdgeCoord(){ x = x, y = y, z = z, count = hc, dir=dir };
        var second = dir switch {
            0 => new EdgeCoord(){ x = x + hc, y = y, z = z, count = hc, dir=0 },
            1 => new EdgeCoord(){ x = x, y = y + hc, z = z, count = hc, dir=1 },
            2 or _ => new EdgeCoord(){ x = x, y = y, z = z + hc, count = hc, dir=2 },
        };
        return (first, second);
    }

    public static bool operator==(EdgeCoord a, EdgeCoord b) => a.xyz == b.xyz;
    public static bool operator!=(EdgeCoord a, EdgeCoord b) => a.xyz != b.xyz; 
    public override bool Equals(object obj) => obj is EdgeCoord other && Equals(other);
    public bool Equals(EdgeCoord other) => xyz == other.xyz;
    public override int GetHashCode() => HashCode.Combine(xyz);
}

[InlineArray(4)]
public struct FaceEdges {
    public EdgeCoord element0;
}

public struct Face {
    public int firstChild;
    public bool IsLeaf => firstChild == -1;
    public Face() {
        firstChild = -1;
    }
}

public class CMSTree {
    public const float SHARP_FEATURE_ANGLE_THRESHOLD = 0.80f;

    public int size;
    public int rootCell;
    HermiteData data;

    // cell arrays of structures
    public List<Cell> cellPool = new();
    public List<byte> cellIsInsideBits = new();
    public List<CellFaces> cellFacePool = new();

    public List<Face> facePool = new();
    public List<FaceEdges> faceEdgePool = new();

    int newCell() {
        int ret = cellPool.Count;
        cellPool.Add(new Cell());
        cellFacePool.Add(new CellFaces());
        cellIsInsideBits.Add(0);
        return ret;
    }

    int newFace() {
        int ret = facePool.Count;
        facePool.Add(new Face());
        faceEdgePool.Add(new FaceEdges());
        return ret;
    }
    
    public CMSTree(HermiteData data) {
        this.size = data.size;
        this.rootCell = newCell(); 
        InitRoot(rootCell);
        this.data = data;
    }

    private void InitRoot(int rootId) {
        ref Cell cell = ref CollectionsMarshal.AsSpan(cellPool)[rootId];
        ref CellFaces cellFaces = ref CollectionsMarshal.AsSpan(cellFacePool)[rootId];
        // initialize edges
        var edges = new EdgeCoord[12];
        for (int i = 0; i < 12; i++) {
            var dir = i >> 2;
            var (es, _) = Lookups.EdgeOffset(i, size);
            edges[i] = new EdgeCoord(es, size, dir);
        }
        // initialize faces
        for (int i = 0; i < 6; i++) {
            var faceId = newFace();
            ref FaceEdges facee = ref CollectionsMarshal.AsSpan(faceEdgePool)[faceId];
            for (int j = 0; j < 4; j++) {
                facee[j] = edges[Lookups.faceEdgeToCubeEdge[i, j]];
            }
            cellFaces[i] = faceId;
        }
    }

    struct SubdivideCtx {
        public HermiteData data;
        public Dictionary<int, (int faceIdx, int cellId, I3 min, int size)> faceSet;
        public List<(int, I3, int)> leafCells;
        public int maxDepth;
    }

    private void AdaptiveSubdivide(out Dictionary<int, (int faceIdx, int cellId, I3 min, int size)> faceSet, out List<(int, I3, int)> leafCells) {
        int maxDepth = (int)Math.Round(Math.Log2(data.size));
        var sw = new System.Diagnostics.Stopwatch();
        sw.Start();
        // leaves before subdivision
        List<(int c, I3 m, int s, int d)> leaves = new();
        // final leaf cells
        leafCells = new();
        // leaf faces
        faceSet = new();

        SubdivideCtx ctx = new() {
            data = data,
            faceSet = faceSet,
            leafCells = leafCells,
            maxDepth = maxDepth
        };

        void traverse(int cellId, I3 min, int size, int depth) {
            if (!cellPool[cellId].IsLeaf) {
                for (int i = 0; i < 8; i++) {
                    var child = cellPool[cellId].firstChild + i;
                    var co = Lookups.CornerOffsetI(i, size>>1);
                    var childMin = min + co;
                    traverse(child, childMin, size>>1, depth+1);
                }
            } else {
                leaves.Add((cellId, min, size, depth));
            }
        }
        traverse(rootCell, new I3(), size, 0);
        foreach(var (leaf, leafMin, leafSize, depth) in leaves) {
            AdaptiveSubdivide(leaf, leafMin, leafSize, depth, in ctx);
        }
        System.Console.WriteLine($"AdaptiveSubdivide took {sw.Elapsed.TotalMilliseconds} ms");
    } 

    void AdaptiveSubdivide(int cellId, I3 cellMin, int cellSize, int depth, in SubdivideCtx ctx) {
        bool foundAny = false;
        List<V3> allIntr = new ();

        for (int i = 0; i < 12; i++) {
            var (es, _) = Lookups.EdgeOffset(i, cellSize);
            var ecoord = new EdgeCoord(cellMin + es, cellSize, i>>2);
            var intrs = data.FindEdgeIntersections(ecoord);
            foreach(var intr in intrs) {
                allIntr.Add(intr.n);
            }
            if (intrs.Length > 1) {
                foundAny = true;
            }
        }
        // assign corners
        byte corners = 0;
        for (int i = 0; i < 8; i++) {
            var offset = Lookups.CornerOffsetI(i, cellSize);
            var cornerOffset = cellMin + offset;
            bool val = data.isInside[cornerOffset.x, cornerOffset.y, cornerOffset.z];
            corners |= (byte)(val ? 1 << i : 0);
        }
        cellIsInsideBits[cellId] = corners;

        bool complx = LikelyToContainComplexSurface(allIntr);
        if ((complx && depth < ctx.maxDepth) || (foundAny && depth < ctx.maxDepth)) {
            SubdivideCell(cellId, cellMin, cellSize);
            if (!cellPool[cellId].IsLeaf) {
                for (int i = 0; i < 8; i++) {
                    var child = cellPool[cellId].firstChild + i;
                    var co = Lookups.CornerOffsetI(i, cellSize>>1);
                    var childMin = cellMin + co;
                    AdaptiveSubdivide(child, childMin, cellSize>>1, depth+1, in ctx);
                }
            }
        }
        else
        {
            ctx.leafCells.Add((cellId, cellMin, cellSize));
        }
        for (int i = 0; i < 6; i++) {
            var faceId = cellFacePool[cellId][i];
            if (facePool[faceId].IsLeaf) {
                if (!ctx.faceSet.ContainsKey(faceId)) {
                    ctx.faceSet.Add(faceId, (i, cellId, cellMin, cellSize));
                }
            }
        }
    }

    static bool LikelyToContainComplexSurface(List<V3> inormals) {
        if (inormals.Count < 2) return false;
        float max = 0.0f;
        for (int i = 0; i < inormals.Count; i++) {
            for (int j = i + 1; j < inormals.Count; j++) {
                var cosAngle = V3.Dot(inormals[i], inormals[j]);
                var dist = 1.0f - cosAngle;
                max = MathF.Max(max, dist);
            }
        }
        // closer to a 0 means more detail
        return max > 0.2f;
    }

    public void Subdivide(int count) {
        void recurse(int cellId, I3 cellMin, int size, int left) {
            if (left == 0) return;
            SubdivideCell(cellId, cellMin, size);
            for (int i = 0; i < 8; i++) {
                var child = cellPool[cellId].firstChild + i;
                var co = Lookups.CornerOffsetI(i, size>>1);
                I3 childMin = cellMin + co;
                recurse(child, childMin, size>>1, left-1);
            }
        }
        recurse(rootCell, new I3(), size, count);
    }

    public List<CMSSegment>[] EvaluateFaces(Dictionary<int, (int faceIdx, int cellId, I3 min, int size)> faceSet) {
        var sw = new System.Diagnostics.Stopwatch();
        sw.Start();

        //List<
        List<(int, (int faceIdx, int cellId, I3 min, int size))> faces = new();
        faces.EnsureCapacity(faceSet.Count);
        foreach (var kvp in faceSet) {
            if (facePool[kvp.Key].IsLeaf) {
                faces.Add((kvp.Key, kvp.Value));
            }
        }

        // Find all unique leaf faces
        // NOTE: leaf nodes in octree are at least 7/8 percent of total nodes, so it's always more efficient to use array instead of hashmap, if ids are dense
		List<CMSSegment>[] faceSegments = new List<CMSSegment>[facePool.Count];

        
        /*
        // parallel implementation. Only like 50% faster with 6 cores so not worth it
        //   (most time is spent in ExtractSurface anyways)
        System.Collections.Concurrent.ConcurrentBag<List<(int, List<CMSSegment>)>> bag = new();
        var partitioner = System.Collections.Concurrent.Partitioner.Create(0, faces.Count, 1024);
        var options = new System.Threading.Tasks.ParallelOptions();
        options.TaskScheduler = System.Threading.Tasks.TaskScheduler.Default;
        options.MaxDegreeOfParallelism = 6;
        System.Threading.Tasks.Parallel.ForEach(partitioner, options, x => {
            List<(int, List<CMSSegment>)> els = new();
            for (int i = x.Item1; i < x.Item2; i++) {
                List<CMSSegment> segs = new();
                int faceId = faces[i].Item1;
                var (faceIdx, cellId, nodeMin, nodeSize) = faces[i].Item2;
                EvaluateCellFace(cellId, faceEdgePool[faceId], faceIdx, segs, cellIsInsideBits[cellId], nodeMin, nodeSize);
                els.Add((faceId, segs));
            }
            bag.Add(els);
        });
        foreach(var llist in bag) {
            foreach(var (faceId, segs) in llist) {
                faceSegments[faceId] = segs;
            }
        }*/

        foreach (var (faceId, (faceIdx, cellId, nodeMin, nodeSize)) in faces) {
            List<CMSSegment> segs = new();
            EvaluateCellFace(cellId, faceEdgePool[faceId], faceIdx, segs, cellIsInsideBits[cellId], nodeMin, nodeSize);
            faceSegments[faceId] = segs;
        }

        System.Console.WriteLine($"EvaluateFaces took {sw.Elapsed.TotalMilliseconds} ms");

		return faceSegments;
    }

    struct CellCtx {
        public Cell cell;
        public I3 nodeMin;
        public int nodeSize;
        public List<CMSSegment> cellSegments;
        public HermiteData data;

        public CellCtx(Cell cell, I3 nodeMin, int nodeSize, List<CMSSegment> segs, HermiteData data) {
            this.cell = cell;
            this.nodeMin = nodeMin;
            this.nodeSize = nodeSize;
            this.data = data;
            this.cellSegments = segs;
        }
    }

    struct LocalVertex {
        public CMSVertex vertex;
        public V3 cellCoord;
        public V3? normal;
    }

    struct CMSComponent {
        public List<LocalVertex> vertices;
        public List<(int, int)> segments;
        public V3 centralVertex = default;
        public bool isSharp = default;

        public CMSComponent() {
            vertices = new();
            segments = new();
        }
    }

    static void ResolveCellVertex(in CellCtx ctx, ref LocalVertex lv, CMSVertex cv) {
        if (cv is CMSEdgeVertex mev) {
            var intr = ctx.data.GetIntersectionNormalized(mev.edge, ctx.nodeMin, ctx.nodeSize);
            lv.normal = intr.Value.n;
            lv.cellCoord = intr.Value.p;
        } else  if(cv is CMSNewVertex mnv) {
            if (mnv.cellSize != ctx.nodeSize) {
                var dif = mnv.cellMin - ctx.nodeMin;
                lv.cellCoord = (mnv.position*mnv.cellSize + dif.ToV3()) / ctx.nodeSize;
            } else {
                lv.cellCoord = mnv.position;
            }
        }
    }

    static void TriangulateFan(in CellCtx ctx, in CMSComponent comp, List<CMSVertex> outV, List<int> outI) {
        int start = outV.Count;
        var vspan = CollectionsMarshal.AsSpan(comp.vertices);
        foreach(ref LocalVertex v in vspan) {
            outV.Add(v.vertex);
        }
        int centerIdx = outV.Count;
        outV.Add(new CMSNewVertex(comp.centralVertex, ctx.nodeMin, ctx.nodeSize));

        foreach(var (s, e) in comp.segments) {
            outI.Add(start + s);
            outI.Add(start + e);
            outI.Add(centerIdx);
        }
    }

    static bool CheckIntersection3D(in CMSComponent comp1, in CMSComponent comp2) {
        //throw new NotImplementedException();
        return true;
    }

    // square of longest edge
    // NOTE: this does not guarantee valid triangulation,
    //   but it seems to work well in practice
    static float triangleCost(V3 v1, V3 v2, V3 v3) {
        var e1 = v2 - v1;
        var e2 = v3 - v2;
        var e3 = v1 - v3;
        //var ret =  float.Max(V3.Dot(e1, e1), V3.Dot(e2, e2));
        //return float.Max(ret, V3.Dot(e3, e3));
        return V3.Dot(e1, e1) + V3.Dot(e2, e2) + V3.Dot(e3, e3);
    }

    static void TriangulateConnected(in CMSComponent comp1, in CMSComponent comp2, List<CMSVertex> outV, List<int> outI) {
        // take all vertices except the central one
        var vs1 = comp1.vertices;
        var vs2 = comp2.vertices;
        var verts1 = comp1.vertices.Select(x => x.cellCoord).ToArray();
        // NOTE: components will always be oriented opposite
        var verts2 = comp2.vertices.Select(x => x.cellCoord).Reverse().ToArray();
        var center1 = comp1.centralVertex;
        var center2 = comp2.centralVertex;
        var dir = V3.Normalize(center2 - center1);

        List<bool> bestPath = new();
        float bestCost = float.MaxValue;
        int bestOffset = 0;
        // try all offsets
        for(int l2Ofst = 0; l2Ofst < verts2.Length; l2Ofst++) {
            int[,] bestOptions = new int[verts1.Length+1, verts2.Length+1];
            float?[,] memo = new float?[verts1.Length + 1, verts2.Length + 1];
            // DP to find best triangulation
            float next(int li, int lj) {
                if (li >= verts1.Length && lj >= verts2.Length) {
                    return 0.0f;
                }
                if (memo[li, lj].HasValue) {
                    return memo[li, lj].Value;
                }

                float twistCost = 0.0f;

                float costA = float.MaxValue;
                float costB = float.MaxValue;
                if (li < verts1.Length) {
                    var v1 = verts1[li%verts1.Length];
                    var v2 = verts1[(li + 1)%verts1.Length];
                    var v3 = verts2[(l2Ofst + lj)%verts2.Length];
                    costA = triangleCost(v1, v2, v3) + next(li+1, lj);
                }
                if (lj < verts2.Length) {
                    var v1 = verts1[(li)%verts1.Length];
                    var v2 = verts2[(l2Ofst + lj)%verts2.Length];
                    var v3 = verts2[(l2Ofst + lj + 1)%verts2.Length];
                    costB = triangleCost(v1, v2, v3) + next(li, lj+1);
                }
                var ret = float.Min(costA, costB);
                bestOptions[li, lj] = ret == costA ? 0 : 1;
                memo[li, lj] = ret + twistCost;
                return ret + twistCost;
            }
            float cost = next(0, 0);
            Console.WriteLine($"cost {cost}");
            if (cost < bestCost) {
                bestPath.Clear();
                int mi = 0, mj = 0;
                while (mi < verts1.Length || mj < verts2.Length) {
                    var opt = bestOptions[mi, mj];
                    bestPath.Add(opt == 0);
                    if (opt == 0) {
                        mi++;
                    } else {
                        mj++;
                    }
                }
                Console.WriteLine($"mi {mi}/{verts1.Length} mj {mj}/{verts2.Length} cost {cost}");
                bestCost = cost;
                bestOffset = l2Ofst;
            }
        }

        outV.Clear();
        outI.Clear();
        int comp1Base = outV.Count;
        foreach(var v in comp1.vertices) {
            outV.Add(v.vertex);
        }
        int comp2Base = outV.Count;
        foreach(var v in comp2.vertices.AsEnumerable().Reverse()) {
            outV.Add(v.vertex);
        }

        int i = 0, j = 0;
        foreach(var option in bestPath) {
            if(option) {
                outI.Add(comp1Base + i);
                outI.Add(comp1Base + ((i+1)%verts1.Length));
                outI.Add(comp2Base + (bestOffset + j)%verts2.Length);
                i += 1;
            } else {
                outI.Add(comp1Base + i);
                outI.Add(comp2Base + (bestOffset + j)%verts2.Length);
                outI.Add(comp2Base + (bestOffset + j + 1)%verts2.Length);
                j += 1;
            }
        }
        Console.WriteLine($"i {i}/{verts1.Length} j {j}/{verts2.Length} count {bestPath.Count}");
        Console.WriteLine($"Triangulated connected components with {outV.Count} vertices and {outI.Count/3} triangles");
    }

    static void processComponent(int[] loop, in CellCtx ctx, ref CMSComponent outComp)
    {
        outComp.vertices.Clear();
        outComp.segments.Clear();
        V3 massPoint = V3.Zero;
        int vIdx = 0;
        Dictionary<CMSVertex, int> vertexMap = new();
        float mass = 0.0f;
        foreach (var idx in loop) {
            var seg = ctx.cellSegments[idx];
            // NOTE: do not move this above the .Add() !!!
            int v1Idx;
            if (!vertexMap.TryGetValue(ctx.cellSegments[idx].v1, out v1Idx)) {
                LocalVertex mv1 = default;
                mv1.vertex = ctx.cellSegments[idx].v1;
                ResolveCellVertex(ctx, ref mv1, seg.v1);
                outComp.vertices.Add(mv1);
                massPoint += mv1.cellCoord;
                v1Idx = vIdx++;
                vertexMap[ctx.cellSegments[idx].v1] = v1Idx;
                mass += 1.0f;
            }
            int v2Idx;
            if (!vertexMap.TryGetValue(ctx.cellSegments[idx].v2, out v2Idx)) {
                LocalVertex mv2 = default;
                mv2.vertex = ctx.cellSegments[idx].v2;
                ResolveCellVertex(ctx, ref mv2, seg.v2);
                outComp.vertices.Add(mv2);
                massPoint += mv2.cellCoord;
                v2Idx = vIdx++;
                vertexMap[ctx.cellSegments[idx].v2] = v2Idx;
                mass += 1.0f;
            }
            outComp.segments.Add((v1Idx, v2Idx));
        }
        massPoint *= 1.0f / mass;

        var vcount = outComp.vertices.Count;
        float minCos = 1.0f;
        for (int i = 0; i < vcount; i++) {
            for (int j = i + 1; j < vcount; j++) {
                var n1 = outComp.vertices[i].normal;
                var n2 = outComp.vertices[j].normal;
                if (n1 == null || n2 == null) continue;
                var cos = V3.Dot(n1.Value, n2.Value);
                if (cos < minCos) {
                    minCos = cos;
                }
            }
        }

        if (minCos < 0.9f || true) {
            var withNormalCount = outComp.vertices.Count(v => v.normal != null);
            var matrix = Matrix<float>.Build.Dense(withNormalCount, 3);
            var vector = Vector<float>.Build.Dense(withNormalCount, 1.0f);
            int vi = 0;
            for (int i = 0; i < vcount; i++) {
                if (outComp.vertices[i].normal == null) {
                    continue;
                }
                var n = outComp.vertices[i].normal.Value;
                matrix.SetRow(vi, new float[] { n.X, n.Y, n.Z });
                vector[vi] = V3.Dot(n, outComp.vertices[i].cellCoord - massPoint);
                vi++;
            }
            var svd = matrix.Svd(true);
            var mw = svd.W;
            var ms = svd.S;
            float lambda = 1e-2f;
            for (int i = 0; i < ms.Count; i++)
            {
                // ridge regression style regularization
                //   helps counter numberical instability
                ms[i] = ms[i] / (ms[i]*ms[i] + lambda);
                // truncation style regularization
                //ms[i] = MathF.Abs(ms[i]) > 1e-3 ? 1/ms[i] : 0.0f;
            }
            mw.SetDiagonal(ms);
            var pseudoInverse = (svd.U * mw * svd.VT).Transpose();
            var result = pseudoInverse * vector;
            var p = new V3(result[0], result[1], result[2]) + massPoint;
            if (p.X < -0.5f || p.X > 1.5f ||
                p.Y < -0.5f || p.Y > 1.5f ||
                p.Z < -0.5f || p.Z > 1.5f) {
                outComp.centralVertex = massPoint;
                outComp.isSharp = false;
            } else {
                outComp.centralVertex = p;
                outComp.isSharp = true;
            }
        } else {
            outComp.centralVertex = massPoint;
            outComp.isSharp = false;
        }
    }

    public (V3[] vertices, int[] indices) ExtractSurface() {
        AdaptiveSubdivide(out var faceSet, out var leafCells);

        var sw = new System.Diagnostics.Stopwatch();
        sw.Start();

        // segments of all leaf faces
        List<CMSSegment>[] faceSegments = EvaluateFaces(faceSet);
        // storage for components (max 4 per cell)
        CMSComponent[] comps = new CMSComponent[4];
        for (int i = 0; i < comps.Length; i++) {
            comps[i] = new CMSComponent();
        }
        List<CMSSegment> cs = new();

        // output vertices and indices
        List<CMSVertex> cmsVertices = new();
        List<int> indices = new();

        foreach (var (cellId, cellMin, cellSize) in leafCells) {
            cs.Clear();
            ref Cell cell = ref CollectionsMarshal.AsSpan(cellPool)[cellId];
            CellCtx cellCtx = new(cell, cellMin, cellSize, cs, data);

            GetCellFaceSegments(cellId, cs, faceSegments);
            var cornerBits = cellIsInsideBits[cellId]; 

            // find and process all components in the cell
            var loops = GetLoops(cs);
            for (int i = 0; i < loops.Length; i++) {
                var loop = loops[i];
                processComponent(loop, in cellCtx, ref comps[i]);
            }

            // check if diagonal case
            //   0 - wont connect components
            //   1 - diagonal case
            //   2 - inverted diagonal case
            int cellCase = 0;
            for (int i = 0; i < 4; i++) {
                byte diag1 = (byte)((0x1 << i) | (0x80 >> i));
                byte diag0 = (byte)~diag1;
                if (cornerBits == diag1) {
                    // case where 2 diagonal corners are inside
                    cellCase = 1;
                } else if (cornerBits == diag0) {
                    // inverted diagonal case
                    cellCase = 2;
                }
            }

            List<CMSVertex> tvs = new();
            List<int> idxs = new();
            if (cellCase == 0 || !CheckIntersection3D(in comps[0], in comps[1])) {
                // normal triangle fan case
                for (int i = 0; i < loops.Length; i++) {
                    TriangulateFan(in cellCtx, in comps[i], tvs, idxs); 
                }
            } else {
                // intersection found
                //   connect components with 'cylinder'
                Debug.Assert(loops.Length == 2);
                TriangulateConnected(in comps[0], in comps[1], tvs, idxs);
                var line = new Line3D(mode: MeshVertexMode.Segments);
                List<Vector3> vs = new();
                List<Color> css = new();
                foreach(var seg in comps[0].segments) {
                    var v1 = comps[0].vertices[seg.Item1].vertex;
                    var v2 = comps[0].vertices[seg.Item2].vertex;
                    V3 p1 = default, p2 = default;
                    if (v1 is CMSEdgeVertex ev1) {
                        var res = data.GetIntersection(ev1.edge);
                        p1 = res.Value.p;
                    } else if (v1 is CMSNewVertex nv1) {
                        p1 = data.ResolveVertex(nv1);
                    }
                    if (v2 is CMSEdgeVertex ev2) {
                        var res = data.GetIntersection(ev2.edge);
                        p2 = res.Value.p;
                    } else if (v2 is CMSNewVertex nv2) {
                        p2 = data.ResolveVertex(nv2);
                    }
                    vs.Add(p1);
                    vs.Add(p2);
                    css.Add(Color.CYAN);
                    css.Add(Color.CYAN);
                }
                foreach(var seg in comps[1].segments) {
                    var v1 = comps[1].vertices[seg.Item1].vertex;
                    var v2 = comps[1].vertices[seg.Item2].vertex;
                    V3 p1 = default, p2 = default;
                    if (v1 is CMSEdgeVertex ev1) {
                        var res = data.GetIntersection(ev1.edge);
                        p1 = res.Value.p;
                    } else if (v1 is CMSNewVertex nv1) {
                        p1 = data.ResolveVertex(nv1);
                    }
                    if (v2 is CMSEdgeVertex ev2) {
                        var res = data.GetIntersection(ev2.edge);
                        p2 = res.Value.p;
                    } else if (v2 is CMSNewVertex nv2) {
                        p2 = data.ResolveVertex(nv2);
                    }
                    vs.Add(p1);
                    vs.Add(p2);
                    css.Add(Color.MAGENTA);
                    css.Add(Color.MAGENTA);
                }
                line.Vertices = vs.ToArray();
                line.Colors = css.ToArray();
                World.current.CreateInstantly(line);
                /*for (int i = 0; i < loops.Length; i++) {
                    TriangulateFan(in cellCtx, in comps[i], tvs, idxs); 
                }*/
            }
            // add vertices and indices to the output
            if ((idxs?.Count ?? 0) > 0) {
                int startIdx = cmsVertices.Count;
                cmsVertices.AddRange(tvs);
                for (int j = 0; j < idxs.Count; j++) {
                    indices.Add(startIdx + idxs[j]);
                }
            }
        }

        // resolve vertex coordinates - compose final mesh data
        var retVertices = new V3[cmsVertices.Count];
        for (int i = 0; i < retVertices.Length; i++) {
            switch (cmsVertices[i]) {
                case CMSEdgeVertex ev:
                    var res = data.GetIntersection(ev.edge);
                    retVertices[i] = res.Value.p;
                    break;
                case CMSNewVertex nv:
                    retVertices[i] = data.ResolveVertex(nv);
                    break;
            }
        }

        System.Console.WriteLine($"ExtractSurface took {sw.Elapsed.TotalMilliseconds} ms");
        System.Console.WriteLine($"cellPool size: {cellPool.Count} facePool size {facePool.Count}");
        return (retVertices, indices.ToArray());
	}

    public void SubdivideCell(int cellId, I3 min, int size) {
        Debug.Assert(size > 1 && (size&1) == 0);
        // subdivide faces first
        for (int i = 0; i < 6; i++) {
            var faceId = cellFacePool[cellId][i];
            if (facePool[faceId].IsLeaf) {
                SubdivideFace(faceId, min, size, i);
            }
        }

        // create new children
        int firstChild = newCell();
        // NOTE: we always assume all 8 children are allocated continuously
        for (int i = 0; i < 7; i++) {
            int res = newCell();
            Debug.Assert(res == firstChild + i + 1);
        }
        // NOTE: do not move this above newCell() calls!
        ref Cell cell = ref CollectionsMarshal.AsSpan(cellPool)[cellId];
        cell.firstChild = firstChild;

        // create new edges in the middle of the cell and assign them to children
        // assign outer edges created by face subdivision to children
        ConnectChildFaces(cellId, min, size);
    }

    private void ConnectChildFaces(int cellId, I3 min, int size) {
        var cellSpan = CollectionsMarshal.AsSpan(cellPool);
        var cellFaceSpan = CollectionsMarshal.AsSpan(cellFacePool);
        ref Cell cell = ref cellSpan[cellId];
        ref CellFaces cellFaces = ref cellFaceSpan[cellId];
        Debug.Assert(size > 1 && (size&1) == 0);
        int hsize = size >> 1;
        // outer faces 
        for (int i = 0; i < 24; i++) {
            int child = cell.firstChild + Lookups.childOuterFaceMap[i, 0];
            int fidx = Lookups.childOuterFaceMap[i, 1];
            int face = i >> 2;
            cellFaceSpan[child][face] = facePool[cellFaces[face]].firstChild + fidx;
        }

        // create inner faces (without edges assigned)
        var newFaces = new int[12];
        for (int i = 0; i < 12; i++) {
            newFaces[i] = newFace();
        }
        var faceEdgeSpan = CollectionsMarshal.AsSpan(faceEdgePool);

        for (int i = 0; i < 24; i++) {
            var child = Lookups.childInnerFaceMap[i, 0];
            var childFace = Lookups.childInnerFaceMap[i, 1];
            var newFace = i>>1;
            cellFaceSpan[cell.firstChild + child][childFace] = newFaces[newFace];
        }

        // create new edges at the centers of 4 cells at each of the 6 faces
        var cellCenter = min + new I3(hsize, hsize, hsize);
        EdgeCoord[] newEdges = [
            // x
            new EdgeCoord(min + new I3(0, hsize, hsize), hsize, 0),
            new EdgeCoord(cellCenter, hsize, 0),
            // y
            new EdgeCoord(min + new I3(hsize, 0, hsize), hsize, 1),
            new EdgeCoord(cellCenter, hsize, 1),
            // z
            new EdgeCoord(min + new I3(hsize, hsize, 0), hsize, 2),
            new EdgeCoord(cellCenter, hsize, 2),
        ];

        //                            _|/_
        // assign new edges to faces   /|
        for (int i = 0; i < 24; i++) {
            var newFace = i >> 1;
            var edge = Lookups.faceInnerEdgeMap[i, 0];
            var newEdge = Lookups.faceInnerEdgeMap[i, 1];
            faceEdgeSpan[newFaces[newFace]][edge] = newEdges[newEdge];
        }

        EdgeCoord[,] newInternalEdges = new EdgeCoord[6, 4];
        var neSpan = MemoryMarshal.CreateSpan<EdgeCoord>(ref Unsafe.As<byte, EdgeCoord>( ref MemoryMarshal.GetArrayDataReference(newInternalEdges)), newInternalEdges.Length);
        for (int i = 0; i < 6; i++) {
            FaceInternalEdges(i, size, min, neSpan.Slice(i*4));
        }

        for (int i = 0; i < 24; i++) {
            var face = i >> 1;
            var edge = Lookups.faceOuterEdgeMap[i, 0];
            var iface = Lookups.faceOuterEdgeMap[i, 1];
            var iedge = Lookups.faceOuterEdgeMap[i, 2];
            faceEdgeSpan[newFaces[face]][edge] = newInternalEdges[iface, iedge];
        }
    }

    static bool SegmentsIntersect(V2 s1, V2 e1, V2 s2, V2 e2)
    {
        var d1 = e1 - s1;
        var d2 = e2 - s2;
        var d = s1 - s2;
        var denom = d2.Y * d1.X - d2.X * d1.Y;
        if (denom == 0.0f)
        {
            return false;
        }
        var t1 = (d2.X * d.Y - d2.Y * d.X) / denom;
        var t2 = (d1.X * d.Y - d1.Y * d.X) / denom;
        return t1 >= 0.0f && t1 <= 1.0f && t2 >= 0.0f && t2 <= 1.0f;
    }

    struct ResolvedEdgeVertex {
        public EdgeCoord edge;
        public V3 localPos3;
        public V2? localNormal;
        public V2 localPos2;
    }

    struct MSEdgeVertex {
        public EdgeCoord edge;
        public int edgeIdx; // MS edge index
        public ResolvedEdgeVertex resolved;
    }

    struct MSSegment {
        public MSEdgeVertex v1;
        public MSEdgeVertex v2;
        public V3? sharpFeature; // in cell normalized coordinates
        public V2 sharpFeature2; // on face
    }

    static bool CheckIntersection(MSSegment[] segments) {
        Debug.Assert(segments.Length == 2);
        (V2 s, V2 e)[] checkSegs = new (V2, V2)[4];
        int segCount = 0;
        // collect all segments
        foreach(var seg in segments) {
            if (seg.sharpFeature.HasValue) {
                checkSegs[segCount++] = (
                    seg.v1.resolved.localPos2,
                    seg.sharpFeature2
                );
                checkSegs[segCount++] = (
                    seg.sharpFeature2,
                    seg.v2.resolved.localPos2
                );
            } else {
                checkSegs[segCount++] = (
                    seg.v1.resolved.localPos2,
                    seg.v2.resolved.localPos2
                );
            }
        }
        // check all pairs of segments
        for (int i = 0; i < segCount; i++) {
            for (int j = i + 1; j < segCount; j++) {
                if (SegmentsIntersect(checkSegs[i].Item1, checkSegs[i].Item2, checkSegs[j].Item1, checkSegs[j].Item2)) {
                    return true;
                }
            }
        }
        return false;
    }

    static void ResolveEdgeVertex(HermiteData data, ref MSEdgeVertex ev, int faceId) {
        var intr = data.GetIntersectionWithLeaf(ev.edge);
        ref ResolvedEdgeVertex res = ref ev.resolved;
        res.edge = intr.Value.leaf;
        res.localNormal = Lookups.CubeDirToFaceDir(faceId, intr.Value.n);
        int cubeEdge = Lookups.faceEdgeToCubeEdge[faceId, ev.edgeIdx];
        var (e1s, e1e) = Lookups.EdgeOffset(cubeEdge, 1);
        res.localPos3 = V3.Lerp(e1s.ToV3(), e1e.ToV3(), intr.Value.t);
        res.localPos2 = Lookups.CubePosToFacePos(faceId, res.localPos3);
    }

    public void EvaluateCellFace(int cellId, FaceEdges edges, int faceId, List<CMSSegment> segments, byte cornerBits, I3 min, int size) {	

        int caseId = 0;
		for (int i = 0; i < 4; i++) {
			int corner = Lookups.faceCubeCorners[faceId, i];
            caseId |= (((cornerBits >> corner) & 1)) << i;
		}

		if (caseId == 0 || caseId == 15) {
			return; // no segments to draw
		}

        static void tryFindSharpFeature(ref MSSegment seg, int faceId) {
            ref ResolvedEdgeVertex ev1 = ref seg.v1.resolved;
            ref ResolvedEdgeVertex ev2 = ref seg.v2.resolved;
            if (ev1.localNormal is V2 n1 && ev2.localNormal is V2 n2) {
                // first line equation (perpendicular to normal)
                var t1 = new V2(n1.Y, -n1.X);
                // second line equation (perpendicular to normal)
                var t2 = new V2(-n2.Y, n2.X);
                // determinant for 2D matrix inversion
                var det = t1.X * t2.Y - t1.Y * t2.X;
                if (float.Abs(det) < 1e-9
                    || V2.Dot(n1, n2) >= SHARP_FEATURE_ANGLE_THRESHOLD) {
                    return;
                }
                var p1 = ev1.localPos2;
                var p2 = ev2.localPos2;
				var t = (t2.Y*(p2.X-p1.X) - t2.X*(p2.Y-p1.Y)) / det;
				var p3 = p1 + t * t1;
                // TODO: not sure what to do with this
                //   clamp or just ignore sharp feature?
				//p3.X = float.Clamp(p3.X, 0.0f, 1.0f);
				//p3.Y = float.Clamp(p3.Y, 0.0f, 1.0f);
                if (p3.X > -0.5f && p3.X < 1.5f
                    && p3.Y > -0.5f && p3.Y < 1.5f) {
                    seg.sharpFeature2 = p3;
                    seg.sharpFeature = Lookups.FacePosToCubePos(faceId, p3);
                }
            }
        }

        var segs = new MSSegment[2];
        segs[0] = new MSSegment();
        segs[1] = new MSSegment();

        // detect sharp feature and solve ambiguous cases
        int segCount = 0;
        for (int alt = 0; alt < 2; alt++) {
            segCount = 0;
            // use alt case if ambiguous and first fails
            int curCase = alt == 0 ? caseId : caseId switch {
                6 => 16, // 6 -> 6b
                9 => 17, // 9 -> 9b
                _ => 0
            };
            for (int i = 0; i < 4 && Lookups.quadSegments[curCase, i] >= 0; i+=2) {
                ref MSSegment seg = ref segs[i>>1];

                seg.v1.edgeIdx = Lookups.quadSegments[curCase, i];
                seg.v1.edge = edges[seg.v1.edgeIdx];
                ResolveEdgeVertex(data, ref seg.v1, faceId);

                seg.v2.edgeIdx = Lookups.quadSegments[curCase, i + 1];
                seg.v2.edge = edges[seg.v2.edgeIdx];
                ResolveEdgeVertex(data, ref seg.v2, faceId);
                tryFindSharpFeature(ref seg, faceId);
 
                segCount++;
            }
            // should never try alt case with only 1 segment
            Debug.Assert(alt == 0 || segCount == 2);

            // only 1 segment - not ambiguous
            if (segCount == 1) {
                break;
            } else {
                Debug.Assert(segCount == 2, "Should be 1 or 2 segments");
                // check if segments intersect
                if (CheckIntersection(segs)) {
                    if (alt == 0) {
                        // segments intersect, so we need to use alternative case
                        continue;
                    } else {
                        // somehow segments still intersect
                        //   just discard sharp features
                        segs[0].sharpFeature = null;
                        segs[1].sharpFeature = null;
                    }
                }
            }
        }

        // add segments
        for (int i = 0; i < segCount; i++) {
            ref MSSegment seg = ref segs[i];
            if (!seg.sharpFeature.HasValue) {
                segments.Add(new CMSSegment(
                    new CMSEdgeVertex(seg.v1.resolved.edge),
                    new CMSEdgeVertex(seg.v2.resolved.edge)
                ));
            } else {
                var sfv = seg.sharpFeature.Value;
                var newVert = new CMSNewVertex(sfv, min, size);
                var toNewVert1 = seg.sharpFeature2 - seg.v1.resolved.localPos2;
                var toNewVert2 = seg.sharpFeature2 - seg.v2.resolved.localPos2;
                float minDist = float.Max(V2.Dot(toNewVert1, toNewVert1), V2.Dot(toNewVert2, toNewVert2));
                // if new vertex is very close to one of the vertices
                //   then just discard it
                //   don't want to create very small triangles
                if (minDist < 1e-3) {
                    segments.Add(new CMSSegment(
                        new CMSEdgeVertex(seg.v1.resolved.edge),
                        new CMSEdgeVertex(seg.v2.resolved.edge)
                    ));
                } else {
                    segments.Add(new CMSSegment(
                        new CMSEdgeVertex(seg.v1.resolved.edge),
                        newVert
                    ));
                    segments.Add(new CMSSegment(
                        newVert,
                        new CMSEdgeVertex(seg.v2.resolved.edge)
                    ));
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void FaceInternalEdges(int faceIndex, int cellSize, I3 cellMin, Span<EdgeCoord> outArray) {
        var hsize = cellSize >> 1;
        var center = faceIndex switch {
            0 => new I3(0, hsize, hsize),
            1 => new I3(cellSize, hsize, hsize),
            2 => new I3(hsize, 0, hsize),
            3 => new I3(hsize, cellSize, hsize),
            4 => new I3(hsize, hsize, 0),
            5 or _ => new I3(hsize, hsize, cellSize),
        };
        center = cellMin + center;
        var dir = faceIndex>>1;
        var axis1 = dir switch {
            0 => new I3(0, hsize, 0),
            1 => new I3(0, 0, hsize),
            2 or _ => new I3(hsize, 0, 0),
        };
        var axis2 = dir switch {
            0 => new I3(0, 0, hsize),
            1 => new I3(hsize, 0, 0),
            2 or _ => new I3(0, hsize, 0),
        };
        var edgeDirs = dir switch {
            0 => (1, 2),
            1 => (2, 0),
            2 or _ => (0, 1),
        };
        outArray[0] = new EdgeCoord(center - axis1, hsize, edgeDirs.Item1);
        outArray[1] = new EdgeCoord(center, hsize, edgeDirs.Item1);
        outArray[2] = new EdgeCoord(center - axis2, hsize, edgeDirs.Item2);
        outArray[3] = new EdgeCoord(center, hsize, edgeDirs.Item2);
    }

    public void SubdivideFace(int faceUid, I3 cellMin, int cellSize, int faceIndex) {
        Debug.Assert(cellSize > 1 && (cellSize&1) == 0);
        // NOTE: assuming all face children are allocated continuously
        int firstFaceChild = newFace();
        int c2 = newFace();
        Debug.Assert(c2 == firstFaceChild + 1);
        int c3 = newFace();
        Debug.Assert(c3 == firstFaceChild + 2);
        int c4 = newFace();
        Debug.Assert(c4 == firstFaceChild + 3);
        var faceSpan = CollectionsMarshal.AsSpan(facePool);
        var faceEdgeSpan = CollectionsMarshal.AsSpan(faceEdgePool);
        ref FaceEdges face = ref faceEdgeSpan[faceUid];
        ref FaceEdges face0 = ref faceEdgeSpan[firstFaceChild + 0];
        ref FaceEdges face1 = ref faceEdgeSpan[firstFaceChild + 1];
        ref FaceEdges face2 = ref faceEdgeSpan[firstFaceChild + 2];
        ref FaceEdges face3 = ref faceEdgeSpan[firstFaceChild + 3];

        var hsize = cellSize >> 1;

        var newEdges = new EdgeCoord[4];
        FaceInternalEdges(faceIndex, cellSize, cellMin, newEdges);

        // inner
        face0[2] = newEdges[0];
        face0[3] = newEdges[2];
        face1[1] = newEdges[2];
        face1[2] = newEdges[1];
        face2[0] = newEdges[0];
        face2[3] = newEdges[3];
        face3[1] = newEdges[3];
        face3[0] = newEdges[1];

        // outer
        var (e01, e02) = face[0].Subdivide();
        face0[0] = e01;
        face1[0] = e02;
        var (e11, e12) = face[1].Subdivide();
        face0[1] = e11;
        face2[1] = e12;
        var (e21, e22) = face[2].Subdivide();
        face2[2] = e21;
        face3[2] = e22;
        var (e31, e32) = face[3].Subdivide();
        face1[3] = e31;
        face3[3] = e32;

        faceSpan[faceUid].firstChild = firstFaceChild;
    }

    public void GetCellFaceSegments(int cellId, List<CMSSegment> outSegments, List<CMSSegment>[] faceSegments) {
        bool reverse = false;
		void recurseFace(int faceId) {
			if (facePool[faceId].IsLeaf) {
                var segs = faceSegments[faceId];
                var len = segs.Count;
                // segments on negative face need to be reversed
                for (int i = 0; i < len; i++) {
                    var cseg = reverse ? segs[i] : segs[len - i - 1];
                    if (reverse) {
                        outSegments.Add(cseg);
                    } else {
                        // reverse segment
                        outSegments.Add(new CMSSegment(cseg.v2, cseg.v1));
                    }
                }
			} else {
                int fc = facePool[faceId].firstChild;
                recurseFace(fc);
                recurseFace(fc + 1);
                recurseFace(fc + 2);
                recurseFace(fc + 3);
			}
		}
        for (int i = 0; i < 6; i++) {
            var face = cellFacePool[cellId][i];
            // this ternary wouldn't be needed if I chose
            //   zx instead of xz as faces 2, 3
            //   but I don't feel like redoing all the lookup tables
            //reverse = (i>>1) == 1 ? (i&1) != 0 : (i&1) == 0;
            reverse = (i&1) == 0;
			recurseFace(face);
		}
    }

    // finds all segment loops in CMS cell
    public static Span<int[]> GetLoops(List<CMSSegment> segments) {
        var ret = new int[4][];
        int curRetIdx = 0;
        HashSet<CMSVertex> visited = new();
        Dictionary<CMSVertex, List<CMSSegment>> vertexSegments = new();
		foreach(var seg in segments) {
			if (!vertexSegments.ContainsKey(seg.v1)) {
				vertexSegments[seg.v1] = new List<CMSSegment>();
			}
			if (!vertexSegments.ContainsKey(seg.v2)) {
				vertexSegments[seg.v2] = new List<CMSSegment>();
			}
			vertexSegments[seg.v1].Add(seg);
			vertexSegments[seg.v2].Add(seg);
		}

		List<int> BuildLoop(CMSVertex start, HashSet<CMSSegment> usedSegments)
		{
			List<int> loop = new();
			CMSVertex current = start;
			CMSVertex previous = null;

			while (true)
			{
				visited.Add(current);
				var segs = vertexSegments[current];

				CMSSegment nextSeg = segs
					.FirstOrDefault(s => !usedSegments.Contains(s));

				if (nextSeg == default(CMSSegment)) {
                    throw new Exception("No next segment in loop");
					//break; // dead end or loop complete
				}

				usedSegments.Add(nextSeg);
				loop.Add(segments.IndexOf(nextSeg));

				CMSVertex next = nextSeg.v1.Equals(current) ? nextSeg.v2 : nextSeg.v1;

				if (next.Equals(start))
				{
					// closed loop
					break;
				}

				previous = current;
				current = next;
			}

			return loop;
		}
		HashSet<CMSSegment> usedSegments = new();

		foreach (var v in vertexSegments.Keys)
		{
			if (!visited.Contains(v))
			{
				var loop = BuildLoop(v, usedSegments);
                ret[curRetIdx++] = loop.ToArray();
			}
		}
#if DEBUG
        if (usedSegments.Count < segments.Count) {
            //throw new Exception("Not all segments were used!?");
            Debug.Error("Not all segments were used");
        }
#endif
		return ret.AsSpan().Slice(0, curRetIdx);
	}
}

[InlineArray(6)]
public struct CellFaces {
    public int element0;
}

public struct Cell {
    // 8 byte reference or 64 byte array
    public int firstChild;
    public bool IsLeaf => firstChild == -1;

    // subdivision constructor
    public Cell() {
        firstChild = -1;
    }      
}

public class HermiteData {
    // NOTE: might be better to use Dictionary for very sparse geometry
	public (V3 p, V3 n)?[,,,] intersections;
    //Dictionary<(int, int, int, int), (V3 p, V3 n)> intersections = new();
	public bool[,,] isInside;
	public V3 offset;
	public float step;
    public int size;

	public static HermiteData FromSdf(Func<V3, float> eval, Func<V3, V3> evalNormal, V3 offset, float step, int gridSize) {
        var ret = new HermiteData();
		ret.offset = offset;
		ret.step = step;
        ret.size = gridSize;

		var gp1 = gridSize+1;
		ret.isInside = new bool[gp1, gp1, gp1];
		ret.intersections = new (V3, V3)?[3, gp1, gp1, gridSize];
		// eval grid
		for (int i = 0; i < gp1; i++)
		for (int j = 0; j < gp1; j++)
		for (int k = 0; k < gp1; k++) {
			var location = new V3(offset.X + i * step, offset.Y + j * step, offset.Z + k * step);
			ret.isInside[i, j, k] = eval(location) <= 0.0f;
		}

		for (int dir = 0; dir < 3; dir++) {
			for (int k = 0; k < gridSize; k++)
			for (int i = 0; i < gp1; i++) {
				for (int j = 0; j < gp1; j++) {
					var sample1 = dir switch {
						0 => ret.isInside[k, i, j],
						1 => ret.isInside[i, k, j],
						2 => ret.isInside[i, j, k],
						_ => false
					};
					var sample2 = dir switch {
						0 => ret.isInside[k+1, i, j],
						1 => ret.isInside[i, k+1, j],
						2 => ret.isInside[i, j, k+1],
						_ => false
					};
					if (sample1 == sample2) continue;
					var start = dir switch {
						0 => new V3(offset.X + k * step, offset.Y + i * step, offset.Z + j * step),
						1 => new V3(offset.X + i * step, offset.Y + k * step, offset.Z + j * step),
						2 or _ => new V3(offset.X + i * step, offset.Y + j * step, offset.Z + k * step),
					};
					var end = start + (dir switch {
						0 => new V3(step, 0.0f, 0.0f),
						1 => new V3(0.0f, step, 0.0f),
						2 or _ => new V3(0.0f, 0.0f, step),
					});
					float curMin = float.MaxValue;
					V3 minPos = V3.Zero;
					for (int s = 0; s <= 100; s++) {
						V3 pos = V3.Lerp(start, end, s / 100.0f);
						float val = eval(pos);
						float dist = val*val;
						if (dist < curMin) {
							curMin = dist;
							minPos = pos;
						}
					}
					V3 minNormal = evalNormal(minPos);
					ret.intersections[dir, i, j, k] = (minPos, minNormal);
				}
			}
		}
        return ret;
	}

    public (V3 p, V3 n)? GetIntersection(EdgeCoord coord) {
        for (int i = 0; i < coord.count; i++) {
            (V3 v, V3 n)? cur = coord.dir switch {
                0 => this.intersections[0, coord.y, coord.z, coord.x+i],
                1 => this.intersections[1, coord.x, coord.z, coord.y+i],
                2 or _ => this.intersections[2, coord.x, coord.y, coord.z+i],
            };
            if (cur.HasValue) {
                return cur;
            }
        }
        return null;
    } 

    public (EdgeCoord leaf, float t, V3 n)? GetIntersectionWithLeaf(EdgeCoord coord) {
        for (int i = 0; i < coord.count; i++) {
            (V3 v, V3 n)? cur = coord.dir switch {
                0 => this.intersections[0, coord.y, coord.z, coord.x+i],
                1 => this.intersections[1, coord.x, coord.z, coord.y+i],
                2 => this.intersections[2, coord.x, coord.y, coord.z+i],
                _ => throw new ArgumentException(),
            };
            var leaf = coord.dir switch {
                0 => new EdgeCoord() { x = coord.x+i, y = coord.y, z = coord.z, count = 1, dir = coord.dir },
                1 => new EdgeCoord() { x = coord.x, y = coord.y+i, z = coord.z, count = 1, dir = coord.dir },
                2 or _ => new EdgeCoord() { x = coord.x, y = coord.y, z = coord.z+i, count = 1, dir = coord.dir },
            };
            if (cur.HasValue) {
                V3 start  = offset + new V3(leaf.x*step, leaf.y*step, leaf.z*step);
                float t = (cur.Value.v - start).Length() / step;
                return (leaf, t, cur.Value.n);
            }
        }
        return null;
    }

    // get intersection in cell-local coordinates
    public (V3 p, V3 n)? GetIntersectionNormalized(EdgeCoord coord, I3 cellMin, int cellSize) {
        for (int i = 0; i < coord.count; i++) {
            (V3 v, V3 n)? cur = coord.dir switch {
                0 => this.intersections[0, coord.y, coord.z, coord.x+i],
                1 => this.intersections[1, coord.x, coord.z, coord.y+i],
                2 or _ => this.intersections[2, coord.x, coord.y, coord.z+i],
            };
            if (cur is (V3 v, V3 n) curv) {
                V3 normPos = curv.v - offset - cellMin.ToV3()*step;
                Debug.Assert(float.Abs(cellSize) > 1e-6);
                normPos = normPos / ((float)cellSize * step);
                return (normPos, curv.n);
            }
        }
        return null;
    }

    // convert normalized vertex back to global position
    public V3 ResolveVertex(CMSNewVertex vert) {
        return vert.cellMin.ToV3()*step + offset + vert.position*vert.cellSize*step;
    }

    internal (float t, V3 n)[] FindEdgeIntersections(EdgeCoord coord) {
        Debug.Assert(coord.count > 0);
        List<(float t, V3 n)> intersectionsList = new();
        for (int i = 0; i < coord.count; i++) {
            (V3 v, V3 n)? cur = coord.dir switch {
                0 => this.intersections[0, coord.y, coord.z, coord.x + i],
                1 => this.intersections[1, coord.x, coord.z, coord.y + i],
                2 => this.intersections[2, coord.x, coord.y, coord.z + i],
                _ => throw new ArgumentException(),
            };
            if (cur.HasValue) {
                V3 start  = offset + new V3(coord.x*step, coord.y*step, coord.z*step);
                float t = (cur.Value.v - start).Length() / (coord.count*step);
                intersectionsList.Add((t, cur.Value.n));
            }
        }
        return intersectionsList.ToArray();
    }
}

public struct I3 {
    public int x, y, z;

    public I3(int x, int y, int z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    public V3 ToV3() => new V3(x, y, z);
    public override string ToString() => $"({x} {y} {z})";
    public static I3 operator+(I3 a, I3 b) => new I3(a.x+b.x, a.y+b.y, a.z+b.z);
    public static I3 operator-(I3 a, I3 b) => new I3(a.x-b.x, a.y-b.y, a.z-b.z);
    public static bool operator==(I3 a, I3 b) => a.x == b.x && a.y == b.y && a.z == b.z;
    public static bool operator!=(I3 a, I3 b) => !(a==b);
    public override bool Equals(object obj) => obj is I3 other && Equals(other);
    public override int GetHashCode() => HashCode.Combine(x, y, z);
};
