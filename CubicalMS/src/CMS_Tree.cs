using AnimLib;
using System;
using System.Linq;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;

public struct HermiteIntersection {
    public Vector3 pos;
    public Vector3 normal;
    public int dir;
    public float t;
}

// edge binary tree node
public class EdgeNode {
    public Vector3 s;
    public Vector3 e;
    public EdgeNode first;
    public EdgeNode second;
    public bool isInternal; // not really needed

    public float? intersectionT;
    public Vector3 intersectionNormal;

    public bool IsLeaf() {
        return first == null;
    }

    public void GetIntersections(List<(Vector3 pos, Vector3 normal, float t, EdgeNode node)> intersections, List<(Vector3 s, Vector3 e)> tests = null) {
        if (IsLeaf()) {
            tests?.Add((s, e));
            if (intersectionT != null) {
                intersections.Add((Vector3.Lerp(s, e, intersectionT.Value), intersectionNormal, intersectionT.Value, this));
            }
        } else {
            first.GetIntersections(intersections, tests);
            second.GetIntersections(intersections, tests);
        }
    }
}

public class CMSFace {
    public readonly EdgeNode[] edgeTrees = new EdgeNode[4];
    public CMSFace[] children;

    public CubicalMS.CMSVertex[] genVertices;
    public int[] genIndices;

    public bool IsLeaf() {
        return children == null;
    }

    public bool IsEvaluated() {
        return genVertices != null;
    }
}

public class CMSCell {
    public CMSCell[] children;
    public Vector3 min;
    public float size;

    public readonly CMSFace[] faceTrees = new CMSFace[6];

    public bool IsLeaf() {
        return children == null;
    }

    public void Subdivide() {
        children = new CMSCell[8];
        for (int i = 0; i < 8; i++) {
            children[i] = new CMSCell();
            children[i].min = min + CMS.cornerOffsets[i] * size * 0.5f;
            children[i].size = size / 2;
        }
    }

    public bool Contains(Vector3 pos) {
        Vector3 max = min + new Vector3(size);
        return pos.x >= min.x && pos.y >= min.y && pos.z >= min.z
            && pos.x < max.x && pos.y < max.y && pos.z < max.z;
    }

    protected HermiteIntersection[] getEdgeIntersections(int edge, Func<(Vector3 s, Vector3 e), HermiteIntersection[]> getIntersections) {
        var (startC, endC) = CMS.cubeEdgeCorners[edge];
        var startO = CMS.cornerOffsets[startC];
        var endO = CMS.cornerOffsets[endC];

        var start = this.min + startO * this.size;
        var end = this.min + endO * this.size;

        var results = getIntersections((start, end));
        return results ?? Array.Empty<HermiteIntersection>();
    }

    // SUBDIVIDECELL from Cubical Marching Squares paper
    public void SubdivideCell(Func<(Vector3 s, Vector3 e), HermiteIntersection[]> getIntersections, int maxDepth, int depth = 0) {
        if (depth > maxDepth) {
            return;
        }

        HermiteIntersection[][] edgeIntersections = new HermiteIntersection[12][];
        for (int edge = 0; edge < 12; edge++) {
            edgeIntersections[edge] = getEdgeIntersections(edge, getIntersections);
        }

        if (HasEdgeAmbiguity(edgeIntersections) || LikelyToContainComplexSurface(edgeIntersections)) {
            Subdivide();
            foreach (var child in children) {
                child.SubdivideCell(getIntersections, maxDepth, depth + 1);
            }
        }
    }


    public bool HasEdgeAmbiguity(HermiteIntersection[][] edgeIntersections) {
        bool foundAmbiguousEdge = false;
        for (int edge = 0; edge < 12; edge++) {
            var results = edgeIntersections[edge];
            if (results.Length > 1) {
                foundAmbiguousEdge = true;
            }
        }
        return foundAmbiguousEdge;
    }

    public bool LikelyToContainComplexSurface(HermiteIntersection[][] edgeIntersections) {
        var allIntersections = edgeIntersections.SelectMany(x => x).ToArray();
        if (allIntersections.Length < 2) return false;
        float max = 0.0f;
        for (int i = 0; i < allIntersections.Length; i++) {
            for (int j = i + 1; j < allIntersections.Length; j++) {
                var cosAngle = Vector3.Dot(allIntersections[i].normal, allIntersections[j].normal);
                var dist = 1.0f - cosAngle;
                max = Math.Max(max, dist);
            }
        }
        // closer to a 0 means more detail
        return max > 0.1f;
    }

    public void EvaluateFaces(Func<Vector3, bool> isInside) {
        Vector3[] corners = new Vector3[8];
        bool[] samples = new bool[8];
        for (int i = 0; i < 8; i++) {
            corners[i] = min + CMS.cornerOffsets[i] * size;
            samples[i] = isInside(corners[i]);
        }

        (Vector3 pos, Vector3 normal, float t, bool isSet)[] edgeBucket = new (Vector3, Vector3, float, bool)[12];
        Vector2[] intersectionNormal = new Vector2[4];
        Vector2[] intersectionPos2D = new Vector2[4];
        Vector3[] intersectionPos = new Vector3[4];

        bool[] faceSamples = new bool[4];
        for (int faceId = 0; faceId < 6; faceId++) {
            var faceTree = this.faceTrees[faceId];

            faceSamples[0] = samples[CMS.faceCubeCorners[faceId, 0]];
            faceSamples[1] = samples[CMS.faceCubeCorners[faceId, 1]];
            faceSamples[2] = samples[CMS.faceCubeCorners[faceId, 2]];
            faceSamples[3] = samples[CMS.faceCubeCorners[faceId, 3]];

            // early out, all inside or all outside
            if (faceSamples[0] == faceSamples[1] 
                && faceSamples[1] == faceSamples[2] 
                && faceSamples[2] == faceSamples[3]) {
                if (faceTree != null && faceTree.IsLeaf()) {
                    faceTree.genVertices = Array.Empty<CubicalMS.CMSVertex>();
                    faceTree.genIndices = Array.Empty<int>();
                }
                continue;
            }

            // only leaves need to be evaluated
            // internal face is an union of its descendants
            if (!faceTree.IsLeaf()) {
                continue;
            }

            // the face should be identical on both sides
            // only need to evaluate one side
            if (faceTree.IsEvaluated()) {
                continue;
            }

            for (int ei = 0; ei < 4; ei++) {
                var (c1, c2) = CMS.quadEdgeCorners[ei];
                var cubeEdge = CMS.faceEdgeToCubeEdge[faceId, ei];
                if (faceSamples[c1] != faceSamples[c2]) {
                    var p0 = corners[CMS.faceCubeCorners[faceId, c1]];
                    var p1 = corners[CMS.faceCubeCorners[faceId, c2]];

                    Vector3 normal, p;
                    Vector2 p2D;
                    float t;

                    if (edgeBucket[cubeEdge].isSet) {
                        normal = edgeBucket[cubeEdge].normal;
                        p = edgeBucket[cubeEdge].pos;
                        t = edgeBucket[cubeEdge].t;
                    } else {
                        List<(Vector3 pos, Vector3 normal, float t, EdgeNode node)> intersections = new();
                        var edgeNode = this.faceTrees[faceId].edgeTrees[ei];
                        if (edgeNode == null) {
                            Debug.Error($"edgeNode is null for cell edge with a sign change");
                        }
                        List<(Vector3 s, Vector3 e)> tests = new();
                        edgeNode?.GetIntersections(intersections, tests);

                        if (intersections.Count == 0) {
                            throw new Exception($"No intersections on edge with a sign change. {p0} {p1}, s1 {faceSamples[c1]} s2 {faceSamples[c2]}");
                        } else if (intersections.Count > 1) {
                            Debug.Warning($"Multiple intersections on edge with a sign change.");
                        }
                        normal = intersections[0].normal;

                        // intersection points in face space (0..1)
                        p = intersections[0].pos;
                        t = 0.0f;
                        var dif = p - p0;
                        if (MathF.Abs(dif.x) > float.Epsilon 
                            || MathF.Abs(dif.y) > float.Epsilon 
                            || MathF.Abs(dif.z) > float.Epsilon) {
                            t = (p - p0).Length / (p1 - p0).Length;
                        }
                        edgeBucket[cubeEdge] = (p, normal, t, true);
                    }

                    var lp0 = CMS.quadCorners[c1];
                    var lp1 = CMS.quadCorners[c2];
                    p2D = Vector2.Lerp(lp0, lp1, t);

                    intersectionNormal[ei] = CMS.ProjectOnFace(faceId, normal);
                    intersectionPos2D[ei] = p2D;
                    intersectionPos[ei] = p;
                }
            }

            CubicalMS.MSVertex2D[] outVertices = new CubicalMS.MSVertex2D[6];
            int[] outIndices = new int[8];
            CubicalMS.MarchSquareComplex(faceSamples, intersectionNormal, intersectionPos2D, outVertices, outIndices, out var vcount, out var icount);


            CubicalMS.CMSVertex[] genVertices = new CubicalMS.CMSVertex[vcount];
            int[] genIndices = new int[icount];

            for (int i = 0; i < vcount; i++) {
                var nv = new CubicalMS.CMSVertex();
                nv.isNew = outVertices[i].isNew;
                if (outVertices[i].isNew) {
                    nv.newVertex = CMS.FacePosToCubePos(faceId, outVertices[i].newVertex) * this.size + this.min;
                    nv.normal = Vector3.ZERO;
                } else {
                    var ii = outVertices[i].inputIndex;
                    nv.edgeIndex = ii;
                    nv.normal = intersectionNormal[ii];
                }
                genVertices[i] = nv;
            }

            for (int i = 0; i < icount; i++) {
                genIndices[i] = outIndices[i];
            }

            faceTree.genVertices = genVertices;
            faceTree.genIndices = genIndices;
        }
    }

    public void GetSegments(out (Vector3 v, Vector3 n, bool sharp)[] genVertices, out int[] genIndices) {
        List<(Vector3 v, Vector3 n, bool sharp)> retVertices = new();
        List<int> retIndices = new List<int>();

        int[] retVertToGenVert = new int[8];

        List<(Vector3 pos, Vector3 normal, float t, EdgeNode node)> intersections = new();
        Dictionary<EdgeNode, int> edgeNodeToIndex = new Dictionary<EdgeNode, int>();

        for (int faceId = 0; faceId < 6; faceId++) {
            if (faceTrees[faceId] == null) continue;

            void addSegments(CMSFace ftree) {
                if (ftree.IsLeaf()) {
                    Debug.Assert(ftree.IsEvaluated());
                    var varr = ftree.genVertices;
                    for (int i = 0; i < varr.Length; i++) {
                        if (varr[i].isNew) {
                            retVertToGenVert[i] = retVertices.Count;
                            retVertices.Add((varr[i].newVertex, varr[i].normal, true));
                        } else {
                            //var ei = CMS.faceEdgeToCubeEdge[faceId, varr[i].edgeIndex];
                            var en = ftree.edgeTrees[varr[i].edgeIndex];
                            intersections.Clear();
                            en.GetIntersections(intersections);
                            int vIdx = retVertices.Count;
                            bool found = false;
                            foreach(var intr in intersections) {
                                if (edgeNodeToIndex.TryGetValue(intr.node, out vIdx)) {
                                    found = true;
                                    break;
                                } 
                            }
                            if (!found) {
                                vIdx = retVertices.Count;
                                edgeNodeToIndex.Add(intersections[0].node, vIdx);
                                var lei = varr[i].edgeIndex;
                                retVertices.Add((intersections[0].pos, intersections[0].normal, false));
                            }
                            retVertToGenVert[i] = vIdx;
                        }
                    }
                    var iarr = ftree.genIndices;
                    for (int i = 0; i < iarr.Length; i += 2) {
                        retIndices.Add(retVertToGenVert[iarr[i]]);
                        retIndices.Add(retVertToGenVert[iarr[i + 1]]);
                    }
                } else {
                    foreach (var child in ftree.children) {
                        addSegments(child);
                    }
                }
            }

            var faceTree = this.faceTrees[faceId];
            addSegments(faceTree);
        }

        genVertices = retVertices.ToArray();
        genIndices = retIndices.ToArray();
    }
}

public class CMSTree {
    public CMSCell root;

    int countLeafCells(CMSCell cell) {
        if (cell.children == null) return 1;
        int count = 0;
        foreach (var child in cell.children) {
            count += countLeafCells(child);
        }
        return count;
    }

    public void getLeafCells(CMSCell cell, List<CMSCell> cells) {
        if (cell.children == null) {
            cells.Add(cell);
            return;
        }
        foreach (var child in cell.children) {
            getLeafCells(child, cells);
        }
    }

    public void InitTest(int n0, float size0, Vector3 offset) {
        root = new CMSCell();
        root.min = offset;
        root.size = size0;

        root.Subdivide();
        root.children[3].Subdivide();
        root.children[3].children[3].Subdivide();
        root.children[3].children[7].Subdivide();
        root.children[3].children[7].children[3].Subdivide();

        root.children[5].Subdivide();
        root.children[5].children[2].Subdivide();
    }

    public void Init(int n0, float size0, Vector3 offset, Func<(Vector3 s, Vector3 e), HermiteIntersection[]> getIntersections, int nmax = 1) {
        float cellSize = size0 / n0;

        root = new CMSCell();
        root.min = offset;
        root.size = size0;

        // subdivide to match n0
        int subDivisionCount = (int)Math.Log(n0, 2);
        void subdivide(CMSCell cell, int depth) {
            if (depth == subDivisionCount) return;
            cell.Subdivide();
            foreach (var child in cell.children) {
                subdivide(child, depth + 1);
            }
        }
        subdivide(root, 0);

        Debug.Assert(countLeafCells(root) == n0 * n0 * n0);

        List<CMSCell> leafCells = new List<CMSCell>();
        getLeafCells(root, leafCells);
        foreach (var cell in leafCells) {
            cell.SubdivideCell(getIntersections, nmax, 0);
        }
    }

    void FindEdgeIntersections(EdgeNode node, Func<(Vector3 s, Vector3 e), HermiteIntersection[]> getIntersections) {
        if (node == null) return;
        if (node.IsLeaf()) {
            if (node.intersectionT != null) return;
            Debug.Assert(node.s.x <= node.e.x);
            Debug.Assert(node.s.y <= node.e.y);
            Debug.Assert(node.s.z <= node.e.z);
            var intersections = getIntersections((node.s, node.e));
            if (intersections.Length > 0) {
                if (intersections.Length > 1) {
                    Debug.Warning($"Multiple intersections on leaf edge.");

                    Vector3 nsum = Vector3.ZERO;
                    float tsum = 0.0f;
                    foreach (var intr in intersections) {
                        nsum += intr.normal;
                        tsum += intr.t;
                    }
                    node.intersectionNormal = nsum.Normalized;
                    node.intersectionT = tsum / intersections.Length;

                    var sphere = new Sphere();
                    sphere.Radius = 0.01f;
                    sphere.Transform.Pos = intersections[0].pos;
                    sphere.Color = (1.3f*Color.RED);
                    //World.current.CreateInstantly(sphere);

                } else {
                    node.intersectionT = intersections[0].t;
                    node.intersectionNormal = intersections[0].normal;
                }
            }
        } else {
            FindEdgeIntersections(node.first, getIntersections);
            FindEdgeIntersections(node.second, getIntersections);
        }
    }

    public void FindEdgeIntersections(CMSCell cell, Func<(Vector3 s, Vector3 e), HermiteIntersection[]> getIntersections) {
        for (int face = 0; face < 6; face++) {
            var faceTree = cell.faceTrees[face];
            if (faceTree == null) continue;
            for (int edge = 0; edge < 4; edge++) {
                var edgeTree = faceTree.edgeTrees[edge];
                if (edgeTree == null) continue;
                FindEdgeIntersections(edgeTree, getIntersections);
            }
        }
        if (!cell.IsLeaf()) {
            foreach (var child in cell.children) {
                FindEdgeIntersections(child, getIntersections);
            }
        }
    }

    public void ExtractSurface(out Vector3[] outVertices, out int[] outIndices, Func<Vector3, bool> isInsideFunc) {
        var leaves = new List<CMSCell>();
        getLeafCells(root, leaves);

        Stack<(CMSCell cell, (Vector3 v, Vector3 n, bool sharp)[] vertices, int[] indices)> stack = new ();

        List<Vector3> retVertices = new List<Vector3>();
        List<int> retIndices = new List<int>();

        foreach (var leaf in leaves) {
            leaf.EvaluateFaces(isInsideFunc);
        }

        foreach (var leaf in leaves) {
            (Vector3 v, Vector3 n, bool sharp)[] vertices;
            int[] indices;
            leaf.GetSegments(out vertices, out indices);
            if (indices.Length == 0) continue;
            stack.Push((leaf, vertices, indices));
        }

        while (stack.Count > 0) {
            var (cell, vertices, indices) = stack.Pop();
            var newLines = new Line3D();
            List<Vector3> newVertices = new List<Vector3>();
            foreach (var idx in indices) {
                var rnd = random.NextSingle()*0.00f;
                newVertices.Add(vertices[idx].v + new Vector3(rnd));
            }

            bool[,] adjacencies = new bool[vertices.Length, vertices.Length];
            for (int i = 0; i < indices.Length; i += 2) {
                adjacencies[indices[i], indices[i + 1]] = true;
                adjacencies[indices[i + 1], indices[i]] = true;
            }

            bool[] visited = new bool[vertices.Length];
            List<int> nonVisited = new List<int>();
            for (int i = 0; i < vertices.Length; i++) {
                nonVisited.Add(i);
            }

            List<int> findLoop(int start) {
                List<int> loop = new List<int>();
                int current = start;
                while (true) {
                    loop.Add(current);
                    visited[current] = true;
                    nonVisited.Remove(current);
                    bool found = false;
                    for (int i = 0; i < vertices.Length; i++) {
                        if (adjacencies[current, i] && !visited[i]) {
                            current = i;
                            found = true;
                            break;
                        }
                    }
                    if (!found) break;
                }
                return loop;
            }

            bool firstLoop = true;

            void processLoop(List<int> loop, bool forceFlat = false) {
                if (!firstLoop) {
                    var color = Color.Random().WithA(0.8f);
                    Debug.Warning($"multiple loops not supported yet {cell.GetHashCode()}");
                    foreach (var idx in loop) {
                        var sphere = new Sphere();
                        sphere.Radius = 0.03f + random.NextSingle()*0.01f;
                        sphere.Transform.Pos = vertices[idx].v;
                        sphere.Color = color;
                        World.current.CreateInstantly(sphere);
                    }
                }

                var unique = loop.Distinct().ToArray();
                var sharpIndices = unique.Where(x => vertices[x].sharp).ToArray();
                if (sharpIndices.Length == 2 
                    && !adjacencies[sharpIndices[0], sharpIndices[1]]
                    && !forceFlat
                ) {
                    // split the loop in two
                    var sharpIdx = sharpIndices[0]; 
                    var sharpIdx2 = sharpIndices[1];

                    var idx1 = loop.IndexOf(sharpIdx);
                    var idx2 = loop.IndexOf(sharpIdx2);

                    List<int> cutLoop(List<int> loop, int idx1, int idx2) {
                        List<int> ret = new List<int>();
                        for (int i = idx1; i != idx2; i = (i + 1) % loop.Count) {
                            ret.Add(loop[i]);
                        }
                        ret.Add(loop[idx2]);
                        return ret;
                    }

                    var loop1 = cutLoop(loop, idx1, idx2);
                    var loop2 = cutLoop(loop, idx2, idx1);

                    if (loop1.Count > 2 && loop2.Count > 2) {
                        processLoop(loop1, forceFlat: true);
                        firstLoop = true;
                        processLoop(loop2, forceFlat: true);
                        return;
                    } else {
                        Debug.Warning("Tried to cut sharp feature, but the cut was too small.");
                    }
                }

                //Debug.Log($"loop: {loop.Count}");
                var nonZeroNormals = loop.Select(x => vertices[x].n != Vector3.ZERO ? ((Vector3, Vector3)?)(vertices[x].n, vertices[x].v) : null).OfType<(Vector3, Vector3)?>().ToArray();
                int nonZeroCount = nonZeroNormals.Length;
                var matrix = Matrix<float>.Build.Dense(nonZeroCount, 3);
                var vector = Vector<float>.Build.Dense(nonZeroCount, 1.0f);
                //Debug.Log($"nonZeroCount: {nonZeroCount}");
                for (int i = 0; i < nonZeroCount; i++) {
                    var n = nonZeroNormals[i].Value.Item1;
                    matrix.SetRow(i, new float[] { n.x, n.y, n.z });
                    vector[i] = Vector3.Dot(nonZeroNormals[i].Value.Item1, nonZeroNormals[i].Value.Item2);
                }
                //matrix = matrix.Transpose();

                Vector3 componentCenter;
                try {
                    if (forceFlat) {
                        throw new Exception("force flat");
                    }
                    var pseudoInverse = matrix.PseudoInverse();
                    var result = pseudoInverse * vector;
                    var p = new Vector3(result[0], result[1], result[2]);
                    var residual = matrix * result - vector;
                    var residualNorm = residual.L2Norm();

                    if (residualNorm > 1e-4) {
                        throw new Exception("residual too large");
                    }
                    if (cell.Contains(p)) {
                        componentCenter = p;
                        var sphere = new Sphere();
                        sphere.Radius = 0.03f + random.NextSingle()*0.01f;
                        sphere.Transform.Pos = p;
                        sphere.Color = Color.ORANGE.WithA(0.8f);
                        //World.current.CreateInstantly(sphere);
                    } else {
                        throw new Exception("not in cell");
                    }
                    //Debug.Log($"p: {p}");
                } catch (Exception) {
                    Vector3 sum = Vector3.ZERO;
                    var distinctIndices = loop.Distinct().ToArray();
                    foreach (var idx in distinctIndices) {
                        sum += vertices[idx].v;
                    }
                    componentCenter = sum * (1.0f / distinctIndices.Length);
                    if (!firstLoop) {
                        var sphere = new Sphere();
                        sphere.Radius = 0.03f + random.NextSingle()*0.01f;
                        sphere.Transform.Pos = componentCenter;
                        sphere.Color = 1.5f*Color.ORANGE;
                        World.current.CreateInstantly(sphere);
                    }
                }
                int offset = retVertices.Count;
                retVertices.AddRange(loop.Select(x => vertices[x].v));
                int centerIdx = retVertices.Count;
                retVertices.Add(componentCenter);
                for (int i = 0; i < loop.Count; i++) {
                    retIndices.Add(offset + i);
                    retIndices.Add(centerIdx);
                    retIndices.Add(offset + (i + 1) % loop.Count);
                }
                firstLoop = false;
            }

            while (nonVisited.Count > 0) {
                var loop = findLoop(nonVisited[0]);
                if (loop.Count > 2) {
                    processLoop(loop);
                } else {
                    processLoop(loop);
                    //throw new Exception($"Not a loop {loop.Count}");
                    Debug.Error($"Not a loop {loop.Count} {vertices.Length} #{cell.GetHashCode()}");
                    {
                        var sb = new System.Text.StringBuilder();
                        sb.Append($"loop: {loop.Count} | ");
                        for (int j = 0; j < loop.Count; j++) {
                            sb.Append($"{loop[j]} ");
                        }
                        sb.AppendLine();
                        sb.Append("Non visited: ");
                        for (int j = 0; j < nonVisited.Count; j++) {
                            sb.Append($"{nonVisited[j]} ");
                        }
                        sb.AppendLine();
                        sb.Append("Adjacencies: ");
                        sb.AppendLine();
                        for (int j = 0; j < vertices.Length; j++) {
                            sb.Append($"{j:00}: ");
                            for (int k = 0; k < vertices.Length; k++) {
                                sb.Append($"{(adjacencies[j, k] ? 1 : 0)} ");
                            }
                            sb.AppendLine();
                        }
                        sb.AppendLine();
                        sb.Append("Vertices: ");
                        sb.AppendLine();
                        for (int j = 0; j < vertices.Length; j++) {
                            sb.Append($"{j:00}: {vertices[j].v} {vertices[j].n}");
                            sb.AppendLine();
                        }
                        Debug.Log(sb.ToString());
                    }
                    var sphere = new Sphere();
                    sphere.Radius = 0.03f + random.NextSingle()*0.01f;
                    sphere.Transform.Pos = vertices[loop[0]].v;
                    sphere.Color = Color.RED.WithA(0.8f);
                    World.current.CreateInstantly(sphere);
                }
            }

            newLines.Vertices = newVertices.ToArray();
            newLines.Color = Color.Random().WithA(0.8f);
            newLines.Width = 6.0f;
            //World.current.CreateInstantly(newLines);
         }
        outVertices = retVertices.ToArray();
        outIndices = retIndices.ToArray();
    }

    public void TraceComponents() {
        var leaves = new List<CMSCell>();
        getLeafCells(root, leaves);
        foreach (var leaf in leaves) {
        }
    }

    EdgeNode TraverseEdge(CMSCell cell, int edgeId, bool isInternal) {
        var node = new EdgeNode();
        node.isInternal = isInternal;
        if (cell.IsLeaf()) {
            Vector3[] corners = new Vector3[8];
            for (int i = 0; i < 8; i++) {
                corners[i] = cell.min + CMS.cornerOffsets[i] * cell.size;
            }
            var (c0, c1) = CMS.cubeEdgeCorners[edgeId];
            node.s = corners[c0];
            node.e = corners[c1];
        } else {
            var segments1 = TraverseEdge(cell.children[CMS.edgeCells[edgeId, 0]], edgeId, isInternal);
            var segments2 = TraverseEdge(cell.children[CMS.edgeCells[edgeId, 1]], edgeId, isInternal);
            node.first = segments1;
            node.second = segments2;
        }
        return node;
    }
    static Random random = new Random();

    void AssignEdgeTree(CMSCell cell, int edgeId, EdgeNode node) {
        if (node == null) return;
        var (face1, fedge1) = CMS.cubeEdgeToFaceEdge[edgeId, 0];
        var (face2, fedge2) = CMS.cubeEdgeToFaceEdge[edgeId, 1];
        cell.faceTrees[face1].edgeTrees[fedge1] = node;
        cell.faceTrees[face2].edgeTrees[fedge2] = node;
        if (cell.children == null) return;
        AssignEdgeTree(cell.children[CMS.edgeCells[edgeId, 0]], edgeId, node.first);
        AssignEdgeTree(cell.children[CMS.edgeCells[edgeId, 1]], edgeId, node.second);
    }

    EdgeNode MergeEdgeNodes(EdgeNode dst, EdgeNode alt) {
        if (dst == null) return alt;
        if (alt == null) return dst;
        var node = dst;
        node.first = MergeEdgeNodes(dst.first, alt.first);
        node.second = MergeEdgeNodes(dst.second, alt.second);
        return node;
    }

    // assigns edges that are in the middle of 4 cells
    public void AssignInternalEdges(CMSCell treeCell) {
        if (treeCell.IsLeaf()) return;
        for (int face = 0; face < 6; face++) {
            EdgeNode[] edgeNodes = new EdgeNode[4];
            for (int faceCell = 0; faceCell < 4; faceCell++) {
                var (cell, edge) = CMS.faceCells[face, faceCell];
                edgeNodes[faceCell] = TraverseEdge(treeCell.children[cell], edge, true);
            }
            // merge edge trees preserving most detail
            var edgeNode = MergeEdgeNodes(edgeNodes[0], edgeNodes[1]);
            edgeNode = MergeEdgeNodes(edgeNode, edgeNodes[2]);
            edgeNode = MergeEdgeNodes(edgeNode, edgeNodes[3]);

            for (int faceCell = 0; faceCell < 4; faceCell++) {
                var (cell, edge) = CMS.faceCells[face, faceCell];
                AssignEdgeTree(treeCell.children[cell], edge, edgeNode);
            }
        }
        if (treeCell.children == null) return;
        foreach (var child in treeCell.children) {
            AssignInternalEdges(child);
        }
    }

    // quadtree of edges between faces
    // used for merging the detail between shared faces
    class FaceNode {
        public FaceNode[] children;
        public EdgeNode[] edgeTrees;
    }

    CMSFace ConstructFaceTree(CMSCell cell, int faceId, FaceNode faceNode) {
        if (faceNode == null) return null;
        var node = new CMSFace();
        if (cell != null) {
            cell.faceTrees[faceId] = node;
        }
        if (faceNode.children != null) {
            node.children = new CMSFace[4];
            for (int i = 0; i < 4; i++) {
                var (cId1, _) = CMS.faceCells[faceId, i];
                var child1 = (cell == null || cell.IsLeaf()) 
                    ? null 
                    : cell.children[cId1];
                node.children[i] = ConstructFaceTree(child1, faceId, faceNode.children[i]);
            }
        }
        return node;
    }

    CMSFace ConstructFaceTree(CMSCell cell1, CMSCell cell2, int faceId1, int faceId2, FaceNode faceNode) {
        if (faceNode == null) return null;
        var node = new CMSFace();
        if (cell1 != null) {
            cell1.faceTrees[faceId1] = node;
        }
        if (cell2 != null) {
            cell2.faceTrees[faceId2] = node;
        }
        Debug.Assert((cell1 == null && cell2 == null) || cell1 != cell2);
        if (faceNode.children != null) {
            node.children = new CMSFace[4];
            for (int i = 0; i < 4; i++) {
                var (cId1, _) = CMS.faceCells[faceId1, i];
                var child1 = (cell1 == null || cell1.IsLeaf()) 
                    ? null 
                    : cell1.children[cId1];
                var (cId2, _) = CMS.faceCells[faceId2, i];
                var child2 = (cell2 == null || cell2.IsLeaf())
                    ? null
                    : cell2.children[cId2];
                node.children[i] = ConstructFaceTree(child1, child2, faceId1, faceId2, faceNode.children[i]);
            }
        }
        return node;
    }

    // assign face "edges" to the octree
    void AssignFace(CMSCell treeCell, int faceId, FaceNode faceNode) {
        //treeCell.faceTrees[faceId] = faceNode;
        if (!treeCell.IsLeaf()) {
            for (int i = 0; i < 4; i++) {
                // TODO: change to faceCubeCorners?
                var (cId, _) = CMS.faceCells[faceId, i];
                AssignFace(treeCell.children[cId], faceId, faceNode.children[i]);
                var (cId21, eId1) = CMS.faceEdges[faceId, i, 0];
                var (cId22, eId2) = CMS.faceEdges[faceId, i, 1];
                AssignEdgeTree(treeCell.children[cId21], eId1, faceNode.edgeTrees[i]);
                AssignEdgeTree(treeCell.children[cId22], eId2, faceNode.edgeTrees[i]);
            }
        }
    }

    // traverse edges on a face of a cell
    FaceNode TraverseFace(CMSCell treeCell, int faceId) {
        FaceNode faceNode = new FaceNode();
        if (!treeCell.IsLeaf()) {
            faceNode.children = new FaceNode[4];
            faceNode.edgeTrees = new EdgeNode[4];
            for (int i = 0; i < 4; i++) {
                // internal "+" of 4 children
                var (cId, _) = CMS.faceCells[faceId, i];
                faceNode.children[i] = TraverseFace(treeCell.children[cId], faceId);

                // edges between children (the "+" of this face)
                var (cId21, eId1) = CMS.faceEdges[faceId, i, 0];
                var (cId22, eId2) = CMS.faceEdges[faceId, i, 1];
                var t1 = TraverseEdge(treeCell.children[cId21], eId1, false);
                var t2 = TraverseEdge(treeCell.children[cId22], eId2, false);
                var merged = MergeEdgeNodes(t1, t2);
                faceNode.edgeTrees[i] = merged;
            }
        }
        return faceNode;
    }

    // merge two face quadtrees, preserving most detail
    FaceNode MergeFaceNodes(FaceNode node1, FaceNode node2) {
        if (node1 == null) return node2;
        if (node2 == null) return node1;
        var node = new FaceNode() {
            edgeTrees = new EdgeNode[4],
            children = new FaceNode[4],
        };
        var et1 = node1.edgeTrees;
        var et2 = node2.edgeTrees;
        var ct1 = node1.children;
        var ct2 = node2.children;
        bool hadChildren = false;
        for (int i = 0; i < 4; i++) {
            node.edgeTrees[i] = MergeEdgeNodes(et1?[i], et2?[i]);
            node.children[i] = MergeFaceNodes(ct1?[i], ct2?[i]);
            if (node.children[i] != null) {
                hadChildren = true;
            }
        }
        if (!hadChildren) {
            node.children = null;
        }
        return node;
    }

    public void ConstructFaceTrees(CMSCell treeCell) {
        if (treeCell.IsLeaf()) return;

        for (int i = 0; i < 12; i++) {
            var (cId1, fId1) = CMS.internalFaceCells[i, 0];
            var (cId2, fId2) = CMS.internalFaceCells[i, 1];
            var cell1 = treeCell.children[cId1];
            var cell2 = treeCell.children[cId2];

            var qtree1 = TraverseFace(cell1, fId1);
            var qtree2 = TraverseFace(cell2, fId2);
            var merged = MergeFaceNodes(qtree1, qtree2);

            ConstructFaceTree(cell1, cell2, fId1, fId2, merged);
        }

        foreach (var child in treeCell.children) {
            ConstructFaceTrees(child);
        }
    }

    // edges between cell faces
    // the 'transition faces' if one is a leaf cell and the other is not
    public void AssignInternalFaceEdges(CMSCell treeCell) {
        if (treeCell.IsLeaf()) return;

        for (int i = 0; i < 12; i++) {
            var (cId1, fId1) = CMS.internalFaceCells[i, 0];
            var (cId2, fId2) = CMS.internalFaceCells[i, 1];
            var cell1 = treeCell.children[cId1];
            var cell2 = treeCell.children[cId2];

            var qtree1 = TraverseFace(cell1, fId1);
            var qtree2 = TraverseFace(cell2, fId2);
            var merged = MergeFaceNodes(qtree1, qtree2);

            AssignFace(cell1, fId1, merged);
            AssignFace(cell2, fId2, merged);
        }

        foreach (var child in treeCell.children) {
            AssignInternalFaceEdges(child);
        }
    }

    public void CalculateIntersections(CMSCell rootCell, Func<(Vector3 s, Vector3 e), HermiteIntersection[]> getIntersections) {
        var sw = new System.Diagnostics.Stopwatch();
        ConstructFaceTrees(rootCell);
        Debug.Log($"ConstructFaceTrees: {sw.ElapsedMilliseconds}ms");
        sw.Restart();
        AssignInternalFaceEdges(rootCell);
        Debug.Log($"AssignInternalFaceEdges: {sw.ElapsedMilliseconds}ms");
        sw.Restart();
        AssignInternalEdges(rootCell);
        Debug.Log($"AssignInternalEdges: {sw.ElapsedMilliseconds}ms");
        sw.Restart();
        FindEdgeIntersections(rootCell, getIntersections);
        Debug.Log($"FindEdgeIntersections: {sw.ElapsedMilliseconds}ms");
        sw.Stop();
    }
}
