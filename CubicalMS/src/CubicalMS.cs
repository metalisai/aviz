using System;
using System.Linq;
using AnimLib;
using System.Collections.Generic;
using System.Threading.Tasks;
using CommunityToolkit.HighPerformance;

public class CubicalMS : AnimationBehaviour
{
    World world;
    Animator animator;

    Color[] cornerColors = new Color[] {
        new Color(0.0f, 0.0f, 0.0f, 1.0f),
        new Color(1.0f, 0.0f, 0.0f, 1.0f),
        new Color(1.0f, 1.0f, 0.0f, 1.0f),
        new Color(0.0f, 1.0f, 0.0f, 1.0f),
        new Color(0.0f, 0.0f, 1.0f, 1.0f),
        new Color(1.0f, 0.0f, 1.0f, 1.0f),
        new Color(0.5f, 0.5f, 0.5f, 1.0f),
        new Color(0.0f, 1.0f, 1.0f, 1.0f),
    };


public void Init(AnimationSettings settings) {
    settings.Name = "My animation";
    // animation length must be bounded
    // (it gets "baked" to allow seeking whole animation in editor)
    settings.MaxLength = 600.0f; 
}

/*
// edges relative to lower left corner
// vertex will be placed between these two points
(Vector2 s, Vector2 e)[] quadEdges = new (Vector2, Vector2)[] {
    (new Vector2(0, 0), new Vector2(1, 0)),
    (new Vector2(1, 0), new Vector2(1, 1)),
    (new Vector2(1, 1), new Vector2(0, 1)),
    (new Vector2(0, 1), new Vector2(0, 0)),
};*/

static float opUnion(float d1, float d2) {
    return MathF.Min(d1, d2);
}

static float opSubtract(float d1, float d2) {
    return MathF.Max(-d1, d2);
}

static float opXor(float d1, float d2) {
    return MathF.Max(MathF.Min(d1, d2), -MathF.Max(d1, d2));
}

static float sdSphere(Vector3 p, float r)
{
    return p.Length - r;
}

static float sdBox(Vector3 p, Vector3 b )
{
    var q = (p).Abs - b;
    var t1 = Vector3.Max(q, new Vector3(0.0f));
    return t1.Length + MathF.Min(MathF.Max(q.x,MathF.Max(q.y,q.z)), 0.0f);
}

static float sdTorus(Vector3 p/*, vec2 t */)
{
    Vector2 t = new Vector2(5.0f, 2.0f);
    float pxzm = new Vector2(p.x, p.z).Length;
    Vector2 q = new Vector2(pxzm-t.x, p.y);
    return q.Length - t.y;
}

static Vector3 sdTorusNormal(Vector3 p)
{
    float R = 5.0f;
    Vector2 c = Vector2.ZERO;

    Vector2 v = new Vector2(p.x, p.z);
    float magV = v.Length;
    Vector2 a = c + v / magV * R;

    //return (p - a).normalized;
    return (p - new Vector3(a.x, 0.0f, a.y)).Normalized;
}

public static float Evaluate2(Vector3 pos) {
    //return sdTorus(pos);
    var torusBox = opUnion(sdBox(pos, new Vector3(4.11111111111f, 6.111111111f, 4.1111111111111f)), sdTorus(pos));
    var subSphere = opUnion(torusBox, sdSphere(pos - new Vector3(0.0f, 5.0f, -2.0f), 3.0f));
    subSphere = opSubtract(sdSphere(pos - new Vector3(0.0f, -5.0f, -2.0f), 3.0f), subSphere);
    return subSphere;
    //return torusBox;
}

public static float Evaluate(Vector3 pos) {
    //return sdTorus(pos);
    var torusBox = opUnion(sdBox(pos, new Vector3(4.11111111111f, 6.111111111f, 4.1111111111111f)), sdTorus(pos));
    var subSphere = opUnion(torusBox, sdSphere(pos - new Vector3(0.0f, 5.0f, -2.0f), 3.0f));
    subSphere = opSubtract(sdSphere(pos - new Vector3(0.0f, -5.0f, -2.0f), 3.0f), subSphere);
    return subSphere;
    //return torusBox;
}

public static Vector3 EvaluateNormal(Vector3 pos) {
    //return sdTorusNormal(pos);
    // use gradient approximation
    float eps = 0.001f;
    float d = Evaluate2(pos);
    float dx = Evaluate2(pos + new Vector3(eps, 0.0f, 0.0f));
    float dy = Evaluate2(pos + new Vector3(0.0f, eps, 0.0f));
    float dz = Evaluate2(pos + new Vector3(0.0f, 0.0f, eps));
    return new Vector3(dx - d, dy - d, dz - d).Normalized;
}

public HermiteIntersection[] GetIntersections((Vector3 start, Vector3 end) edge) {
    int steps = 128;
    Vector3 step = (edge.end - edge.start) * (1.0f/steps);
    float lastSample = Evaluate(edge.start);
    List<HermiteIntersection> intersections = new List<HermiteIntersection>();
    for (int i = 0; i < steps+1; i++) {
        Vector3 samplePos = edge.start + step * i;
        float sample = Evaluate(samplePos);

        if (MathF.Sign(sample) != MathF.Sign(lastSample)) {
            HermiteIntersection intersection = new HermiteIntersection();
            intersection.pos = samplePos;
            intersection.normal = EvaluateNormal(samplePos);
            intersection.t = (float)i / steps;
            intersections.Add(intersection);
        }

        lastSample = sample;
    }
    return intersections.ToArray();
}

static public void MarchSquare(float[] cornerSamples, int[] buffer, out int outputCount) {
    bool sample0 = cornerSamples[0] < 0.0f;
    bool sample1 = cornerSamples[1] < 0.0f;
    bool sample2 = cornerSamples[2] < 0.0f;
    bool sample3 = cornerSamples[3] < 0.0f;
    MarchSquare(new bool[] { sample0, sample1, sample2, sample3 }, buffer, out outputCount);
}

static public void MarchSquare(bool[] cornerSamples, int[] buffer, out int outputCount) {
    bool sample0 = cornerSamples[0];
    bool sample1 = cornerSamples[1];
    bool sample2 = cornerSamples[2];
    bool sample3 = cornerSamples[3];
    int caseId = (sample0 ? 1 : 0) | (sample1 ? 2 : 0) | (sample2 ? 4 : 0) | (sample3 ? 8 : 0);

    outputCount = 0;
    for (int i = 0; i < 4; i+=2) {
        int startEdge = CMS.quadSegments[caseId, i];
        if (startEdge == -1) {
            return;
        }
        int endEdge = CMS.quadSegments[caseId, i+1];
        buffer[outputCount++] = startEdge;
        buffer[outputCount++] = endEdge;
    }
}

static bool lineListHasIntersection(Vector2[] vertices, int vcount, int[] indices, int icount) {
    int lineCount = indices.Length / 2;
    for (int i = 0; i < lineCount; i+=2) {
        var v1 = vertices[indices[i]];
        var v2 = vertices[indices[i+1]];

        for (int j = i+2; j < lineCount; j+=2) {
            var v3 = vertices[indices[j]];
            var v4 = vertices[indices[j+1]];

            if (AMath.SegmentsIntersect(v1, v2, v3, v4)) {
                return true;
            }
        }
    }
    return false;
}

const float SHARP_FEATURE_ANGLE_THRESHOLD = 0.99f;

public struct MSVertex2D {
    public bool isNew; // is this a new vertex generated by sharp feature detection
    public Vector2 newVertex; // coordinate in face-space, valid if isNew is true
    public int inputIndex; // input intersection point index, valid if isNew is false
}

// cell face vertex (discriminated union of edge intersection point and new vertex)
public struct CMSVertex {
    public bool isNew;
    public Vector3 newVertex; // a new vertex generated by sharp feature detection
    public int edgeIndex; // cell edge index

    public Vector3 normal;
}

static public void MarchSquareComplex(bool[] cornerSamples, Vector2[] intersectionNormals, Vector2[] intersectionPositions, MSVertex2D[] outputVertices, int[] outputSegments, out int outVCount, out int outICount) {
    outVCount = 0;
    outICount = 0;

    bool sample0 = cornerSamples[0];
    bool sample1 = cornerSamples[1];
    bool sample2 = cornerSamples[2];
    bool sample3 = cornerSamples[3];
    int caseId = (sample0 ? 1 : 0) | (sample1 ? 2 : 0) | (sample2 ? 4 : 0) | (sample3 ? 8 : 0);

    if (caseId == 0 || caseId == 15) {
        return;
    }

    Vector2? detectSharpFeature(int startEdge, int endEdge) {
        var p1 = intersectionPositions[startEdge];
        var p2 = intersectionPositions[endEdge];

        Debug.Assert(p1.x >= 0.0f && p1.x <= 1.0f);
        Debug.Assert(p1.y >= 0.0f && p1.y <= 1.0f);
        Debug.Assert(p2.x >= 0.0f && p2.x <= 1.0f);
        Debug.Assert(p2.y >= 0.0f && p2.y <= 1.0f);

        var normal1 = intersectionNormals[startEdge].Normalized;
        var normal2 = intersectionNormals[endEdge].Normalized;
        if (Vector2.Dot(normal1, normal2) > SHARP_FEATURE_ANGLE_THRESHOLD) {
            return null;
        }
        var tangent1 = normal1.PerpCw;
        var tangent2 = normal2.PerpCcw;
        // solution by inverse matrix (A^-1 * b)
        var det = tangent1.x*tangent2.y - tangent1.y*tangent2.x;
        if (det == 0.0f) {
            return null;
        }
        var t = (tangent2.y*(p2.x-p1.x) - tangent2.x*(p2.y-p1.y)) / det;
        var sharpFeature = p1 + tangent1 * t;
        sharpFeature.x = Math.Clamp(sharpFeature.x, 0.0f, 1.0f);
        sharpFeature.y = Math.Clamp(sharpFeature.y, 0.0f, 1.0f);
        return sharpFeature;
    }

    var qSegs = new Span2D<int>(CMS.quadSegments).GetRow(caseId).ToArray();
    if (caseId != 5 && caseId != 10) {
        int startEdge = CMS.quadSegments[caseId, 0];
        int endEdge = CMS.quadSegments[caseId, 1];
        var sharpFeature = detectSharpFeature(startEdge, endEdge);
        outVCount = sharpFeature == null ? 2 : 3;
        outICount = sharpFeature == null ? 2 : 4;
        outputVertices[0] = new MSVertex2D() { 
            isNew = false, 
            inputIndex = startEdge 
        };
        outputVertices[1] = new MSVertex2D() { 
            isNew = false, 
            inputIndex = endEdge 
        };
        if (sharpFeature != null) {
            outputVertices[2] = new MSVertex2D() { 
                isNew = true, 
                newVertex = sharpFeature.Value 
            };
            outputSegments[0] = 0;
            outputSegments[1] = 2;
            outputSegments[2] = 2;
            outputSegments[3] = 1;
        } else {
            outputSegments[0] = 0;
            outputSegments[1] = 1;
        }
    } else { // ambiguous case
        var altSegs = new Span2D<int>(CMS.alternativeSegments).GetRow(caseId == 5 ? 0 : 1).ToArray();

        // there could be a gap between corners
        // or they could be connected (for example see case 5 in marching squares)
        // detecting sharp feature might make one of the cases impossible
        // so we consider both cases and pick the one that has no intersections
        int[][] possibleSegments = new int[2][];
        possibleSegments[0] = qSegs;
        possibleSegments[1] = altSegs;

        MSVertex2D[,] potentialVertices = new MSVertex2D[2, 6];
        int[,] potentialSegments = new int[2, 8];

        int[] vCounts = new int[2];
        int[] iCounts = new int[2];

        for (int si = 0; si < 2; si++) {
            int s1 = possibleSegments[si][0];
            int e1 = possibleSegments[si][1];
            var sFeature1 = detectSharpFeature(s1, e1);
            int s2 = possibleSegments[si][2];
            int e2 = possibleSegments[si][3];
            var sFeature2 = detectSharpFeature(s2, e2);

            potentialVertices[si, 0] = new MSVertex2D() { 
                isNew = false, 
                inputIndex = s1 
            };
            potentialVertices[si, 1] = new MSVertex2D() { 
                isNew = false, 
                inputIndex = e1 
            };
            potentialVertices[si, 2] = new MSVertex2D() { 
                isNew = false, 
                inputIndex = s2 
            };
            potentialVertices[si, 3] = new MSVertex2D() { 
                isNew = false, 
                inputIndex = e2 
            };
            int vi = 4;
            int ii = 0;
            if (sFeature1 != null) {
                potentialVertices[si, vi++] = new MSVertex2D() { 
                    isNew = true, 
                    newVertex = sFeature1.Value 
                };
                potentialSegments[si, ii++] = 0;
                potentialSegments[si, ii++] = vi-1;
                potentialSegments[si, ii++] = vi-1;
                potentialSegments[si, ii++] = 1;
            }
            if (sFeature2 != null) {
                potentialVertices[si, vi++] = new MSVertex2D() { 
                    isNew = true, 
                    newVertex = sFeature2.Value 
                };
                potentialSegments[si, ii++] = 2;
                potentialSegments[si, ii++] = vi-1;
                potentialSegments[si, ii++] = vi-1;
                potentialSegments[si, ii++] = 3;
            }
            vCounts[si] = vi;
            iCounts[si] = ii;
        }

        var vs1 = potentialVertices.GetRow(0).ToArray().Select(
            v => v.isNew ? v.newVertex : intersectionPositions[v.inputIndex]
        ).ToArray();
        var is1 = potentialSegments.GetRow(0).ToArray();
        var vs2 = potentialVertices.GetRow(1).ToArray().Select(
            v => v.isNew ? v.newVertex : intersectionPositions[v.inputIndex]
        ).ToArray();
        var is2 = potentialSegments.GetRow(1).ToArray();
        var hasIntersection1 = lineListHasIntersection(vs1, vCounts[0], is1, iCounts[0]);
        var hasIntersection2 = lineListHasIntersection(vs2, vCounts[1], is2, iCounts[1]);

        int selectedOption = 0;
        if (!hasIntersection1) {
            selectedOption = 0;
        } else if (!hasIntersection2) {
            selectedOption = 1;
        } else {
            //throw new Exception("Both cases have intersection");
            Debug.Error("Both cases have intersection");
        }

        outVCount = vCounts[selectedOption];
        outICount = iCounts[selectedOption];
        for (int i = 0; i < outVCount; i++) {
            outputVertices[i] = potentialVertices[selectedOption, i];
        }
        for (int i = 0; i < outICount; i++) {
            outputSegments[i] = potentialSegments[selectedOption, i];
        }
    }

    // TODO: detect pair count
    // TODO: detect sharp feature for each pair (if angle above threshold)
    // TODO: if two pairs, resolve ambiguity
}

public void CubicalMarchingSquares() {
    Vector3 offset = new Vector3(-8.0f, -8.0f, -8.0f);

    int gridSize = 16;

    float[,,] samples = new float[gridSize+1, gridSize+1, gridSize+1];
    //Vector3[,,] normals = new Vector3[gridSize+1, gridSize+1, gridSize+1];

    for (int i = 0; i < gridSize+1; i++)
    for (int j = 0; j < gridSize+1; j++) 
    for (int k = 0; k < gridSize+1; k++) {
        Vector3 p = new Vector3(i, j, k) + offset;
        samples[i, j, k] = Evaluate(p);
        //normals[i, j, k] = sdTorusNormal(samplePos);

        /*if (j > 5 && j < 11) {
            var sphere = new Sphere();
            sphere.Radius = 0.05f;
            sphere.Color = samples[i, j, k] < 0.0f ? Color.ORANGE : Color.BLUE;
            sphere.Transform.Pos = p;
            world.CreateInstantly(sphere);
        }*/
    }

    int segmentCount = 0;
    // edge indices within face quad
    int[] segmentBuffer = new int[4];

    float[] faceSampleBuf = new float[4];

    int[] vertexEdgeBuffer = new int[4*6];

    // DEBUG
    int[] vertexFaceBuffer = new int[4*6];

    for (int i = 0; i < gridSize; i++)
    for (int j = 0; j < gridSize; j++) 
    for (int k = 0; k < gridSize; k++) {
        float[] cornerSamples = new float[8];
        for (int c = 0; c < CMS.cornerOffsetsI.Length; c++) {
            var o = CMS.cornerOffsetsI[c];
            cornerSamples[c] = samples[i + o.x, j + o.y, k + o.z];
        }

        int vIdx = 0;
        for (int faceId = 0; faceId < CMS.faceCubeCorners.GetLength(0); faceId++) {
            faceSampleBuf[0] = cornerSamples[CMS.faceCubeCorners[faceId, 0]];
            faceSampleBuf[1] = cornerSamples[CMS.faceCubeCorners[faceId, 1]];
            faceSampleBuf[2] = cornerSamples[CMS.faceCubeCorners[faceId, 2]];
            faceSampleBuf[3] = cornerSamples[CMS.faceCubeCorners[faceId, 3]];

            MarchSquare(faceSampleBuf, segmentBuffer, out segmentCount);
            for (int eIdx = 0; eIdx < segmentCount; eIdx++) {
                vertexFaceBuffer[vIdx] = faceId; // DEBUG
                vertexEdgeBuffer[vIdx++] = CMS.faceEdgeToCubeEdge[faceId, segmentBuffer[eIdx]];
            }
        }

        Vector3[] vertexPositions = new Vector3[vIdx];
        for (int vi = 0; vi < vIdx; vi++) {
            var edge = vertexEdgeBuffer[vi];
            var face = vertexFaceBuffer[vi]; // DEBUG
            var (c0, c1) = CMS.cubeEdgeCorners[edge];
            var p0 = CMS.cornerOffsets[c0];
            var p1 = CMS.cornerOffsets[c1];
            var p = (p0 + p1) * 0.5f;

            var worldP = p + new Vector3(i, j, k) + offset;
            /*var sphere = new Sphere();
            sphere.Radius = 0.05f;
            sphere.Color = Color.ORANGE;
            sphere.Transform.Pos = worldP;
            world.CreateInstantly(sphere);*/
            vertexPositions[vi] = worldP;

            if (vi % 2 == 1) {
                var line = new Line3D();
                line.Color = cornerColors[face]; // DEBUG
                line.Vertices = [vertexPositions[vi-1], vertexPositions[vi]];
                line.Width = 4.0f;
                //world.CreateInstantly(line);
            }
        }
    }
}

public async Task DoCMS() {
    List<(Vector3, Vector3)> intersectionsInternal = new();
    List<(Vector3, Vector3)> intersectionsFace = new();
    List<(Vector3, Vector3)> intersectionNormals = new();

    var tree = new CMSTree();
    var sw = new System.Diagnostics.Stopwatch();
    sw.Start();
    tree.Init(8, 16.0f, new Vector3(-8.0f, -8.0f, -8.0f), GetIntersections);
    Debug.Log($"CMS tree init took {sw.ElapsedMilliseconds} ms");
    sw.Restart();
    tree.CalculateIntersections(tree.root, GetIntersections);
    Debug.Log($"CMS tree intersections took {sw.ElapsedMilliseconds} ms");
    void edgeDebug() {
        void showEdgeTrees(CMSCell cell) {
            if (cell.children != null) {
                foreach (var c in cell.children) {
                    showEdgeTrees(c);
                }
            }
            void showEdgeTree(EdgeNode n) {
                if (n == null) return;
                if (n.first != null) {
                    showEdgeTree(n.first);
                    showEdgeTree(n.second);
                } else {
                    /*var line = new Line3D();
                    line.Color = Color.Random();
                    line.Vertices = [n.s, n.e];
                    line.Width = 2.0f;
                    world.CreateInstantly(line);*/
                    if (n.isInternal) {
                        intersectionsInternal.Add((n.s, n.e));
                    } else {
                        intersectionsFace.Add((n.s, n.e));
                    }
                    if (n.intersectionT.HasValue) {
                        intersectionNormals.Add((Vector3.Lerp(n.s, n.e, n.intersectionT.Value), n.intersectionNormal));
                    }
                }
            }
            foreach (var ft in cell.faceTrees) {
                if (ft == null) continue;
                foreach (var et in ft.edgeTrees) {
                    showEdgeTree(et);
                }
            }
        }
        foreach(var c in tree.root.children) {
            showEdgeTrees(c);
        }
    }
    edgeDebug();
    var intrLines = new Line3D();
    intrLines.Color = Color.RED;
    intrLines.Vertices = intersectionsInternal.SelectMany(i => new Vector3[] { i.Item1, i.Item2 }).ToArray();
    intrLines.Width = 2.0f;
    //world.CreateInstantly(intrLines);

    var faceLines = new Line3D();
    faceLines.Color = Color.VIOLET;
    faceLines.Vertices = intersectionsFace.SelectMany(i => new Vector3[] { i.Item1, i.Item2 }).ToArray();
    faceLines.Width = 2.0f;
    //world.CreateInstantly(faceLines);

    var normalLines = new Line3D();
    normalLines.Color = Color.GREEN;
    normalLines.Vertices = intersectionNormals.SelectMany(i => new Vector3[] { i.Item1, i.Item1 + 0.3f*i.Item2 }).ToArray();
    normalLines.Width = 3.0f;
    //world.CreateInstantly(normalLines);

    //await Time.WaitSeconds(1.0f);
    //return;

    List<CMSCell> cells = new List<CMSCell>();
    tree.getLeafCells(tree.root, cells);
    foreach (var cell in cells) {
        if (cell.size == 2.0f) continue;
        var c = world.Clone(cube);
        c.Transform.Pos = cell.min;
        c.Transform.Scale = new Vector3(cell.size, cell.size, cell.size);
        //world.CreateInstantly(c);
    }

    List<(Vector3 s, Vector3 e)> segments = new();

    Vector3[] meshVertices;
    int[] meshIndices;
    sw.Restart();
    tree.ExtractSurface(out meshVertices, out meshIndices);
    Debug.Log($"CMS tree surface extraction took {sw.ElapsedMilliseconds} ms");
    sw.Stop();

    var mesh = new Mesh();
    mesh.Vertices = meshVertices;
    mesh.Indices = meshIndices.Select(i => (uint)i).ToArray();
    mesh.Color = new Color(0.2f, 0.2f, 1.0f, 1.0f);
    world.CreateInstantly(mesh);


    await Time.WaitSeconds(1.0f);
}

//List<(Vector3, Vector3)> normals = new();

Cube cube;

public async Task Animation(World world, Animator animator) {
    Test();

    this.world = world;
    this.animator = animator;

    var cam = world.ActiveCamera;
    cam.Transform.Pos = new Vector3(0.0f, 0.0f, -5.0f);

    var c1 = new Sphere();
    c1.Radius = 0.05f;
    c1.Color = Color.YELLOW;
    c1.Transform.Pos = new Vector3(-2.5f, 0.0f, 0.5f);
    for (int i = 0; i < 8; ++i) {
        var s = world.Clone(c1);
        s.Transform.Pos = CMS.cornerOffsets[i] + new Vector3(-2.5f, 0.0f, 0.0f);
        s.Color = cornerColors[i];
        world.CreateInstantly(s);
    }

    cube = new Cube();
    cube.Transform.Pos = new Vector3(-2.0f, 0.5f, 0.5f);
    cube.Color = Color.TRANSPARENT;
    cube.Outline = Color.BLACK;
    world.CreateInstantly(cube);

    var quad = new Quad();
    quad.Vertices = (new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(1, 1, 0), new Vector3(0, 1, 0));

    for (int i = 0; i < 6; i++) {
        var q = world.Clone(quad);
        q.Transform.Pos = CMS.quadOffsets[i];
        world.CreateInstantly(q);

        for (int j = 0; j < 4; j++) {
            var s = world.Clone(c1);
            s.Transform.Pos = CMS.quadOffsets[i] + new Vector3(CMS.quadCorners[j], 0.0f);
            var c = cornerColors[CMS.faceCubeCorners[i, j]];
            s.Color = c;
            world.CreateInstantly(s);
        }
    }

    var sphere = world.Clone(c1);
    sphere.Color = Color.YELLOW;
    world.CreateInstantly(sphere);

    CubicalMarchingSquares();
    //await DoCMS();

    await Time.WaitSeconds(3.0f);
}


Line3D make3DGrid(Vector3 min, float step, int size) {
    var grid = new Line3D(mode: MeshVertexMode.Segments);
    grid.Color = new Color(0.05f, 0.05f, 0.05f, 1.0f);
    grid.Width = 2.0f;
    grid.Vertices = new Vector3[6*(size+1)*(size+1)];
    int idx = 0;
    for (int j = 0; j < size+1; j++)
    for (int i = 0; i < size+1; i++) {
        grid.Vertices[idx++] = min + new Vector3(i*step, j*step, 0.0f);
        grid.Vertices[idx++] = min + new Vector3(i*step, j*step, size*step);

        grid.Vertices[idx++] = min + new Vector3(i*step, 0.0f, j*step);
        grid.Vertices[idx++] = min + new Vector3(i*step, size*step, j*step);

        grid.Vertices[idx++] = min + new Vector3(0.0f, i*step, j*step);
        grid.Vertices[idx++] = min + new Vector3(size*step, i*step, j*step);
    }
    return grid;
}

void Test() {
    var p = CMS.FacePosToCubePos(3, new Vector2(0.25f, 0.25f));
    //Debug.Assert(p == new Vector3(0.0f, 0.25f, 0.25f));
}
}
